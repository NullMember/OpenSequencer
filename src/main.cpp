// Include required Pico libraries
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico/time.h"

// Include USB stack
#include "tusb.h"

// Include custom libraries
#include "midi.h"
#include "bootsel-reboot.hpp"

struct gpio_delayed_t
{
    uint pin;
    bool state;
};

#define MIDI_BUFFER_SIZE 1024

#define MIDI_UART_ID uart0
#define MIDI_UART_BAUD 31250
#define MIDI_UART_PARITY UART_PARITY_NONE
#define MIDI_UART_DATA_BITS 8
#define MIDI_UART_STOP_BITS 1
#define MIDI_UART_TX_PIN 0
#define MIDI_UART_RX_PIN 1
#define MIDI_UART_IRQ MIDI_UART_ID == uart0 ? UART0_IRQ : UART1_IRQ

enum clock_source_t {
    CLOCK_SOURCE_INTERNAL,
    CLOCK_SOURCE_EXTERNAL,
    CLOCK_SOURCE_UART_MIDI,
    CLOCK_SOURCE_USB_MIDI
};

struct sync_t
{
    clock_source_t source;
    float ppq;
    float bpm;
    int64_t period_us;
};
#define SYNC_PPQ 24
#define SYNC_IN_PIN 2
#define SYNC_OUT_PIN 3
sync_t sync_data;

#define GATE_OUT_PIN 4

Midi* uart_midi = nullptr;
Midi* usb_midi = nullptr;

#define SEQUENCER_STEP_COUNT 16
#define SEQUENCER_TRIGGER_COUNT 8
#define SEQUENCER_TRIGGER_PIN_START 8
#define SEQUENCER_TRIGGER_NOTE_START 36
uint8_t** sequencer_trigger_states;
uint8_t sequencer_midi_channel = 0;

int64_t gpio_put_delayed_alarm(alarm_id_t _id, void *_user_data){
    gpio_delayed_t* data = (gpio_delayed_t *)_user_data;
    gpio_put(data->pin, data->state);
    delete data;
    return 0;
}

alarm_id_t gpio_put_delayed(uint _pin, bool _state, uint64_t _us){
    gpio_delayed_t* data = new gpio_delayed_t;
    data->pin = _pin;
    data->state = _state;
    return alarm_pool_add_alarm_in_us(alarm_pool_get_default(), _us, gpio_put_delayed_alarm, data, true);
}

void sync_clock_driver(){
    static uint64_t counter_clock = 0;
    static uint64_t counter_step = 0;
    static uint64_t pulse_t_last = 0;
    static uint64_t pulse_t_current = 0;
    static uint64_t pulse_period = 0;

    pulse_t_last = pulse_t_current;
    pulse_t_current = time_us_64();
    pulse_period = pulse_t_current - pulse_t_last;

    uart_midi->send_clock();
    usb_midi->send_clock();
    gpio_put(SYNC_OUT_PIN, true);
    gpio_put_delayed(SYNC_OUT_PIN, false, pulse_period >> 1);

    if ((++counter_clock % 6) == 0)
    {
        gpio_put(GATE_OUT_PIN, true);
        gpio_put_delayed(GATE_OUT_PIN, false, pulse_period >> 1);
        ++counter_step;
        for (auto i = 0; i < SEQUENCER_TRIGGER_COUNT; i++)
        {
            uint8_t velocity = sequencer_trigger_states[i][counter_step % SEQUENCER_STEP_COUNT];
            if (velocity)
            {
                gpio_put(SEQUENCER_TRIGGER_PIN_START + i, true);
                gpio_put_delayed(SEQUENCER_TRIGGER_PIN_START + i, false, pulse_period >> 1);
            }
            uart_midi->send_note_on(SEQUENCER_TRIGGER_NOTE_START + i, velocity, sequencer_midi_channel);
            usb_midi->send_note_on(SEQUENCER_TRIGGER_NOTE_START + i, velocity, sequencer_midi_channel);
        }
    }
}

void uart_midi_reader(uint8_t* buffer, uint16_t length){
    if (length == 1)
    {
        uint8_t command = buffer[0];
        if (command == 0xF8)
        {
            if (sync_data.source == CLOCK_SOURCE_UART_MIDI)
            {
                sync_clock_driver();
            }
        }
    }
    if (length == 3)
    {
        uint8_t command = buffer[0] & 0xF0;
        uint8_t channel = buffer[0] & 0x0F;
        if (command == 0x90)
        {
            uint8_t note = buffer[1];
            uint8_t velocity = buffer[2];
            if (channel >= 8)
            {
                sequencer_trigger_states[channel - 8][note] = velocity;
            }
        }
    }
}

void uart_midi_writer(uint8_t* buffer, uint16_t length){
    uart_write_blocking(MIDI_UART_ID, buffer, length);
}

void usb_midi_reader(uint8_t* buffer, uint16_t length){
    if (length == 1)
    {
        uint8_t command = buffer[0];
        if (command == 0xF8)
        {
            if (sync_data.source == CLOCK_SOURCE_USB_MIDI)
            {
                sync_clock_driver();
            }
        }
    }
    if (length == 3)
    {
        uint8_t command = buffer[0] & 0xF0;
        uint8_t channel = buffer[0] & 0x0F;
        if (command == 0x90)
        {
            uint8_t note = buffer[1];
            uint8_t velocity = buffer[2];
            if (channel >= 8)
            {
                sequencer_trigger_states[channel - 8][note] = velocity;
            }
        }
    }
}

void usb_midi_writer(uint8_t* buffer, uint16_t length){
    tud_midi_stream_write(0, buffer, length);
}

void uart_midi_handler(){
    while (uart_is_readable(MIDI_UART_ID))
    {
        uart_midi->process_midi(uart_getc(MIDI_UART_ID));
    }
}

// usb_midi_handler
void tud_midi_rx_cb(uint8_t itf){
    static uint8_t buffer[MIDI_BUFFER_SIZE];

    // Perform USB MIDI Task
    if(tud_midi_available())
    {
        auto result = tud_midi_stream_read(buffer, MIDI_BUFFER_SIZE);
        usb_midi->process_midi(buffer, result);
    }
}

int64_t sync_internal_clock_handler(alarm_id_t _id, void *_user_data){
    if(sync_data.source == CLOCK_SOURCE_INTERNAL){
        sync_clock_driver();
    }
    
    return -(sync_data.period_us);
}

void sync_external_clock_handler(uint gpio, uint32_t events){
    if (gpio == SYNC_IN_PIN)
    {
        if (sync_data.source == CLOCK_SOURCE_EXTERNAL)
        {
            if (events & GPIO_IRQ_EDGE_RISE)
            {
                sync_clock_driver();
            }
        }
    }

    gpio_acknowledge_irq(gpio, events);
}

int main(int argc, char const *argv[])
{
    // Initialize onboard led for debug purpose
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, true);

    // Initialize uart0 for MIDI I/O
    gpio_set_function(MIDI_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(MIDI_UART_RX_PIN, GPIO_FUNC_UART);
    uart_init(MIDI_UART_ID, MIDI_UART_BAUD);
    uart_set_hw_flow(MIDI_UART_ID, false, false);
    uart_set_format(MIDI_UART_ID, MIDI_UART_DATA_BITS, MIDI_UART_STOP_BITS, MIDI_UART_PARITY);
    uart_set_fifo_enabled(MIDI_UART_ID, true);
    irq_set_exclusive_handler(MIDI_UART_IRQ, uart_midi_handler);
    irq_set_enabled(MIDI_UART_IRQ, true);
    uart_set_irq_enables(MIDI_UART_ID, true, false);

    // Construct midi objects
    uart_midi = new Midi(MIDI_BUFFER_SIZE, uart_midi_reader, uart_midi_writer);
    usb_midi = new Midi(MIDI_BUFFER_SIZE, usb_midi_reader, usb_midi_writer);

    // Initialize sync data
    sync_data.source = CLOCK_SOURCE_INTERNAL;
    sync_data.ppq = SYNC_PPQ;
    sync_data.bpm = 60.0f;
    sync_data.period_us = (int64_t)(60000000.0 / sync_data.bpm / sync_data.ppq);

    // Initialize alarm pool
    alarm_pool_init_default();

    // Initialize internal clock generator
    gpio_init(SYNC_OUT_PIN);
    gpio_put(SYNC_OUT_PIN, false);
    gpio_set_dir(SYNC_OUT_PIN, true);
    alarm_pool_add_alarm_in_us(
        alarm_pool_get_default(),
        sync_data.period_us >> 1,
        sync_internal_clock_handler,
        nullptr,
        true);

    // Initialize external clock input
    gpio_init(SYNC_IN_PIN);
    gpio_set_irq_enabled_with_callback(
        SYNC_IN_PIN,
        GPIO_IRQ_EDGE_RISE,
        true,
        sync_external_clock_handler);
    
    // Initialize gate output
    gpio_init(GATE_OUT_PIN);
    gpio_put(GATE_OUT_PIN, false);
    gpio_set_dir(GATE_OUT_PIN, true);

    // Initialize sequencer triggers
    sequencer_trigger_states = new uint8_t*[SEQUENCER_TRIGGER_COUNT];
    for (auto i = 0; i < SEQUENCER_TRIGGER_COUNT; i++)
    {
        sequencer_trigger_states[i] = new uint8_t[SEQUENCER_STEP_COUNT];
        for (auto j = 0; j < SEQUENCER_STEP_COUNT; j++)
        {
            sequencer_trigger_states[i][j] = 0;
        }
        gpio_init(SEQUENCER_TRIGGER_PIN_START + i);
        gpio_put(SEQUENCER_TRIGGER_PIN_START + i, false);
        gpio_set_dir(SEQUENCER_TRIGGER_PIN_START + i, true);
    }

    // Setup watchdog timer
    arm_watchdog();

    // Initialize USB
    tusb_init();

    while (true)
    {
        // Check bootsel button for 
        check_bootsel_button();
        // Perform USB Device Task
        tud_task();
    }
    return 0;
}