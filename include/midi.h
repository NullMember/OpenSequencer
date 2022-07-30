#ifndef MIDI_H
#define MIDI_H

#include "pico/stdlib.h"

class Midi
{
private:
    void (*reader)(uint8_t*, uint16_t) = nullptr;
    void (*writer)(uint8_t*, uint16_t) = nullptr;

    uint16_t buffer_size = 0;
    uint8_t* input_buffer = nullptr;
    uint8_t output_buffer[3];
    uint16_t length = 0;
    uint8_t expect = 0;
public:
    Midi(uint16_t _buffer_size, void (*_reader)(uint8_t*, uint16_t), void (*writer)(uint8_t*, uint16_t));
    ~Midi();

    void process_midi(uint8_t _data);
    void process_midi(uint8_t* _buffer, uint8_t _length);

    void send_raw(uint8_t* _buffer, uint16_t _length);
    void send_note_off(uint8_t _note, uint8_t _velocity = 0, uint8_t _channel = 0);
    void send_note_on(uint8_t _note, uint8_t _velocity, uint8_t _channel = 0);
    void send_aftertouch(uint8_t _note, uint8_t _touch, uint8_t _channel = 0);
    void send_control_change(uint8_t _control, uint8_t _value, uint8_t _channel = 0);
    void send_clock();
};

#endif // MIDI_H