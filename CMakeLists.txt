cmake_minimum_required(VERSION 3.16)

include("../../pico-sdk/pico_sdk_init.cmake")

project(sequencer C CXX ASM)

pico_sdk_init()

file(GLOB SEQUENCER_SRC_FILES "${PROJECT_SOURCE_DIR}/src/*.cpp" "${PROJECT_SOURCE_DIR}/src/*.c")

add_executable(sequencer "${SEQUENCER_SRC_FILES}")

target_include_directories(sequencer PRIVATE "${PROJECT_SOURCE_DIR}/include")
target_link_libraries(sequencer pico_stdlib hardware_pwm hardware_uart tinyusb_device tinyusb_board)

pico_add_extra_outputs(sequencer)
