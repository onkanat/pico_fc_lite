#ifndef SBUS_READER_H
#define SBUS_READER_H

#include "pico/stdlib.h"
#include "hardware/pio.h"

// SBUS protocol defines
#define SBUS_BAUDRATE 100000
#define SBUS_FRAME_SIZE 25
#define SBUS_NUM_CHANNELS 16

// Public functions
void sbus_reader_init(PIO pio, uint sm, uint pin);
bool sbus_get_latest_frame(uint16_t* channels, bool* failsafe, bool* frame_lost);

#endif // SBUS_READER_H
