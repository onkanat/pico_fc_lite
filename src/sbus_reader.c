#include <stdio.h>
#include "sbus_reader.h"
#include "sbus_reader.pio.h"

// --- Static variables ---
static PIO pio_instance;
static uint sm_instance;
static uint pio_offset;

// Double buffer for SBUS frames
// One is being filled by the PIO task, the other can be safely read by the main loop
static volatile uint16_t sbus_channels[2][SBUS_NUM_CHANNELS];
static volatile bool sbus_failsafe[2];
static volatile bool sbus_frame_lost[2];
static volatile int active_buffer = 0;

// Raw frame buffer
static uint8_t raw_frame[SBUS_FRAME_SIZE];

// --- Private functions ---

// Parses a raw 25-byte SBUS frame into 16 channels.
static void sbus_parse_frame() {
    int write_buffer = 1 - active_buffer;

    // Decode the 16 channels (11 bits each)
    sbus_channels[write_buffer][0]  = (uint16_t)(((raw_frame[1]    | raw_frame[2] << 8) & 0x07FF));
    sbus_channels[write_buffer][1]  = (uint16_t)(((raw_frame[2] >> 3 | raw_frame[3] << 5) & 0x07FF));
    sbus_channels[write_buffer][2]  = (uint16_t)(((raw_frame[3] >> 6 | raw_frame[4] << 2 | raw_frame[5] << 10) & 0x07FF));
    sbus_channels[write_buffer][3]  = (uint16_t)(((raw_frame[5] >> 1 | raw_frame[6] << 7) & 0x07FF));
    sbus_channels[write_buffer][4]  = (uint16_t)(((raw_frame[6] >> 4 | raw_frame[7] << 4) & 0x07FF));
    sbus_channels[write_buffer][5]  = (uint16_t)(((raw_frame[7] >> 7 | raw_frame[8] << 1 | raw_frame[9] << 9) & 0x07FF));
    sbus_channels[write_buffer][6]  = (uint16_t)(((raw_frame[9] >> 2 | raw_frame[10] << 6) & 0x07FF));
    sbus_channels[write_buffer][7]  = (uint16_t)(((raw_frame[10] >> 5 | raw_frame[11] << 3) & 0x07FF));
    sbus_channels[write_buffer][8]  = (uint16_t)(((raw_frame[12]   | raw_frame[13] << 8) & 0x07FF));
    sbus_channels[write_buffer][9]  = (uint16_t)(((raw_frame[13] >> 3 | raw_frame[14] << 5) & 0x07FF));
    sbus_channels[write_buffer][10] = (uint16_t)(((raw_frame[14] >> 6 | raw_frame[15] << 2 | raw_frame[16] << 10) & 0x07FF));
    sbus_channels[write_buffer][11] = (uint16_t)(((raw_frame[16] >> 1 | raw_frame[17] << 7) & 0x07FF));
    sbus_channels[write_buffer][12] = (uint16_t)(((raw_frame[17] >> 4 | raw_frame[18] << 4) & 0x07FF));
    sbus_channels[write_buffer][13] = (uint16_t)(((raw_frame[18] >> 7 | raw_frame[19] << 1 | raw_frame[20] << 9) & 0x07FF));
    sbus_channels[write_buffer][14] = (uint16_t)(((raw_frame[20] >> 2 | raw_frame[21] << 6) & 0x07FF));
    sbus_channels[write_buffer][15] = (uint16_t)(((raw_frame[21] >> 5 | raw_frame[22] << 3) & 0x07FF));

    // Decode the flags byte (byte 23)
    if (raw_frame[23] & (1 << 3)) {
        sbus_failsafe[write_buffer] = true;
    } else {
        sbus_failsafe[write_buffer] = false;
    }

    if (raw_frame[23] & (1 << 2)) {
        sbus_frame_lost[write_buffer] = true;
    } else {
        sbus_frame_lost[write_buffer] = false;
    }

    // Swap buffers
    active_buffer = write_buffer;
}

// This function should be called periodically from the main loop or a timer.
void sbus_check_for_new_frame() {
    static int sbus_frame_pos = 0;

    while (!pio_sm_is_rx_fifo_empty(pio_instance, sm_instance)) {
        uint16_t raw_char = pio_sm_get(pio_instance, sm_instance);
        uint8_t byte = (uint8_t)(raw_char & 0xFF);

        if (sbus_frame_pos == 0) {
            if (byte == 0x0F) { // SBUS start byte
                raw_frame[sbus_frame_pos++] = byte;
            }
        } else {
            raw_frame[sbus_frame_pos++] = byte;
            if (sbus_frame_pos >= SBUS_FRAME_SIZE) {
                if (raw_frame[SBUS_FRAME_SIZE - 1] == 0x00) { // SBUS end byte
                    sbus_parse_frame();
                }
                sbus_frame_pos = 0; // Reset for next frame
            }
        }
    }
}

// --- Public functions ---

void sbus_reader_init(PIO pio, uint sm, uint pin) {
    pio_instance = pio;
    sm_instance = sm;

    // Load the PIO program
    pio_offset = pio_add_program(pio, &sbus_reader_program);
    
    // Initialize the PIO program
    sbus_reader_program_init(pio, sm, pio_offset, pin);
}

bool sbus_get_latest_frame(uint16_t* channels, bool* failsafe, bool* frame_lost) {
    // Atomically read the active buffer index
    int read_buffer = active_buffer;

    // Copy data from the active buffer
    for (int i = 0; i < SBUS_NUM_CHANNELS; ++i) {
        channels[i] = sbus_channels[read_buffer][i];
    }
    *failsafe = sbus_failsafe[read_buffer];
    *frame_lost = sbus_frame_lost[read_buffer];

    return true; // Could be extended to return false if no frame received yet
}
