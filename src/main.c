#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pio.h"

#include "sbus_reader.h"
#include "pid_controller.h"
#include "mavlink_handler.h"

// --- Defines ---
#define SBUS_PIN 13 // Use GPIO 13 for SBUS input

// --- Global Variables ---
PID_Controller roll_pid, pitch_pid, yaw_pid;

// --- Main Application ---
int main() {
    // Initialize stdio
    stdio_init_all();

    // --- Module Initialization ---
    // 1. SBUS Reader
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    sbus_reader_init(pio, sm, SBUS_PIN);

    // 2. PID Controllers
    // Gains are placeholders, they need to be tuned.
    pid_init(&roll_pid, 1.0f, 0.1f, 0.05f);
    pid_init(&pitch_pid, 1.0f, 0.1f, 0.05f);
    pid_init(&yaw_pid, 0.8f, 0.05f, 0.02f);
    // Set some example limits
    pid_set_limits(&roll_pid, -1.0f, 1.0f, -0.5f, 0.5f);
    pid_set_limits(&pitch_pid, -1.0f, 1.0f, -0.5f, 0.5f);
    pid_set_limits(&yaw_pid, -1.0f, 1.0f, -0.5f, 0.5f);

    // 3. MAVLink Handler
    mavlink_handler_init();

    // --- Timing Variables ---
    uint32_t last_mavlink_time = 0;
    const uint32_t mavlink_heartbeat_interval_ms = 1000;
    const uint32_t mavlink_attitude_interval_ms = 100; // 10 Hz

    printf("Pico FC Lite - Initialization Complete\n");

    // --- Main Loop ---
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // 1. Check for new SBUS data
        sbus_check_for_new_frame(); // This should be called frequently

        // 2. Get latest RC channels data
        uint16_t rc_channels[SBUS_NUM_CHANNELS];
        bool failsafe, frame_lost;
        if (sbus_get_latest_frame(rc_channels, &failsafe, &frame_lost)) {
            // For now, let's just print the first 4 channels
            // In a real FC, you would map these to setpoints for the PID controllers
            // e.g., roll_pid.setpoint = map_rc_to_angle(rc_channels[0]);
        }

        // 3. Run PID controllers (example with dummy data)
        // This requires an IMU to get actual roll, pitch, yaw measurements.
        float current_roll = 0.0f; // Replace with IMU data
        float dt = 0.01f; // Should be calculated based on actual loop time
        float roll_output = pid_update(&roll_pid, current_roll, dt);
        // ... similar for pitch and yaw

        // 4. Send MAVLink Telemetry
        if (now - last_mavlink_time > mavlink_heartbeat_interval_ms) {
            mavlink_send_heartbeat();
            last_mavlink_time = now;
        }
        if (now - last_mavlink_time > mavlink_attitude_interval_ms) {
            // Send dummy attitude data for now
            mavlink_send_attitude(0.1f, -0.2f, 0.3f);
        }

        // A small delay to prevent the loop from running too fast
        sleep_ms(10);
    }

    return 0; // Should not be reached
}
