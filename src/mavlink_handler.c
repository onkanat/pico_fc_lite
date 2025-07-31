#include "mavlink_handler.h"
#include "hardware/uart.h"

// Include the MAVLink C library headers
// Note: You must add the path to the mavlink library to your include paths.
#include "mavlink.h"

// --- Defines ---
#define UART_ID uart0
#define BAUD_RATE 57600
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// MAVLink system and component IDs
#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID MAV_COMP_ID_AUTOPILOT1

// --- Public Functions ---

void mavlink_handler_init() {
    // Initialize UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

void mavlink_send_heartbeat() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_heartbeat_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        MAV_TYPE_FIXED_WING,
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_FLAG_MANUAL_INPUT_ENABLED,
        0, // custom_mode
        MAV_STATE_ACTIVE
    );

    // Copy the message to a send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the buffer over UART
    uart_write_blocking(UART_ID, buf, len);
}

void mavlink_send_attitude(float roll, float pitch, float yaw) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_attitude_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        to_ms_since_boot(get_absolute_time()),
        roll,  // radians
        pitch, // radians
        yaw,   // radians
        0.0f,  // rollspeed
        0.0f,  // pitchspeed
        0.0f   // yawspeed
    );

    // Copy the message to a send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the buffer over UART
    uart_write_blocking(UART_ID, buf, len);
}

void mavlink_send_rc_channels_override(uint16_t channels[16]) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the message
    mavlink_msg_rc_channels_override_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        0, // target system
        0, // target component
        channels[0], channels[1], channels[2], channels[3],
        channels[4], channels[5], channels[6], channels[7],
        channels[8], channels[9], channels[10], channels[11],
        channels[12], channels[13], channels[14], channels[15],
        0, // channels_override_flags_a
        0  // channels_override_flags_b
    );

    // Copy the message to a send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the buffer over UART
    uart_write_blocking(UART_ID, buf, len);
}

void mavlink_handler_update() {
    // This function would handle incoming MAVLink messages.
    // For this example, we are only sending data.
}
