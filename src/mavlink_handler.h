#ifndef MAVLINK_HANDLER_H
#define MAVLINK_HANDLER_H

#include "pico/stdlib.h"

// Initialize the MAVLink handler (e.g., setup UART)
void mavlink_handler_init();

// Send a heartbeat message to the ground station
void mavlink_send_heartbeat();

// Send attitude information (roll, pitch, yaw)
void mavlink_send_attitude(float roll, float pitch, float yaw);

// Send RC channels override (for debugging or control)
void mavlink_send_rc_channels_override(uint16_t channels[16]);

// This function should be called periodically to handle MAVLink communication
void mavlink_handler_update();

#endif // MAVLINK_HANDLER_H
