#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

// PID Controller State
typedef struct {
    float kp, ki, kd;            // Proportional, Integral, Derivative gains
    float setpoint;              // Desired value
    float integral;              // Integral accumulator
    float prev_error;            // Error from the previous update
    float integral_limit_max;    // Anti-windup: max integral value
    float integral_limit_min;    // Anti-windup: min integral value
    float output_limit_max;      // Max output value
    float output_limit_min;      // Min output value
} PID_Controller;

// Public Functions
void pid_init(PID_Controller *pid, float kp, float ki, float kd);
void pid_reset(PID_Controller *pid);
float pid_update(PID_Controller *pid, float measurement, float dt);
void pid_set_gains(PID_Controller *pid, float kp, float ki, float kd);
void pid_set_limits(PID_Controller *pid, float out_min, float out_max, float int_min, float int_max);

#endif // PID_CONTROLLER_H
