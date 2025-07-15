#include "pid_controller.h"
#include <stddef.h>

// Initialize PID controller parameters
void pid_init(PID_Controller *pid, float kp, float ki, float kd) {
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit_max = 1.0f;  // Default limits
    pid->integral_limit_min = -1.0f;
    pid->output_limit_max = 1.0f;
    pid->output_limit_min = -1.0f;
}

// Reset the integral and previous error terms
void pid_reset(PID_Controller *pid) {
    if (pid == NULL) return;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// Update the PID controller and return the control output
float pid_update(PID_Controller *pid, float measurement, float dt) {
    if (pid == NULL || dt <= 0.0f) return 0.0f;

    // Calculate error
    float error = pid->setpoint - measurement;

    // Proportional term
    float p_term = pid->kp * error;

    // Integral term with anti-windup
    pid->integral += pid->ki * error * dt;
    if (pid->integral > pid->integral_limit_max) {
        pid->integral = pid->integral_limit_max;
    } else if (pid->integral < pid->integral_limit_min) {
        pid->integral = pid->integral_limit_min;
    }
    float i_term = pid->integral;

    // Derivative term
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;

    // Update previous error for next iteration
    pid->prev_error = error;

    // Calculate total output
    float output = p_term + i_term + d_term;

    // Clamp output to defined limits
    if (output > pid->output_limit_max) {
        output = pid->output_limit_max;
    } else if (output < pid->output_limit_min) {
        output = pid->output_limit_min;
    }

    return output;
}

// Set new PID gains
void pid_set_gains(PID_Controller *pid, float kp, float ki, float kd) {
    if (pid == NULL) return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

// Set output and integral limits
void pid_set_limits(PID_Controller *pid, float out_min, float out_max, float int_min, float int_max) {
    if (pid == NULL) return;
    pid->output_limit_min = out_min;
    pid->output_limit_max = out_max;
    pid->integral_limit_min = int_min;
    pid->integral_limit_max = int_max;
}
