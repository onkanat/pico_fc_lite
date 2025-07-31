#ifndef MIXER_H
#define MIXER_H

#include "pico/stdlib.h"

// Motor output structure for quadcopter
typedef struct {
    float motor1;  // Front left
    float motor2;  // Front right  
    float motor3;  // Rear left
    float motor4;  // Rear right
} MotorOutputs;

// Control inputs structure
typedef struct {
    float throttle;  // 0.0 to 1.0
    float roll;      // -1.0 to 1.0
    float pitch;     // -1.0 to 1.0
    float yaw;       // -1.0 to 1.0
} ControlInputs;

class Mixer {
public:
    // Constructor
    Mixer();
    
    // Mix control inputs to motor outputs (quadcopter X configuration)
    void mix_quadcopter_x(const ControlInputs* inputs, MotorOutputs* outputs);
    
    // Mix control inputs to servo outputs (fixed wing)
    void mix_fixed_wing(const ControlInputs* inputs, float* aileron, float* elevator, float* rudder, float* throttle);
    
    // Apply motor output limits
    void apply_limits(MotorOutputs* outputs, float min_output = 0.0f, float max_output = 1.0f);

private:
    // Helper function to constrain values
    float constrain(float value, float min_val, float max_val);
};

#endif // MIXER_H