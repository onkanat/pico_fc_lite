#ifndef BMP388_H
#define BMP388_H

#include "hardware/i2c.h"

// Default I2C address for BMP388
#define BMP388_I2C_ADDR 0x77

class BMP388 {
public:
    // Constructor
    BMP388(i2c_inst_t* i2c_port, uint8_t address = BMP388_I2C_ADDR);
    
    // Initialize the sensor
    bool init();
    
    // Read raw pressure and temperature
    bool read_raw(int32_t* pressure, int32_t* temperature);
    
    // Read calibrated values
    bool read(float* pressure_pa, float* temperature_c);

private:
    i2c_inst_t* _i2c;
    uint8_t _address;
    
    // Helper functions
    bool write_register(uint8_t reg, uint8_t data);
    bool read_register(uint8_t reg, uint8_t* data);
    bool read_registers(uint8_t reg, uint8_t* data, size_t len);
};

#endif // BMP388_H