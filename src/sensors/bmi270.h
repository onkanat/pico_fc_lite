#ifndef BMI270_H
#define BMI270_H

#include "hardware/i2c.h"

// Default I2C address for BMI270
#define BMI270_I2C_ADDR 0x68

class BMI270 {
public:
    // Constructor
    BMI270(i2c_inst_t* i2c_port, uint8_t i2c_addr = BMI270_I2C_ADDR);

    // Public Methods
    bool init();
    bool read_raw(int16_t accel[3], int16_t gyro[3]);

private:
    // Private Members
    i2c_inst_t* _i2c_port;
    uint8_t _i2c_addr;

    // Private Methods
    bool write_register(uint8_t reg, uint8_t data);
    bool read_registers(uint8_t reg, uint8_t* data, size_t len);
};

#endif // BMI270_H
