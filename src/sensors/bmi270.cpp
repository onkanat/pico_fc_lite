#include "bmi270.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>

// BMI270 Register Map
#define BMI270_REG_CHIP_ID      0x00
#define BMI270_REG_STATUS       0x03
#define BMI270_REG_ACC_DATA     0x0C
#define BMI270_REG_GYR_DATA     0x12
#define BMI270_REG_ACC_CONF     0x40
#define BMI270_REG_GYR_CONF     0x42
#define BMI270_REG_PWR_CONF     0x7C
#define BMI270_REG_PWR_CTRL     0x7D
#define BMI270_REG_CMD          0x7E

// Expected Chip ID
#define BMI270_CHIP_ID_VAL      0x24

BMI270::BMI270(i2c_inst_t* i2c_port, uint8_t i2c_addr) {
    _i2c_port = i2c_port;
    _i2c_addr = i2c_addr;
}

bool BMI270::init() {
    uint8_t chip_id = 0;
    if (!read_registers(BMI270_REG_CHIP_ID, &chip_id, 1) || chip_id != BMI270_CHIP_ID_VAL) {
        printf("BMI270: Chip ID mismatch. Found 0x%02X\n", chip_id);
        return false;
    }

    // --- Simplified Initialization Sequence ---
    // A full init would require loading a config file, which is complex.
    // This basic sequence enables the sensors in normal mode.

    // 1. Disable advanced power save
    if (!write_register(BMI270_REG_PWR_CONF, 0x00)) return false;
    sleep_ms(1); // Wait for transition

    // 2. Enable Accelerometer and Gyroscope
    // ACC_EN=1, GYR_EN=1, TEMP_EN=0, AUX_EN=0 -> 0b00001110 -> 0x0E
    if (!write_register(BMI270_REG_PWR_CTRL, 0x0E)) return false;
    sleep_ms(50); // Wait for sensors to power up

    // 3. Configure Accelerometer (±8g, normal mode)
    // ODR = 100Hz (0x08), BWP = normal (0b10 << 4)
    if (!write_register(BMI270_REG_ACC_CONF, 0x28)) return false;
    sleep_ms(1);

    // 4. Configure Gyroscope (±2000dps, normal mode)
    // ODR = 200Hz (0x09), BWP = normal (0b10 << 4), noise performance enabled (0b01 << 6)
    if (!write_register(BMI270_REG_GYR_CONF, 0xA9)) return false;
    sleep_ms(1);

    printf("BMI270: Initialization successful.\n");
    return true;
}

bool BMI270::read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[12];

    // Read 12 bytes starting from the first accelerometer data register
    if (!read_registers(BMI270_REG_ACC_DATA, buffer, 12)) {
        return false;
    }

    // Combine the two bytes for each axis into a 16-bit signed integer
    accel[0] = (int16_t)((buffer[1] << 8) | buffer[0]); // X
    accel[1] = (int16_t)((buffer[3] << 8) | buffer[2]); // Y
    accel[2] = (int16_t)((buffer[5] << 8) | buffer[4]); // Z

    gyro[0] = (int16_t)((buffer[7] << 8) | buffer[6]); // X
    gyro[1] = (int16_t)((buffer[9] << 8) | buffer[8]); // Y
    gyro[2] = (int16_t)((buffer[11] << 8) | buffer[10]); // Z

    return true;
}

// --- Private Helper Methods ---

bool BMI270::write_register(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    int result = i2c_write_blocking(_i2c_port, _i2c_addr, buf, 2, false);
    return result == 2;
}

bool BMI270::read_registers(uint8_t reg, uint8_t* data, size_t len) {
    // Write the register address to start reading from
    int write_result = i2c_write_blocking(_i2c_port, _i2c_addr, &reg, 1, true); // true to keep control of bus
    if (write_result != 1) {
        return false;
    }

    // Read the data from the specified register
    int read_result = i2c_read_blocking(_i2c_port, _i2c_addr, data, len, false);
    return read_result == len;
}
