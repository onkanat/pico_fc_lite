#include "bmp388.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>

// BMP388 Register Map
#define BMP388_REG_CHIP_ID      0x00
#define BMP388_REG_STATUS       0x03
#define BMP388_REG_PRESS_DATA   0x04
#define BMP388_REG_TEMP_DATA    0x07

// Expected chip ID for BMP388
#define BMP388_CHIP_ID          0x50

BMP388::BMP388(i2c_inst_t* i2c_port, uint8_t address) 
    : _i2c(i2c_port), _address(address) {
}

bool BMP388::init() {
    uint8_t chip_id;
    if (!read_register(BMP388_REG_CHIP_ID, &chip_id)) {
        printf("BMP388: Failed to read chip ID\n");
        return false;
    }
    
    if (chip_id != BMP388_CHIP_ID) {
        printf("BMP388: Invalid chip ID: 0x%02X (expected 0x%02X)\n", chip_id, BMP388_CHIP_ID);
        return false;
    }
    
    printf("BMP388: Sensor initialized successfully\n");
    return true;
}

bool BMP388::read_raw(int32_t* pressure, int32_t* temperature) {
    uint8_t data[6];
    if (!read_registers(BMP388_REG_PRESS_DATA, data, 6)) {
        return false;
    }
    
    // Convert raw data to pressure and temperature
    *pressure = (int32_t)(data[2] << 16 | data[1] << 8 | data[0]);
    *temperature = (int32_t)(data[5] << 16 | data[4] << 8 | data[3]);
    
    return true;
}

bool BMP388::read(float* pressure_pa, float* temperature_c) {
    int32_t raw_pressure, raw_temperature;
    if (!read_raw(&raw_pressure, &raw_temperature)) {
        return false;
    }
    
    // Basic conversion (needs calibration data for accuracy)
    *pressure_pa = (float)raw_pressure / 64.0f;  // Simplified conversion
    *temperature_c = (float)raw_temperature / 5120.0f;  // Simplified conversion
    
    return true;
}

bool BMP388::write_register(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    return i2c_write_blocking(_i2c, _address, buffer, 2, false) == 2;
}

bool BMP388::read_register(uint8_t reg, uint8_t* data) {
    return i2c_write_blocking(_i2c, _address, &reg, 1, true) == 1 &&
           i2c_read_blocking(_i2c, _address, data, 1, false) == 1;
}

bool BMP388::read_registers(uint8_t reg, uint8_t* data, size_t len) {
    return i2c_write_blocking(_i2c, _address, &reg, 1, true) == 1 &&
           i2c_read_blocking(_i2c, _address, data, len, false) == (int)len;
}