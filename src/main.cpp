#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "sensors/bmi270.h"

// I2C definitions
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

int main() {
    // Initialize standard I/O
    stdio_init_all();
    printf("Pico FC Lite - BMI270 Test\n");

    // Initialize I2C
    i2c_init(I2C_PORT, 400 * 1000); // 400 kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Initialize BMI270
    BMI270 imu(I2C_PORT);
    if (!imu.init()) {
        printf("IMU initialization failed. Halting.\n");
        while (1);
    }

    int16_t accel[3], gyro[3];

    // Main loop
    while (1) {
        if (imu.read_raw(accel, gyro)) {
            printf("Accel: X=%6d, Y=%6d, Z=%6d | Gyro: X=%6d, Y=%6d, Z=%6d\n", 
                   accel[0], accel[1], accel[2], 
                   gyro[0], gyro[1], gyro[2]);
        } else {
            printf("Failed to read from IMU.\n");
        }
        sleep_ms(100); // Read data at 10 Hz
    }

    return 0;
}
