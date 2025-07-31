#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "sensors/bmi270.h"
#include "sensors/bmp388.h"
#include "sensors/gps.h"

// I2C definitions
#define I2C_PORT i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5

// UART definitions for GPS
#define GPS_UART uart1
#define GPS_TX_PIN 8
#define GPS_RX_PIN 9
#define GPS_BAUDRATE 9600

int main() {
    // Initialize standard I/O
    stdio_init_all();
    printf("======================================\n");
    printf("     Pico FC Lite - All Sensors Test\n");
    printf("======================================\n");

    // Initialize I2C for IMU and Barometer
    printf("Initializing I2C...\n");
    i2c_init(I2C_PORT, 400 * 1000); // 400 kHz
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("I2C initialized on pins %d (SDA) and %d (SCL)\n", I2C_SDA_PIN, I2C_SCL_PIN);

    // Initialize BMI270 IMU
    printf("\nInitializing BMI270 IMU...\n");
    BMI270 imu(I2C_PORT);
    bool imu_ready = imu.init();
    if (!imu_ready) {
        printf("❌ IMU initialization failed!\n");
    } else {
        printf("✅ BMI270 IMU initialized successfully\n");
    }

    // Initialize BMP388 Barometer
    printf("\nInitializing BMP388 Barometer...\n");
    BMP388 barometer(I2C_PORT);
    bool baro_ready = barometer.init();
    if (!baro_ready) {
        printf("❌ Barometer initialization failed!\n");
    } else {
        printf("✅ BMP388 Barometer initialized successfully\n");
    }

    // Initialize GPS
    printf("\nInitializing GPS...\n");
    GPS gps(GPS_UART, GPS_TX_PIN, GPS_RX_PIN, GPS_BAUDRATE);
    bool gps_ready = gps.init();
    if (!gps_ready) {
        printf("❌ GPS initialization failed!\n");
    } else {
        printf("✅ GPS initialized successfully on UART1 pins %d/%d\n", GPS_TX_PIN, GPS_RX_PIN);
    }

    printf("\n======================================\n");
    printf("Starting sensor readings...\n");
    printf("======================================\n");

    // Sensor data variables
    int16_t accel[3], gyro[3];
    int32_t raw_pressure, raw_temperature;
    float pressure_pa, temperature_c;
    GPS_Data gps_data;

    uint32_t loop_count = 0;
    uint32_t last_gps_print = 0;

    // Main sensor reading loop
    while (1) {
        loop_count++;
        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        printf("\n--- Loop %lu (Time: %lu ms) ---\n", loop_count, current_time);

        // Read IMU data
        if (imu_ready && imu.read_raw(accel, gyro)) {
            printf("🎯 IMU - Accel: X=%6d, Y=%6d, Z=%6d | Gyro: X=%6d, Y=%6d, Z=%6d\n", 
                   accel[0], accel[1], accel[2], 
                   gyro[0], gyro[1], gyro[2]);
        } else if (imu_ready) {
            printf("❌ Failed to read IMU data\n");
        }

        // Read Barometer data
        if (baro_ready) {
            if (barometer.read_raw(&raw_pressure, &raw_temperature)) {
                printf("🌡️  BARO - Raw Pressure: %ld, Raw Temperature: %ld\n", 
                       raw_pressure, raw_temperature);
                
                // Also try calibrated readings
                if (barometer.read(&pressure_pa, &temperature_c)) {
                    printf("🌡️  BARO - Pressure: %.2f Pa, Temperature: %.2f °C\n", 
                           pressure_pa, temperature_c);
                }
            } else {
                printf("❌ Failed to read barometer data\n");
            }
        }

        // Update and read GPS data (less frequent printing)
        if (gps_ready) {
            gps.update();
            
            // Print GPS data every 5 seconds
            if (current_time - last_gps_print > 5000) {
                if (gps.get_data(&gps_data)) {
                    printf("🛰️  GPS - Fix: %s, Lat: %.6f, Lon: %.6f, Alt: %.1fm\n",
                           gps_data.fix_valid ? "VALID" : "INVALID",
                           gps_data.latitude, gps_data.longitude, gps_data.altitude);
                    printf("🛰️  GPS - Speed: %.1f km/h, Course: %.1f°, Satellites: %d\n",
                           gps_data.speed, gps_data.course, gps_data.satellites);
                } else {
                    printf("🛰️  GPS - No valid fix available\n");
                }
                last_gps_print = current_time;
            }
        }

        // Read sensors at 10 Hz
        sleep_ms(100);
    }

    return 0;
}
