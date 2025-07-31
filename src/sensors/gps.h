#ifndef GPS_H
#define GPS_H

#include "hardware/uart.h"
#include "pico/stdlib.h"

// GPS data structure
typedef struct {
    double latitude;
    double longitude;
    float altitude;
    float speed;
    float course;
    uint8_t satellites;
    bool fix_valid;
} GPS_Data;

class GPS {
public:
    // Constructor
    GPS(uart_inst_t* uart_port, uint tx_pin, uint rx_pin, uint baudrate = 9600);
    
    // Initialize the GPS UART
    bool init();
    
    // Update GPS data (call regularly in main loop)
    void update();
    
    // Get latest GPS data
    bool get_data(GPS_Data* data);
    
    // Check if GPS has valid fix
    bool has_fix() const { return _last_data.fix_valid; }

private:
    uart_inst_t* _uart;
    uint _tx_pin;
    uint _rx_pin;
    uint _baudrate;
    
    GPS_Data _last_data;
    char _rx_buffer[256];
    uint _rx_index;
    
    // NMEA parsing functions
    bool parse_nmea_line(const char* line);
    bool parse_gpgga(const char* line);
    bool parse_gprmc(const char* line);
    
    // Helper functions
    double parse_coordinate(const char* str);
    float parse_float(const char* str);
    int parse_int(const char* str);
};

#endif // GPS_H