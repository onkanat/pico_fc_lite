#include "gps.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

GPS::GPS(uart_inst_t* uart_port, uint tx_pin, uint rx_pin, uint baudrate)
    : _uart(uart_port), _tx_pin(tx_pin), _rx_pin(rx_pin), _baudrate(baudrate), _rx_index(0) {
    // Initialize GPS data
    memset(&_last_data, 0, sizeof(_last_data));
    memset(_rx_buffer, 0, sizeof(_rx_buffer));
}

bool GPS::init() {
    // Initialize UART
    uart_init(_uart, _baudrate);
    gpio_set_function(_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(_rx_pin, GPIO_FUNC_UART);
    
    printf("GPS: UART initialized on pins %d/%d at %d baud\n", _tx_pin, _rx_pin, _baudrate);
    return true;
}

void GPS::update() {
    // Read available characters from UART
    while (uart_is_readable(_uart)) {
        char c = uart_getc(_uart);
        
        if (c == '\n' || c == '\r') {
            if (_rx_index > 0) {
                _rx_buffer[_rx_index] = '\0';
                parse_nmea_line(_rx_buffer);
                _rx_index = 0;
            }
        } else if (_rx_index < sizeof(_rx_buffer) - 1) {
            _rx_buffer[_rx_index++] = c;
        } else {
            // Buffer overflow protection - reset buffer
            _rx_index = 0;
        }
    }
}

bool GPS::get_data(GPS_Data* data) {
    if (data == nullptr) return false;
    
    *data = _last_data;
    return _last_data.fix_valid;
}

bool GPS::parse_nmea_line(const char* line) {
    if (strncmp(line, "$GPGGA", 6) == 0) {
        return parse_gpgga(line);
    } else if (strncmp(line, "$GPRMC", 6) == 0) {
        return parse_gprmc(line);
    }
    return false;
}

bool GPS::parse_gpgga(const char* line) {
    // Basic GPGGA parsing implementation
    // Format: $GPGGA,time,lat,N/S,lon,E/W,quality,satellites,hdop,altitude,M,geoid,M,age,checksum
    
    // Input validation
    if (!line || strlen(line) == 0 || strlen(line) >= 256) {
        return false;
    }
    
    char* tokens[15];
    char line_copy[256];
    strncpy(line_copy, line, sizeof(line_copy) - 1);
    line_copy[sizeof(line_copy) - 1] = '\0';
    
    int token_count = 0;
    char* token = strtok(line_copy, ",");
    while (token && token_count < 15) {
        tokens[token_count++] = token;
        token = strtok(nullptr, ",");
    }
    
    if (token_count >= 6) {
        int quality = parse_int(tokens[6]);
        if (quality > 0) {
            _last_data.latitude = parse_coordinate(tokens[2]);
            _last_data.longitude = parse_coordinate(tokens[4]);
            // Bounds checking for array access
            if (token_count > 7) _last_data.satellites = parse_int(tokens[7]);
            if (token_count > 9) _last_data.altitude = parse_float(tokens[9]);
            _last_data.fix_valid = true;
            return true;
        }
    }
    
    _last_data.fix_valid = false;
    return false;
}

bool GPS::parse_gprmc(const char* line) {
    // Basic GPRMC parsing for speed and course
    // Format: $GPRMC,time,status,lat,N/S,lon,E/W,speed,course,date,magnetic,checksum
    
    // Input validation
    if (!line || strlen(line) == 0 || strlen(line) >= 256) {
        return false;
    }
    
    char* tokens[12];
    char line_copy[256];
    strncpy(line_copy, line, sizeof(line_copy) - 1);
    line_copy[sizeof(line_copy) - 1] = '\0';
    
    int token_count = 0;
    char* token = strtok(line_copy, ",");
    while (token && token_count < 12) {
        tokens[token_count++] = token;
        token = strtok(nullptr, ",");
    }
    
    if (token_count >= 8 && token_count > 2 && tokens[2] && tokens[2][0] == 'A') {  // 'A' = active/valid
        if (token_count > 7) _last_data.speed = parse_float(tokens[7]) * 1.852f;  // Convert knots to km/h
        if (token_count > 8) _last_data.course = parse_float(tokens[8]);
        return true;
    }
    
    return false;
}

double GPS::parse_coordinate(const char* str) {
    if (str == nullptr || strlen(str) == 0) return 0.0;
    
    double coord = atof(str);
    int degrees = (int)(coord / 100);
    double minutes = coord - (degrees * 100);
    
    return degrees + (minutes / 60.0);
}

float GPS::parse_float(const char* str) {
    if (str == nullptr || strlen(str) == 0) return 0.0f;
    return (float)atof(str);
}

int GPS::parse_int(const char* str) {
    if (str == nullptr || strlen(str) == 0) return 0;
    return atoi(str);
}