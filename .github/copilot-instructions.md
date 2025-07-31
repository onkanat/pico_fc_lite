# Pico FC Lite - AI Development Guidelines

## Project Overview
This is a Raspberry Pi Pico-based flight controller implementation using the Pico SDK. The project follows a modular architecture with separate components for RC input, sensor processing, control algorithms, and telemetry.

## Architecture & Module Structure

### Core Components
- **SBUS Reader (`sbus_reader.c/.h/.pio`)**: RC input via PIO state machine with double-buffering
- **PID Controller (`pid_controller.c/.h`)**: Anti-windup PID implementation with configurable limits
- **MAVLink Handler (`mavlink_handler.c/.h`)**: UART-based telemetry communication
- **Sensors (`src/sensors/`)**: BMI270 IMU, BMP388 barometer, GPS modules
- **Control (`src/control/`)**: Motor mixing and advanced control algorithms
- **Filter (`src/filter/`)**: Mahony AHRS filter implementation
- **Comms (`src/comms/`)**: ESP-AT WiFi bridge and MAVLink routing

### Key Patterns

**Mixed C/C++ Codebase**: Legacy C modules (SBUS, PID, MAVLink) with modern C++ sensor drivers. Follow existing language choice per module.

**Hardware Abstraction**: All hardware initialization follows Pico SDK patterns:
```c
// GPIO setup example
gpio_set_function(pin, GPIO_FUNC_I2C);
gpio_pull_up(pin);
```

**PIO Usage**: SBUS reader uses custom PIO program for 100kbaud 8E2 inverted protocol. PIO programs in `.pio` files with C SDK integration.

**PIO Integration Pattern**: 
```c
// Load PIO program
pio_offset = pio_add_program(pio, &sbus_reader_program);
// Initialize with helper function from .pio file
sbus_reader_program_init(pio, sm, pio_offset, pin);
```

**Hardware GPIO Configuration**: SBUS requires inverted GPIO (`gpio_set_inover(pin, GPIO_OVERRIDE_INVERT)`) handled in PIO init.

**Double Buffering**: SBUS reader implements lock-free double buffering for real-time data access between PIO and main loop.

## Development Workflows

### Build System
- **Primary**: Use VS Code task "Compile Project" (`Ctrl+Shift+P` → "Tasks: Run Task")
- **Manual**: `ninja -C build` from workspace root
- **Flash**: VS Code task "Flash" or "Run Project" for picotool loading

### CMake Configuration
- **C++ Standard**: C++17 required for sensor drivers
- **Mixed Source**: Only `src/*.cpp` files included in CMake (C files excluded)
- **MAVLink Dependency**: Expects `c_library_v2/common` directory for MAVLink headers
- **Key Libraries**: `pico_stdlib`, `hardware_i2c`, `hardware_uart`, `hardware_spi`

### Project Structure
```
src/
├── main.c              # Original full FC implementation (legacy)
├── main.cpp            # Current BMI270 sensor test (active)
├── sbus_reader.*       # RC input via PIO
├── pid_controller.*    # Control algorithms
├── mavlink_handler.*   # Telemetry
├── sensors/            # Hardware drivers (C++)
├── control/            # Advanced control (C++)
├── filter/             # Signal processing (C++)
└── comms/              # Communication modules (C++)
```

**Important**: Only `main.cpp` is compiled due to CMake configuration - edit this file for current development.

### Critical Configuration
- **I2C**: Port i2c0, pins 4/5 (SDA/SCL), 400kHz
- **SBUS**: GPIO 13, PIO0 state machine, 100kbaud inverted
- **MAVLink**: UART0, pins 0/1, 57600 baud
- **USB**: Enabled for printf debugging, UART disabled

## Development Guidelines

### Adding New Sensors
1. Create C++ class in `src/sensors/` following BMI270 pattern
2. Use I2C port i2c0 with proper initialization in main
3. Implement `init()` and `read_raw()` methods for consistency

### PID Controller Usage
```c
PID_Controller pid;
pid_init(&pid, kp, ki, kd);
pid_set_limits(&pid, out_min, out_max, int_min, int_max);
float output = pid_update(&pid, measurement, dt);
```

### SBUS Integration
- Call `sbus_check_for_new_frame()` in main loop
- Use `sbus_get_latest_frame()` for thread-safe data access
- Channels are 11-bit values (0-2047)

### MAVLink Communication
- System ID: 1, Component ID: MAV_COMP_ID_AUTOPILOT1
- Heartbeat at 1Hz, attitude at 10Hz
- UART blocking writes for simplicity
- **Critical**: Requires MAVLink C library at `c_library_v2/common/` - clone from [mavlink/c_library_v2](https://github.com/mavlink/c_library_v2)
- Include path: `#include <mavlink/v2.0/common/mavlink.h>`

## Important Notes
- **Timing**: Main loop uses `to_ms_since_boot()` for consistent timing
- **Memory**: No dynamic allocation, all buffers statically allocated
- **Concurrency**: PIO runs independently, main loop polls for data
- **Debugging**: Use printf over USB, not UART (conflicts with MAVLink)

## Common Tasks
- **New PID loop**: Initialize controller, set limits, call update in main loop
- **Sensor integration**: Follow BMI270 pattern, add to main loop initialization
- **MAVLink messages**: Add new message functions to mavlink_handler.c
- **PIO protocols**: Create .pio file, integrate with C SDK helper functions
- **MAVLink Setup**: Clone `c_library_v2` to project root: `git clone https://github.com/mavlink/c_library_v2.git`
