# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.16)

# Disable Wi-Fi and Bluetooth features before SDK initialization
set(PICO_WIFI_SUPPORT 0)
set(PICO_BLUETOOTH_SUPPORT 0)

# Initialize pico-sdk from environment variable PICO_SDK_PATH
include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(pico_fc_lite C CXX ASM)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Initialize the SDK
pico_sdk_init()

# Recursively find all source files
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Add executable and specify source files
add_executable(${PROJECT_NAME} ${SOURCES})

# Add MAVLink library to include path
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/c_library_v2/common)

# Link to required hardware libraries
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_i2c
    hardware_uart
    hardware_spi
)

# Enable usb output for debugging, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(${PROJECT_NAME})

