# Use this CMake file to compile the examples
# This file can also serve as a template for how
# to use the library in your own code. Note that
# more details on how to use the library are
# specified in library/import.cmake.

cmake_minimum_required(VERSION 3.25)

# Pull in SDK by downloading the pico_sdk_import
set(PICO_SDK_IMPORT ${BINARY_DIR}/external/pico_sdk_import.cmake)
if (NOT EXISTS ${PICO_SDK_IMPORT})
        file(DOWNLOAD
                https://raw.githubusercontent.com/raspberrypi/pico-sdk/2e6142b15b8a75c1227dd3edbe839193b2bf9041/external/pico_sdk_import.cmake
                ${PICO_SDK_IMPORT}
                )
endif()
include(${PICO_SDK_IMPORT})

# Create project and set basic info
project(VL53L1X_api
        DESCRIPTION "VL53L1X Raspberry Pi Pico/Pico W examples")
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall)
pico_sdk_init()

# Pull in the API
include(${PROJECT_SOURCE_DIR}/../library/import.cmake)

# Define flag for Pico W
if(${PICO_BOARD} STREQUAL pico_w)
    add_compile_definitions(PICO_W_BOARD)
    message("Compiling examples for Pico W")
else()
    message("Compiling examples for Pico")
endif()

add_subdirectory(continuous_measurement)
add_subdirectory(sampling)