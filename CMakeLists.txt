# Compile this library
cmake_minimum_required(VERSION 3.25)

# Pull in SDK by downloading the pico_sdk_import
set(PICO_SDK_IMPORT ${CMAKE_CURRENT_LIST_DIR}/external/pico_sdk_import.cmake)
if (NOT EXISTS ${PICO_SDK_IMPORT})
        file(DOWNLOAD
                https://raw.githubusercontent.com/raspberrypi/pico-sdk/2e6142b15b8a75c1227dd3edbe839193b2bf9041/external/pico_sdk_import.cmake
                ${PICO_SDK_IMPORT}
                )
endif()

include(${PICO_SDK_IMPORT})

project(VL53L1X_api
        VERSION 1.0.1
        DESCRIPTION "VL53L1X Raspberry Pi Pico/Pico W API")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

add_compile_options(-Wall
        -Wno-format     
        -Wno-unused-function
        -Wno-maybe-uninitialized
        )

add_subdirectory(VL53L1X_pico_api)

add_subdirectory(examples)