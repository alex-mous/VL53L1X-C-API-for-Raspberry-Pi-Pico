# VL53L1X Distance Sensor C/C++ API for Raspberry Pi Pico/Pico W
![GitHub release (latest SemVer)](https://img.shields.io/github/v/release/alex-mous/VL53L1X-C-API-Pico)
![GitHub](https://img.shields.io/github/license/alex-mous/VL53L1X-C-API-Pico)
![GitHub issues](https://img.shields.io/github/issues/alex-mous/VL53L1X-C-API-Pico)
![GitHub contributors](https://img.shields.io/github/contributors/alex-mous/VL53L1X-C-API-Pico)
## Description

A light, easy-to-use VL53L1X Time-of-Flight distance sensor driver/API in C/C++ for the Raspberry Pi Pico/Pico W. This project implements and extends the STMicroelectronics template driver for this sensor, specifically for Rasbperry Pi Pico/Pico W microcontrollers.

## Contributing
Please see [CONTRIBUTING.md](CONTRIBUTING.md)

## Future Features
- C++ object-oriented library
- Easy way to configure defaults passed into initialization functions


# Usage
To use this library in your own project, include the `VL53L1X_pico_api` directory in your CMake script (e.g. `add_subdirectory(VL53L1X_pico_api)`). Then, include the needed headers as specified below, and add `VL53L1X_pico_api` to the `target_link_libraries` for your project. Running `cmake` for your project should then include and compile this API as needed.

# Examples
To build the project, first clone the code and then run the following:
```
mkdir build
cd build
cmake ..
make
```
Note: if you have a Pico W, append `-DPICO_BOARD=pico_w` to the `cmake` command.

This will build the example code. The resulting uf2 file in the `build/examples/continuous_measurement` can be uploaded to a Pico/Pico W. This example shows how to initialize and use the VL53L1X sensor to continously measure. 

# Documentation
The header files all have extensive documentation on the purpose of each function, which can be supplemented by the *STMicroelectronics VL53L1X Ultra Lite Driver User Manual (UM2510)*.

The three header files that can be included to use API functions are:
    - `VL53L1X_types.h`: This header file store types used in the API (status and results definitions, etc.).
    - `VL53L1X_api.h`: This header file contains all of the main API functions (initialization functions, getters and setters for registers, and functions to start/stop/read ranging).
    - `VL53L1X_calibration.h`: This header file contains functions to calibrate the VL53L1X sensor.
In addition, the final header file, `VL53L1X_platform.h`, contains platform-specific functions and definitions (e.g. I2C setup and read/write for the Pico and Pico W). 

For most use cases, just including the first two header files should be sufficient.

# License
This API extends, rewrites, and implements the VL53L1X driver code from STMicroelectronics. Code provided by STMicroelectronics (in `core`) is provided under the BSD 3-clause "New" or "Revised" License. All changes are licensed under the MIT License.
