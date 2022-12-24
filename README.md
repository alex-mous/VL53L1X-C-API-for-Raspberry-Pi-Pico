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
- Thread worker

# Examples
To build the examples for the project, first clone the code and then run the following:
```
cd examples
mkdir build
cd build
cmake ..
make
```
Note: if you have a Pico W, append `-DPICO_BOARD=pico_w` to the `cmake` command.

This will build the example code. Either of th resulting `uf2` files in `build/examples/continuous_measurement` and  `build/examples/sampling` can be uploaded to a Pico/Pico W. The former will continously print the sensor reading to stdout (over USB), whereas the second only prints samples when input is recieved from stdin (also over USB). To use these, leave your Pico connected to your computer after uploading the `uf2` file, and open a USB serial connection to it (for example, using Putty on Windows or native tools on UNIX-based systems).

For the examples to work correctly, the sensor must be attached to the following pins:

| VL53L1X |  Pico |
| ------ | ----- |
| VDD | Unconnected (Provides 2.8V)
| VIN | 3V3 |
| GND | GND |
| SDA | GPIO4 |
| SDA | GPIO5 |
| XSHUT | Unconnected (Used to reset device)|

# Installation
To use/install this library in your own project, acquire a copy of this repo in your project (such as using `git submodule`), and then make the following modifications to your project's `CMakeLists.txt`:

If your project structure is something like this:
``` root/ (for your project)
     yourapp.h
     yourapp.c
     CMakeLists.txt
     build/
         ...
     VL53L1X-C-API-Pico/
         library/
             import.cmake
             ...
         ...
```
Then, add this line to your `CMakeLists.txt` file before defining/linking your executables (e.g. building `yourapp`):
 ```
 include(${PROJECT_SOURCE_DIR}/VL53L1X-C-API-Pico/library/import.cmake)
 ```

Then, to link `<your executable>` with libraries you already have (`<your libraries>`) and this VL53L1X library, change the `target_link_libraries` line for `<your executable>` as follows:
```
target_link_libraries(<your executable> PUBLIC pico_stdlib hardware_i2c VL53L1X_pico_api <your libraries> )
```

Running `cmake` for your project will now include and compile this API.

## Using C++
If you are using C++, wrap the includes of this library in your project with an `extern` linkage specification as follows. Instead of `#include "VL53L1X_api.h"`, use:
```
extern "C" {
    #include "VL53L1X_api.h"
}
```

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
