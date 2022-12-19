# VL53L1X-C-API-Pico
VL53L1X Time-of-Flight distance sensor API for the Raspberry Pi Pico/Pico W

#### Notice: this library is in-progress
- Please feel free to contribute to this library.
- Major issues:
    - i2c communiucation with the VL53L1X hangs up in several steps (booting, starting ranging, etc.). This can be mitigated, albeit not solved, by adding delays.

## Usage
See the examples for usage. To build the project, run the following:
```
mkdir build
cd build
cmake .. -DPICO_BOARD=pico_w
make
```

The resulting uf2 files to upload to the Pico board reside in the build/examples directory.

## License
This API uses the `core` code from STMicroelectronics, and implements the platform for Raspberry Pi Pico/Pico W. The implemntation, under `platform`, is licensed under the MIT License. Code provided by STMicroelectronics (in `core`) is provided under the BSD 3-clause "New" or "Revised" License.