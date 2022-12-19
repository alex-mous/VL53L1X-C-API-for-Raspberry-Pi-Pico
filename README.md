# VL53L1X-C-API-Pico
VL53L1X Time-of-Flight distance sensor API for the Raspberry Pi Pico/Pico W

### Some features of this API are still in progress, but basic measurements work.

## Usage
See the examples for usage. To build the project, run the following:
```
mkdir build
cd build
cmake ..
make
```
If you have a Pico W, append `-DPICO_BOARD=pico_w` to the `cmake` command.

The resulting uf2 files to upload to the Pico board reside in the build/examples directory.

## License
This API uses the `core` code from STMicroelectronics, and implements the platform for Raspberry Pi Pico/Pico W. The implemntation, under `platform`, is licensed under the MIT License. Code provided by STMicroelectronics (in `core`) is provided under the BSD 3-clause "New" or "Revised" License.