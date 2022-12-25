/*
* Example usage of the VL53L1X Pico Library for Pico/Pico W
*
* This program will continously print out packets read from the VL53L1X sensor
* to USB (to read them, leave the Pico/Pico W connected to your computer via USB
* and then open a serial connection to the corresponding port with baudrate 115200).
*/

/*
* Copyright 2022, Alex Mous
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
* DEALINGS IN THE SOFTWARE.*/

#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"

#ifdef PICO_W_BOARD
#include "pico/cyw43_arch.h"
#endif

#include "VL53L1X_api.h"
#include "VL53L1X_types.h"


#define I2C_DEV_ADDR 0x29


int main() {
  VL53L1X_Status_t status;
  VL53L1X_Result_t results;

  stdio_init_all();

  // Blink the onboard LED to show booting
#if PICO_W_BOARD
  if (cyw43_arch_init()) {
    printf("Failed to initialize Pico W.\n");
    return 1;
  }
#else
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif

  for (int i=0; i<10; i++) {
#ifdef PICO_W_BOARD
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
#else
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
#endif

    sleep_ms(250);

#ifdef PICO_W_BOARD
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
#else
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
#endif

    sleep_ms(250);
  }

  // Initialize Pico's I2C using PICO_DEFAULT_I2C_SDA_PIN 
  // and PICO_DEFAULT_I2C_SCL_PIN (GPIO 4 and GPIO 5, respectively)
  if (VL53L1X_I2C_Init(I2C_DEV_ADDR, i2c0) < 0) {
      printf("Error initializing sensor.\n");
      return 0;
  }

  // Ensure the sensor has booted
  uint8_t sensorState;
  do {
    status += VL53L1X_BootState(I2C_DEV_ADDR, &sensorState);
    VL53L1X_WaitMs(I2C_DEV_ADDR, 2);
  } while (sensorState == 0);
  printf("Sensor booted.\n");

  // Initialize and configure sensor
  status = VL53L1X_SensorInit(I2C_DEV_ADDR);
  status += VL53L1X_SetDistanceMode(I2C_DEV_ADDR, 1);
  status += VL53L1X_SetTimingBudgetInMs(I2C_DEV_ADDR, 100);
  status += VL53L1X_SetInterMeasurementInMs(I2C_DEV_ADDR, 100);
  status += VL53L1X_StartRanging(I2C_DEV_ADDR);

  // Measure and print continuously
  bool first_range = true;
  while (1) {
    // Wait until we have new data
    uint8_t dataReady;
    do {
      status = VL53L1X_CheckForDataReady(I2C_DEV_ADDR, &dataReady);
      sleep_us(1);
    } while (dataReady == 0);

    // Read and display result
    status += VL53L1X_GetResult(I2C_DEV_ADDR, &results);
    printf("Status = %2d, dist = %5d, Ambient = %2d, Signal = %5d, #ofSpads = %5d\n",
      results.status, results.distance, results.ambient, results.sigPerSPAD, results.numSPADs);

    // Clear the sensor for a new measurement
    status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
    if (first_range) {  // Clear twice on first measurement
      status += VL53L1X_ClearInterrupt(I2C_DEV_ADDR);
      first_range = false;
    }
  }
}