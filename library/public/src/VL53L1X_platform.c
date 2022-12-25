/*
* Platform-specific functions for VL53L1X driver library
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

/*
* This file is part of VL53L1X Platform
*
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "VL53L1X_platform.h"
#include "VL53L1X_api.h"
#include "VL53L1X_types.h"

/* ----- Global variables ----- */

static uint8_t buffer[VL53L1X_I2C_BUF_SIZE + 2];  // R/W buffer for all transactions
static i2c_inst_t* i2c_dev = NULL;  // i2c Pico device


/* ----- Static helper functions ----- */

// Read len bytes of data from i2c device with address dev into buff
// Expects register address to already be configured
// Returns 0 on success, -1 on failure
static VL53L1X_Status_t Pico_I2CRead(uint16_t addr, uint8_t *buff, uint8_t len) {
  return (i2c_read_blocking(i2c_dev, addr, buff, len, false) == len) - 1;
}

// Write len bytes of data to i2c device with address dev from buff
// Returns 0 on success, -1 on failure
static VL53L1X_Status_t Pico_I2CWrite(uint16_t addr, uint8_t *buff, uint8_t len) {
  return (i2c_write_blocking(i2c_dev, addr, buff, len, false) == len) - 1;
}



/* ----- Library functions ----- */

VL53L1X_Status_t VL53L1X_I2C_Init(uint16_t addr, i2c_inst_t* i2c_device) {
  VL53L1X_Status_t status;

  i2c_dev = i2c_device;

  // Setup i2c device
  i2c_init(i2c_dev, VL53L1X_I2C_BAUDRATE);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  // Validate sensor model ID and Type
  uint16_t sensorId;
  status = VL53L1X_GetSensorId(addr, &sensorId);
  if (sensorId != VL53L1X_SENSOR_ID) { // Bad connection, wrong chip, etc
    return -1;
  }

  return status;
}

VL53L1X_Status_t VL53L1X_WriteMulti(uint16_t addr, uint16_t reg, uint8_t *data, uint32_t count) {
  if ((count + 1) > VL53L1X_I2C_BUF_SIZE)
    return -1;

  buffer[0] = 0xFF & (reg >> 8);
  buffer[1] = 0xFF & (reg		);
  memcpy(&buffer[2], data, count);

  return Pico_I2CWrite(addr, buffer, (count + 2));
}

VL53L1X_Status_t VL53L1X_ReadMulti(uint16_t addr, uint16_t reg, uint8_t *data, uint32_t count) {
  VL53L1X_Status_t status;

  if ((count + 1) > VL53L1X_I2C_BUF_SIZE)
    return -1;

  buffer[0] = reg >> 8;
  buffer[1] = reg & 0xFF;

  status = Pico_I2CWrite(addr, buffer, 2);
  if (!status) {
    data[0] = reg;
    status = Pico_I2CRead(addr, data, count);
  }
  return status;
}

VL53L1X_Status_t VL53L1X_RdWord(uint16_t addr, uint16_t index, uint16_t* data) {
  VL53L1X_Status_t status = VL53L1X_ReadMulti(addr, index, (uint8_t*)data, 2);
  *data = ntohs(*data);
  return status;
}

VL53L1X_Status_t VL53L1X_RdDWord(uint16_t addr, uint16_t index, uint32_t* data) {
  VL53L1X_Status_t status = VL53L1X_ReadMulti(addr, index, (uint8_t*)data, 4);
  *data = ntohl(*data);
  return status;
}

VL53L1X_Status_t VL53L1X_RdByte(uint16_t addr, uint16_t index, uint8_t* data) {
  return VL53L1X_ReadMulti(addr, index, data, 1);
}

VL53L1X_Status_t VL53L1X_WrByte(uint16_t addr, uint16_t index, uint8_t data) {
  return VL53L1X_WriteMulti(addr, index, (uint8_t*)&data, 1);
}

VL53L1X_Status_t VL53L1X_WrWord(uint16_t addr, uint16_t index, uint16_t data) {
  data = htons(data);
  return VL53L1X_WriteMulti(addr, index, (uint8_t*)&data, 2);
}

VL53L1X_Status_t VL53L1X_WrDWord(uint16_t addr, uint16_t index, uint32_t data) {
  data = htonl(data);
  return VL53L1X_WriteMulti(addr, index, (uint8_t*)&data, 4);
}

VL53L1X_Status_t VL53L1X_WaitMs(uint16_t addr, int32_t wait_ms) {
  sleep_ms(wait_ms);
  return 0;
}
