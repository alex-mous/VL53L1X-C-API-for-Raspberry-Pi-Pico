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

#ifndef _VL53L1X_PLATFORM_H_
#define _VL53L1X_PLATFORM_H_

#include <stdint.h>
#include "hardware/i2c.h"
#include "VL53L1X_types.h"

#ifdef __cplusplus
extern "C"
{
#endif


/* ----- Endianness Conversions  -----*/

#ifndef htons
#define htons(x)	((uint16_t)(((uint16_t)(x) & 0x00ff) << 8 |		\
            ((uint16_t)(x) & 0xff00) >> 8))
#define ntohs(x)	htons(x)
#endif

#ifndef htonl
#define htonl(x)	((uint32_t)(((uint32_t)(x) & 0x000000ff) << 24 |	\
            ((uint32_t)(x) & 0x0000ff00) << 8  | 			\
            ((uint32_t)(x) & 0x00ff0000) >> 8  |			\
            ((uint32_t)(x) & 0xff000000) >> 24))
#define ntohl(x)	htonl(x)
#endif

/* ----- I2C Communication Parameters ----- */

#define VL53L1X_I2C_BUF_SIZE 512
#define VL53L1X_I2C_BAUDRATE 100000


/* ----- Other Parameters ----- */

// Sensor ID on successful connection
#define VL53L1X_SENSOR_ID 0xEACC


/* ----- I2C Platform-Specific Implementation ----- */

// Initialize the Pico I2C interface with needed parameters for
// I2C controller <i2c_device> [i2c0 or i2c1, from hardware/i2c.h].
// Address of VL53L1X sensor should be in <dev> (default value is 0x29)
// Validates VL53L1X sensor is connected, and returns 0 on successful
// initialization and validating the sensor. Returns -1 on failure.
VL53L1X_Status_t VL53L1X_I2C_Init(uint16_t dev, i2c_inst_t* i2c_device);

// Delays for <wait_ms> milliseconds. Returns 0.
VL53L1X_Status_t VL53L1X_WaitMs(uint16_t dev, int32_t wait_ms);

// Write a single byte to register <index> in device with address <dev>
// Returns 0 on success, -1 on failure
VL53L1X_Status_t VL53L1X_WrByte(uint16_t dev, uint16_t index, uint8_t data);

// Write a word (2 bytes) to register <index> in device with address <dev>
// Returns 0 on success, -1 on failure
VL53L1X_Status_t VL53L1X_WrWord(uint16_t dev, uint16_t index, uint16_t data);

// Write a double word (4 bytes) to register <index> in device with address <dev>
// Returns 0 on success, -1 on failure
VL53L1X_Status_t VL53L1X_WrDWord(uint16_t dev, uint16_t index, uint32_t data);

// Write <count> bytes starting at register <index> in device with address <dev>
// Returns 0 on success, -1 on failure. Will not write more than VL53L1X_I2C_BUF_SIZE bytes.
VL53L1X_Status_t VL53L1X_WriteMulti(uint16_t dev, uint16_t index, uint8_t* data, uint32_t count);

// Read a single byte from register <index> in device with address <dev>
// Stores result in <data>. Returns 0 on success, -1 on failure
VL53L1X_Status_t VL53L1X_RdByte(uint16_t dev, uint16_t index, uint8_t* data);

// Read a word (2 bytes) from register <index> in device with address <dev>
// Stores result in <data>. Returns 0 on success, -1 on failure
VL53L1X_Status_t VL53L1X_RdWord(uint16_t dev, uint16_t index, uint16_t* data);

// Read a double word (4 bytes) from register <index> in device with address <dev>
// Stores result in <data>. Returns 0 on success, -1 on failure
VL53L1X_Status_t VL53L1X_RdDWord(uint16_t dev, uint16_t index, uint32_t* data);

// Read <count> bytes starting at register <index> in device with address <dev>
// Stores result in <data> (must be adequate size). Returns 0 on success, -1 on failure.
// Will not read more than VL53L1X_I2C_BUF_SIZE bytes.
VL53L1X_Status_t VL53L1X_ReadMulti(uint16_t dev, uint16_t index, uint8_t* data, uint32_t count);


#ifdef __cplusplus
}
#endif

#endif  // _VL53L1X_PLATFORM_H_
