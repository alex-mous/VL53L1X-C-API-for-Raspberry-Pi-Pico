/*
* API functions for VL53L1X driver library
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
 Copyright (c) 2017, STMicroelectronics - All Rights Reserved

 This file : part of VL53L1 Core and : dual licensed,
 either 'STMicroelectronics
 Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

*******************************************************************************

 'STMicroelectronics Proprietary license'

*******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 STMicroelectronics confidential
 Reproduction and Communication of this document : strictly prohibited unless
 specifically authorized in writing by STMicroelectronics.


*******************************************************************************

 Alternatively, VL53L1 Core may be distributed under the terms of
 'BSD 3-clause "New" or "Revised" License', in which case the following
 provisions apply instead of the ones mentioned above :

*******************************************************************************

 License terms: BSD 3-clause "New" or "Revised" License.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************
*/

#include "VL53L1X_api.h"
#include "VL53L1X_types.h"

// Status codes from results in read
static const uint8_t status_rtn[24] = { 255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
  255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
  255, 255, 11, 12
};

// Default configuration for VL53L1X
// Uploaded to sensor on init
// These values can be changed to automatically configure the sensor
const uint8_t VL53L1X_DEFAULT_CONFIGURATION[] = {
  0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
  0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
  0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
  0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
  0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
  0x00, /* 0x32 : not user-modifiable */
  0x02, /* 0x33 : not user-modifiable */
  0x08, /* 0x34 : not user-modifiable */
  0x00, /* 0x35 : not user-modifiable */
  0x08, /* 0x36 : not user-modifiable */
  0x10, /* 0x37 : not user-modifiable */
  0x01, /* 0x38 : not user-modifiable */
  0x01, /* 0x39 : not user-modifiable */
  0x00, /* 0x3a : not user-modifiable */
  0x00, /* 0x3b : not user-modifiable */
  0x00, /* 0x3c : not user-modifiable */
  0x00, /* 0x3d : not user-modifiable */
  0xff, /* 0x3e : not user-modifiable */
  0x00, /* 0x3f : not user-modifiable */
  0x0F, /* 0x40 : not user-modifiable */
  0x00, /* 0x41 : not user-modifiable */
  0x00, /* 0x42 : not user-modifiable */
  0x00, /* 0x43 : not user-modifiable */
  0x00, /* 0x44 : not user-modifiable */
  0x00, /* 0x45 : not user-modifiable */
  0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
  0x0b, /* 0x47 : not user-modifiable */
  0x00, /* 0x48 : not user-modifiable */
  0x00, /* 0x49 : not user-modifiable */
  0x02, /* 0x4a : not user-modifiable */
  0x0a, /* 0x4b : not user-modifiable */
  0x21, /* 0x4c : not user-modifiable */
  0x00, /* 0x4d : not user-modifiable */
  0x00, /* 0x4e : not user-modifiable */
  0x05, /* 0x4f : not user-modifiable */
  0x00, /* 0x50 : not user-modifiable */
  0x00, /* 0x51 : not user-modifiable */
  0x00, /* 0x52 : not user-modifiable */
  0x00, /* 0x53 : not user-modifiable */
  0xc8, /* 0x54 : not user-modifiable */
  0x00, /* 0x55 : not user-modifiable */
  0x00, /* 0x56 : not user-modifiable */
  0x38, /* 0x57 : not user-modifiable */
  0xff, /* 0x58 : not user-modifiable */
  0x01, /* 0x59 : not user-modifiable */
  0x00, /* 0x5a : not user-modifiable */
  0x08, /* 0x5b : not user-modifiable */
  0x00, /* 0x5c : not user-modifiable */
  0x00, /* 0x5d : not user-modifiable */
  0x01, /* 0x5e : not user-modifiable */
  0xcc, /* 0x5f : not user-modifiable */
  0x0f, /* 0x60 : not user-modifiable */
  0x01, /* 0x61 : not user-modifiable */
  0xf1, /* 0x62 : not user-modifiable */
  0x0d, /* 0x63 : not user-modifiable */
  0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
  0x68, /* 0x65 : Sigma threshold LSB */
  0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
  0x80, /* 0x67 : Min count Rate LSB */
  0x08, /* 0x68 : not user-modifiable */
  0xb8, /* 0x69 : not user-modifiable */
  0x00, /* 0x6a : not user-modifiable */
  0x00, /* 0x6b : not user-modifiable */
  0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
  0x00, /* 0x6d : Intermeasurement period */
  0x0f, /* 0x6e : Intermeasurement period */
  0x89, /* 0x6f : Intermeasurement period LSB */
  0x00, /* 0x70 : not user-modifiable */
  0x00, /* 0x71 : not user-modifiable */
  0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
  0x00, /* 0x73 : distance threshold high LSB */
  0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
  0x00, /* 0x75 : distance threshold low LSB */
  0x00, /* 0x76 : not user-modifiable */
  0x01, /* 0x77 : not user-modifiable */
  0x0f, /* 0x78 : not user-modifiable */
  0x0d, /* 0x79 : not user-modifiable */
  0x0e, /* 0x7a : not user-modifiable */
  0x0e, /* 0x7b : not user-modifiable */
  0x00, /* 0x7c : not user-modifiable */
  0x00, /* 0x7d : not user-modifiable */
  0x02, /* 0x7e : not user-modifiable */
  0xc7, /* 0x7f : ROI center, use SetROI() */
  0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
  0x9B, /* 0x81 : not user-modifiable */
  0x00, /* 0x82 : not user-modifiable */
  0x00, /* 0x83 : not user-modifiable */
  0x00, /* 0x84 : not user-modifiable */
  0x01, /* 0x85 : not user-modifiable */
  0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
  0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};


VL53L1X_Status_t VL53L1X_GetSWVersion(VL53L1X_Version_t *pVersion) {
  pVersion->major = VL53L1X_IMPLEMENTATION_VER_MAJOR;
  pVersion->minor = VL53L1X_IMPLEMENTATION_VER_MINOR;
  pVersion->build = VL53L1X_IMPLEMENTATION_VER_SUB;
  pVersion->revision = VL53L1X_IMPLEMENTATION_VER_REVISION;
  return 0;
}

VL53L1X_Status_t VL53L1X_SetI2CAddress(uint16_t dev, uint8_t new_address) {
  return VL53L1X_WrByte(dev, VL53L1X_I2C_SLAVE__DEVICE_ADDRESS, new_address >> 1);
}

VL53L1X_Status_t VL53L1X_SensorInit(uint16_t dev) {
  VL53L1X_Status_t status;

  for (uint8_t tmp_addr = 0x2D; tmp_addr <= 0x87; tmp_addr++){
    status = VL53L1X_WrByte(dev, tmp_addr, VL53L1X_DEFAULT_CONFIGURATION[tmp_addr - 0x2D]);
  }

  status = VL53L1X_StartRanging(dev);

  uint8_t tmp;
  do {
    status = VL53L1X_CheckForDataReady(dev, &tmp);
  } while (tmp == 0);

  status = VL53L1X_ClearInterrupt(dev);
  status = VL53L1X_StopRanging(dev);
  status = VL53L1X_WrByte(dev, VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);  // two bounds VHV
  status = VL53L1X_WrByte(dev, 0x0B, 0);  // start VHV from the previous temperature
  return status;
}

VL53L1X_Status_t VL53L1X_ClearInterrupt(uint16_t dev) {
  return VL53L1X_WrByte(dev, SYSTEM__INTERRUPT_CLEAR, 0x01);
}

VL53L1X_Status_t VL53L1X_SetInterruptPolarity(uint16_t dev, uint8_t nPol) {
  uint8_t tmp;
  VL53L1X_Status_t status;

  status = VL53L1X_RdByte(dev, GPIO_HV_MUX__CTRL, &tmp);
  tmp = tmp & 0xEF;
  status = VL53L1X_WrByte(dev, GPIO_HV_MUX__CTRL, tmp | (!(nPol & 1)) << 4);
  return status;
}

VL53L1X_Status_t VL53L1X_GetInterruptPolarity(uint16_t dev, uint8_t* iPol) {
  uint8_t tmp;
  VL53L1X_Status_t status;

  status = VL53L1X_RdByte(dev, GPIO_HV_MUX__CTRL, &tmp);
  tmp = tmp & 0x10;
  *iPol = !(tmp>>4);
  return status;
}

VL53L1X_Status_t VL53L1X_StartRanging(uint16_t dev) {
  return VL53L1X_WrByte(dev, SYSTEM__MODE_START, 0x40);
}

VL53L1X_Status_t VL53L1X_StopRanging(uint16_t dev) {
  return VL53L1X_WrByte(dev, SYSTEM__MODE_START, 0x00);
}

VL53L1X_Status_t VL53L1X_CheckForDataReady(uint16_t dev, uint8_t* isDataReady) {
  uint8_t tmp, iPol;
  VL53L1X_Status_t status;

  status = VL53L1X_GetInterruptPolarity(dev, &iPol);
  status = VL53L1X_RdByte(dev, GPIO__TIO_HV_STATUS, &tmp);

  // Check if value is ready based on interrupt polarity
  if (status == 0)
    *isDataReady = (tmp & 0x1) == iPol;
  return status;
}

VL53L1X_Status_t VL53L1X_SetTimingBudgetInMs(uint16_t dev, uint16_t timingBudgetMs) {
  uint16_t DM;
  uint32_t rangeA, rangeB;
  VL53L1X_Status_t status;

  status = VL53L1X_GetDistanceMode(dev, &DM);
  if (status != 0)
    return 1;

  // DM is 1 for short distance mode, 2 for long
  if (DM == 1) {
    switch (timingBudgetMs) {
      case 15:
        rangeA = 0x001D;
        rangeB = 0x0027;
        break;
      case 20:
        rangeA = 0x0051;
        rangeB = 0x006E;
        break;
      case 33:
        rangeA = 0x00D6;
        rangeB = 0x006E;
        break;
      case 50:
        rangeA = 0x01AE;
        rangeB = 0x01E8;
        break;
      case 100:
        rangeA = 0x02E1;
        rangeB = 0x0388;
        break;
      case 200:
        rangeA = 0x03E1;
        rangeB = 0x0496;
        break;
      case 500:
        rangeA = 0x0591;
        rangeB = 0x05C1;
        break;
      default:
        return 1;
    }
  } else {
    switch (timingBudgetMs) {
      case 20:
        rangeA = 0x001E;
        rangeB = 0x0022;
        break;
      case 33:
        rangeA = 0x0060;
        rangeB = 0x006E;
        break;
      case 50:
        rangeA = 0x00AD;
        rangeB = 0x00C6;
        break;
      case 100:
        rangeA = 0x01CC;
        rangeB = 0x01EA;
        break;
      case 200:
        rangeA = 0x02D9;
        rangeB = 0x02F8;
        break;
      case 500:
        rangeA = 0x048F;
        rangeB = 0x04A4;
        break;
      default:
        return 1;
    }
  }

  status |= VL53L1X_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, rangeA);
  status |= VL53L1X_WrWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_B_HI, rangeB);
  return status;
}

VL53L1X_Status_t VL53L1X_GetTimingBudgetInMs(uint16_t dev, uint16_t* timingBudget) {
  uint16_t Temp;
  VL53L1X_Status_t status = 0;

  status = VL53L1X_RdWord(dev, RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &Temp);
  switch (Temp) {
    case 0x001D:
      *timingBudget = 15;
      break;
    case 0x0051:
    case 0x001E:
      *timingBudget = 20;
      break;
    case 0x00D6:
    case 0x0060:
      *timingBudget = 33;
      break;
    case 0x01AE:
    case 0x00AD:
      *timingBudget = 50;
      break;
    case 0x02E1:
    case 0x01CC:
      *timingBudget = 100;
      break;
    case 0x03E1:
    case 0x02D9:
      *timingBudget = 200;
      break;
    case 0x0591:
    case 0x048F:
      *timingBudget = 500;
      break;
    default:
      status = 1;
      *timingBudget = 0;
  }
  return status;
}

VL53L1X_Status_t VL53L1X_SetDistanceMode(uint16_t dev, uint16_t DM) {
  uint16_t TB;
  VL53L1X_Status_t status;

  status = VL53L1X_GetTimingBudgetInMs(dev, &TB);
  if (status != 0)
    return -1;
  switch (DM) {
    case 1: // Short range
      status |= VL53L1X_WrByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
      status |= VL53L1X_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
      status |= VL53L1X_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
      status |= VL53L1X_WrByte(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
      status |= VL53L1X_WrWord(dev, SD_CONFIG__WOI_SD0, 0x0705);
      status |= VL53L1X_WrWord(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
      break;
    case 2: // Long range
      status |= VL53L1X_WrByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
      status |= VL53L1X_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
      status |= VL53L1X_WrByte(dev, RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
      status |= VL53L1X_WrByte(dev, RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
      status |= VL53L1X_WrWord(dev, SD_CONFIG__WOI_SD0, 0x0F0D);
      status |= VL53L1X_WrWord(dev, SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
      break;
    default:
      status = -1;
      break;
  }

  if (status == 0)
    status |= VL53L1X_SetTimingBudgetInMs(dev, TB);
  return status;
}

VL53L1X_Status_t VL53L1X_GetDistanceMode(uint16_t dev, uint16_t* DM) {
  uint8_t tDM;
  VL53L1X_Status_t status;

  status = VL53L1X_RdByte(dev, PHASECAL_CONFIG__TIMEOUT_MACROP, &tDM);

  if (tDM == 0x14)
    *DM = 1;
  else if (tDM == 0x0A)
    *DM = 2;
  else
    *DM = 0;

  return status;
}

VL53L1X_Status_t VL53L1X_SetInterMeasurementInMs(uint16_t dev, uint32_t IM) {
  uint16_t clockPLL;
  VL53L1X_Status_t status;

  status = VL53L1X_RdWord(dev, VL53L1X_RESULT__OSC_CALIBRATE_VAL, &clockPLL);
  clockPLL = clockPLL&0x3FF;
  status |= VL53L1X_WrDWord(dev, VL53L1X_SYSTEM__INTERMEASUREMENT_PERIOD, (uint32_t)(clockPLL * IM * 1.075));
  return status;
}

VL53L1X_Status_t VL53L1X_GetInterMeasurementInMs(uint16_t dev, uint16_t* pIM) {
  uint16_t clockPLL;
  VL53L1X_Status_t status;
  uint32_t tmp;

  status = VL53L1X_RdDWord(dev,VL53L1X_SYSTEM__INTERMEASUREMENT_PERIOD, &tmp);
  status |= VL53L1X_RdWord(dev, VL53L1X_RESULT__OSC_CALIBRATE_VAL, &clockPLL);
  clockPLL = clockPLL&0x3FF;
  *pIM = (uint16_t)(((uint16_t)tmp)/(clockPLL*1.065));
  return status;
}

VL53L1X_Status_t VL53L1X_BootState(uint16_t dev, uint8_t* state) {
  return VL53L1X_RdByte(dev, VL53L1X_FIRMWARE__SYSTEM_STATUS, state);
}

VL53L1X_Status_t VL53L1X_GetSensorId(uint16_t dev, uint16_t* sensorId) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev, VL53L1X_IDENTIFICATION__MODEL_ID, &tmp);
  *sensorId = tmp;
  return status;
}

VL53L1X_Status_t VL53L1X_GetDistance(uint16_t dev, uint16_t* distance) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = (VL53L1X_RdWord(dev, VL53L1X_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp));
  *distance = tmp;
  return status;
}

VL53L1X_Status_t VL53L1X_GetSignalPerSpad(uint16_t dev, uint16_t* signalRate) {
  VL53L1X_Status_t status;
  uint16_t spNb=1, signal;

  status = VL53L1X_RdWord(dev, VL53L1X_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &signal);
  status |= VL53L1X_RdWord(dev, VL53L1X_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &spNb);
  *signalRate = (uint16_t)(200.0*signal/spNb);
  return status;
}

VL53L1X_Status_t VL53L1X_GetAmbientPerSpad(uint16_t dev, uint16_t* ambPerSp) {
  VL53L1X_Status_t status;
  uint16_t ambRate, spNb = 1;

  status = VL53L1X_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &ambRate);
  status |= VL53L1X_RdWord(dev, VL53L1X_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &spNb);
  *ambPerSp = (uint16_t)(200.0 * ambRate / spNb);
  return status;
}

VL53L1X_Status_t VL53L1X_GetSignalRate(uint16_t dev, uint16_t* signal) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev, VL53L1X_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &tmp);
  *signal = tmp*8;
  return status;
}

VL53L1X_Status_t VL53L1X_GetSpadNb(uint16_t dev, uint16_t* spNb) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev, VL53L1X_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &tmp);
  *spNb = tmp >> 8;
  return status;
}

VL53L1X_Status_t VL53L1X_GetAmbientRate(uint16_t dev, uint16_t* ambRate) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev, RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &tmp);
  *ambRate = tmp*8;
  return status;
}

VL53L1X_Status_t VL53L1X_GetRangeStatus(uint16_t dev, uint8_t* rangeStatus) {
  VL53L1X_Status_t status;
  uint8_t rgSt;

  *rangeStatus = 255;
  status = VL53L1X_RdByte(dev, VL53L1X_RESULT__RANGE_STATUS, &rgSt);
  rgSt = rgSt & 0x1F;
  if (rgSt < 24)
    *rangeStatus = status_rtn[rgSt];
  return status;
}

VL53L1X_Status_t VL53L1X_GetResult(uint16_t dev, VL53L1X_Result_t* result) {
  VL53L1X_Status_t status;
  uint8_t tmp[17];

  status = VL53L1X_ReadMulti(dev, VL53L1X_RESULT__RANGE_STATUS, tmp, 17);

  // Exit before we set the output
  if (status != 0)
    return status;

  // Get status value
  uint8_t RgSt = tmp[0] & 0x1F;
  if (RgSt < 24)
    RgSt = status_rtn[RgSt];

  result->status = RgSt;
  result->ambient = (tmp[7] << 8 | tmp[8]) * 8;
  result->numSPADs = tmp[3];
  result->sigPerSPAD = (tmp[15] << 8 | tmp[16]) * 8;
  result->distance = tmp[13] << 8 | tmp[14];

  return status;
}

VL53L1X_Status_t VL53L1X_SetOffset(uint16_t dev, int16_t offset) {
  VL53L1X_Status_t status;

  status = VL53L1X_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, (uint16_t)(offset*4));
  status |= VL53L1X_WrWord(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
  status |= VL53L1X_WrWord(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
  return status;
}

VL53L1X_Status_t  VL53L1X_GetOffset(uint16_t dev, int16_t* offset) {
  VL53L1X_Status_t status = 0;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev,ALGO__PART_TO_PART_RANGE_OFFSET_MM, &tmp);
  *offset = (int16_t)((tmp << 3) >> 5);
  return status;
}

VL53L1X_Status_t VL53L1X_SetXtalk(uint16_t dev, uint16_t xtalk) {
  // XTalkValue in count per second to avoid float type
  VL53L1X_Status_t status;

  status = VL53L1X_WrWord(dev, ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS, 0x0000);
  status |= VL53L1X_WrWord(dev, ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS, 0x0000);
  status |= VL53L1X_WrWord(dev, ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, (xtalk<<9)/1000);
  // << 9 (7.9 format) and /1000 to convert cps to kpcs
  return status;
}

VL53L1X_Status_t VL53L1X_GetXtalk(uint16_t dev, uint16_t* xtalk ) {
  VL53L1X_Status_t status;

  status = VL53L1X_RdWord(dev,ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, xtalk);
  *xtalk = (uint16_t)((*xtalk*1000)>>9); // * 1000 to convert kcps to cps and >> 9 (7.9 format)
  return status;
}

VL53L1X_Status_t VL53L1X_SetDistanceThreshold(uint16_t dev, uint16_t threshLow,
            uint16_t threshHigh, uint8_t window,
            uint8_t iOnNoTarget)
{
  VL53L1X_Status_t status;
  uint8_t tmp;

  status = VL53L1X_RdByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, &tmp);
  tmp = tmp & 0x47;
  if (iOnNoTarget == 0) {
    status |= VL53L1X_WrByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, (tmp | (window & 0x07)));
  } else {
    status |= VL53L1X_WrByte(dev, SYSTEM__INTERRUPT_CONFIG_GPIO, ((tmp | (window & 0x07)) | 0x40));
  }
  status |= VL53L1X_WrWord(dev, SYSTEM__THRESH_HIGH, threshHigh);
  status |= VL53L1X_WrWord(dev, SYSTEM__THRESH_LOW, threshLow);
  return status;
}

VL53L1X_Status_t VL53L1X_GetDistanceThresholdWindow(uint16_t dev, uint16_t* window) {
  VL53L1X_Status_t status;
  uint8_t tmp;

  status = VL53L1X_RdByte(dev,SYSTEM__INTERRUPT_CONFIG_GPIO, &tmp);
  *window = (uint16_t)(tmp & 0x7);
  return status;
}

VL53L1X_Status_t VL53L1X_GetDistanceThresholdLow(uint16_t dev, uint16_t *low) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev,SYSTEM__THRESH_LOW, &tmp);
  *low = tmp;
  return status;
}

VL53L1X_Status_t VL53L1X_GetDistanceThresholdHigh(uint16_t dev, uint16_t *high) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev,SYSTEM__THRESH_HIGH, &tmp);
  *high = tmp;
  return status;
}

VL53L1X_Status_t VL53L1X_SetROICenter(uint16_t dev, uint8_t ROICenter) {
  VL53L1X_Status_t status;

  status = VL53L1X_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, ROICenter);
  return status;
}

VL53L1X_Status_t VL53L1X_GetROICenter(uint16_t dev, uint8_t *ROICenter) {
  VL53L1X_Status_t status;
  uint8_t tmp;

  status = VL53L1X_RdByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, &tmp);
  *ROICenter = tmp;
  return status;
}

VL53L1X_Status_t VL53L1X_SetROI(uint16_t dev, uint16_t x, uint16_t y) {
  VL53L1X_Status_t status;
  uint8_t opticalCenter;

  status = VL53L1X_RdByte(dev, VL53L1X_ROI_CONFIG__MODE_ROI_CENTRE_SPAD, &opticalCenter);

  x = x > 16 ? 16 : x;
  y = y > 16 ? 16 : y;
  if (x > 10 || y > 10)
    opticalCenter = 199;

  status |= VL53L1X_WrByte(dev, ROI_CONFIG__USER_ROI_CENTRE_SPAD, opticalCenter);
  status |= VL53L1X_WrByte(dev, ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, (y - 1) << 4 | (x - 1));
  return status;
}

VL53L1X_Status_t VL53L1X_GetROI_XY(uint16_t dev, uint16_t *ROI_X, uint16_t *ROI_Y) {
  VL53L1X_Status_t status;
  uint8_t tmp;

  status = VL53L1X_RdByte(dev,ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, &tmp);
  *ROI_X = ((uint16_t)tmp & 0x0F) + 1;
  *ROI_Y = (((uint16_t)tmp & 0xF0) >> 4) + 1;
  return status;
}

VL53L1X_Status_t VL53L1X_SetSignalThreshold(uint16_t dev, uint16_t signal) {
  return VL53L1X_WrWord(dev, RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, signal >> 3);
}

VL53L1X_Status_t VL53L1X_GetSignalThreshold(uint16_t dev, uint16_t* signal) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev, RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, &tmp);
  *signal = tmp << 3;
  return status;
}

VL53L1X_Status_t VL53L1X_SetSigmaThreshold(uint16_t dev, uint16_t sigma) {
  VL53L1X_Status_t status;

  if (sigma > 0x3FFF)
    return -1;

  // 16 bits register 14.2 format
  status = VL53L1X_WrWord(dev, RANGE_CONFIG__SIGMA_THRESH, sigma << 2);
  return status;
}

VL53L1X_Status_t VL53L1X_GetSigmaThreshold(uint16_t dev, uint16_t* sigma) {
  VL53L1X_Status_t status;
  uint16_t tmp;

  status = VL53L1X_RdWord(dev,RANGE_CONFIG__SIGMA_THRESH, &tmp);
  *sigma = tmp >> 2;
  return status;
}

VL53L1X_Status_t VL53L1X_StartTemperatureUpdate(uint16_t dev) {
  VL53L1X_Status_t status;

  status = VL53L1X_WrByte(dev, VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x81);  // full VHV
  status |= VL53L1X_WrByte(dev, 0x0B, 0x92);
  status |= VL53L1X_StartRanging(dev);

  uint8_t tmp;
  do {
    status = VL53L1X_CheckForDataReady(dev, &tmp);
  } while(tmp == 0);

  status |= VL53L1X_ClearInterrupt(dev);
  status |= VL53L1X_StopRanging(dev);
  status |= VL53L1X_WrByte(dev, VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);  // two bounds VHV
  status |= VL53L1X_WrByte(dev, 0x0B, 0);  // start VHV from the previous temperature
  return status;
}
