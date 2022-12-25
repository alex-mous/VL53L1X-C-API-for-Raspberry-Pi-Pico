/*
* Calibration functions for VL53L1X driver library
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
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file : part of VL53L1 Core and : dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document : strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
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
********************************************************************************/

#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#include "VL53L1X_types.h"


VL53L1X_Status_t VL53L1X_CalibrateOffset(uint16_t dev, uint16_t targetDistInMm, int16_t *offset) {
  int16_t avgDist = 0;
  uint16_t distance;
  VL53L1X_Status_t status;

  status = VL53L1X_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0);
  status |= VL53L1X_WrWord(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
  status |= VL53L1X_WrWord(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
  status |= VL53L1X_StartRanging(dev);  // enable the sensor
  for (uint8_t i=0; i<NUM_CALIBRATION_SAMPLES; i++) {
    uint8_t tmp;
    do {
      status += VL53L1X_CheckForDataReady(dev, &tmp);
    } while (tmp == 0);
    status |= VL53L1X_GetDistance(dev, &distance);
    status |= VL53L1X_ClearInterrupt(dev);
    avgDist = avgDist + distance;
  }
  status |= VL53L1X_StopRanging(dev);
  avgDist = avgDist / NUM_CALIBRATION_SAMPLES;
  *offset = targetDistInMm - avgDist;
  status |= VL53L1X_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, *offset*4);
  return status;
}

VL53L1X_Status_t VL53L1X_CalibrateXtalk(uint16_t dev, uint16_t targetDistInMm, uint16_t *xtalk) {
  float avgSigRate = 0;
  float avgDist = 0;
  float avgSpadNb = 0;
  uint16_t distance = 0, sr, spadNum;
  uint32_t calXtalk;
  VL53L1X_Status_t status = 0;

  status |= VL53L1X_WrWord(dev, 0x0016,0);
  status |= VL53L1X_StartRanging(dev);
  for (uint8_t i = 0; i < 50; i++) {
    uint8_t tmp;
    do {
      status |= VL53L1X_CheckForDataReady(dev, &tmp);
    } while (tmp == 0);
    status |= VL53L1X_GetSignalRate(dev, &sr);
    status |= VL53L1X_GetDistance(dev, &distance);
    status |= VL53L1X_ClearInterrupt(dev);
    status |= VL53L1X_GetSpadNb(dev, &spadNum);
    avgDist = avgDist + distance;
    avgSpadNb = avgSpadNb + spadNum;
    avgSigRate = avgSigRate + sr;
  }
  status |= VL53L1X_StopRanging(dev);
  avgDist = avgDist / NUM_CALIBRATION_SAMPLES;
  avgSpadNb = avgSpadNb / NUM_CALIBRATION_SAMPLES;
  avgSigRate = avgSigRate / NUM_CALIBRATION_SAMPLES;
  // calculate Xtalk value
  calXtalk = (uint16_t)(512*(avgSigRate*(1-(avgDist/targetDistInMm)))/avgSpadNb);
  if (calXtalk > 0xffff)
    calXtalk = 0xffff;
  *xtalk = (uint16_t)((calXtalk*1000)>>9);
  status |= VL53L1X_WrWord(dev, 0x0016, (uint16_t)calXtalk);
  return status;
}
