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
*
*
********************************************************************************
*
*/

#ifndef _VL53L1X_API_H_
#define _VL53L1X_API_H_

#include "VL53L1X_platform.h"


/* ----- STMicroelectronics Version ----- */
#define VL53L1X_IMPLEMENTATION_VER_MAJOR       3
#define VL53L1X_IMPLEMENTATION_VER_MINOR       5
#define VL53L1X_IMPLEMENTATION_VER_SUB         0
#define VL53L1X_IMPLEMENTATION_VER_REVISION    0000

/* ----- VL53L1X Register Addresses ----- */
#define SOFT_RESET											0x0000
#define VL53L1X_I2C_SLAVE__DEVICE_ADDRESS					0x0001
#define VL53L1X_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND       0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 		0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 	0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 	0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM					0x001E
#define MM_CONFIG__INNER_OFFSET_MM							0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 							0x0022
#define GPIO_HV_MUX__CTRL									0x0030
#define GPIO__TIO_HV_STATUS       							0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO 						0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP     				0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI   				0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A        				0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B						0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI  					0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO  					0x0062
#define RANGE_CONFIG__SIGMA_THRESH 							0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS			0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH      				0x0069
#define VL53L1X_SYSTEM__INTERMEASUREMENT_PERIOD				0x006C
#define SYSTEM__THRESH_HIGH 								0x0072
#define SYSTEM__THRESH_LOW 									0x0074
#define SD_CONFIG__WOI_SD0                  				0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0        				0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD					0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE		0x0080
#define SYSTEM__SEQUENCE_CONFIG								0x0081
#define VL53L1X_SYSTEM__GROUPED_PARAMETER_HOLD 				0x0082
#define SYSTEM__INTERRUPT_CLEAR       						0x0086
#define SYSTEM__MODE_START                 					0x0087
#define VL53L1X_RESULT__RANGE_STATUS						0x0089
#define VL53L1X_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0		0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD					0x0090
#define VL53L1X_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0				0x0096
#define VL53L1X_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 	0x0098
#define VL53L1X_RESULT__OSC_CALIBRATE_VAL					0x00DE
#define VL53L1X_FIRMWARE__SYSTEM_STATUS                     0x00E5
#define VL53L1X_IDENTIFICATION__MODEL_ID                    0x010F
#define VL53L1X_ROI_CONFIG__MODE_ROI_CENTRE_SPAD			0x013E

// Get the ST version
VL53L1X_Status_t VL53L1X_GetSWVersion(VL53L1X_Version_t* ver);

// Set I2C address of device at <oldAddr> to <newAddr> (default addr is 0x29)
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_SetI2CAddress(uint16_t oldAddr, uint8_t newAddr);

// Load default values into sensor and restore temperature value,
// as well as ensure we can read from the sensor.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_SensorInit(uint16_t dev);

// Clear the interrupt after ranging so we can set another
// interrupt for the next ranging.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_ClearInterrupt(uint16_t dev);

// Set the interrupt polarity of the sensor.
// 1 = active high (default), 0 = active low.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_SetInterruptPolarity(uint16_t dev, uint8_t iPol);

// Get the interrupt polarity of the sensor.
// 1 = active high (default), 0 = active low.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetInterruptPolarity(uint16_t dev, uint8_t* iPol);

// Start the ranging (enable the sensor). Ranging is continous.
// The clear interrupt must be done after each time data is read
// to let the interrupt reset for the next data.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_StartRanging(uint16_t dev);

// Stop the ranging (disable the sensor)
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_StopRanging(uint16_t dev);

// Check if new ranging data is available, setting isDataReady to 0/1.
// isDataReady is 1 if ready, 0 if not.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_CheckForDataReady(uint16_t dev, uint8_t* isDataReady);

// Set the timing budget of the sensor in ms.
// Must be from the following set of values:
// 		15, 20, 33, 50, 100(default), 200, 500
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_SetTimingBudgetInMs(uint16_t dev, uint16_t timingBudgetMs);

// Get the current timing budget in ms.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetTimingBudgetInMs(uint16_t dev, uint16_t* timingBudgetMs);

// Set the distance mode of the sensor (1 = short, 2 = long(default)).
// Short mode measures up to 1.3m, with better ambient immunity.
// Long mode ranges up to 4m in the dark with 200ms timing budget.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_SetDistanceMode(uint16_t dev, uint16_t distanceMode);

// Get the current distance mode (1 = short, 2 = long).
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetDistanceMode(uint16_t dev, uint16_t* distanceMode);

// Set the intermeasurement period in ms (time beteen measurements).
// MUST BE >= timing budget (not verified by API).
// Can take on same values as timing budget (default is 100ms).
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_SetInterMeasurementInMs(uint16_t dev, uint32_t IM);

// Get the current intermeasurement period in ms.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetInterMeasurementInMs(uint16_t dev, uint16_t* IM);

// Get boot state of the device (1 = booted, 0 = not booted).
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_BootState(uint16_t dev, uint8_t* state);

// Get the sensor ID (model ID and type).
// Should be 0xEACC for VL53L1X.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetSensorId(uint16_t dev, uint16_t* id);

// Get the distance measured by the sensor, in mm.
// (Note: recommended to use GetResult instead)
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetDistance(uint16_t dev, uint16_t* distance);

// Get the returned signal per SPAD (single photon avalanche diode)
// in kcps/SPAD (kilo counts per sec)
// (Note: recommended to use GetResult instead)
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetSignalPerSpad(uint16_t dev, uint16_t* signalPerSp);

// Get the ambient per SPAD (single photon avalanche diode)
// in kcps/SPAD (kilo counts per sec)
// (Note: recommended to use GetResult instead)
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetAmbientPerSpad(uint16_t dev, uint16_t* amb);

// Get signal rate in kpcs.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetSignalRate(uint16_t dev, uint16_t* signalRate);

// Get the current number of enabled SPADs.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetSpadNb(uint16_t dev, uint16_t* spNb);

// Get the ambient rate in kpcs.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetAmbientRate(uint16_t dev, uint16_t* ambRate);

// Returns the ranging status code (success, warning codes, error codes).
// From the  VL53L1X ultra lite driver manual (UM2510):
// There are five range statuses: 0, 1, 2, 4, and 7. When the range status is 0, there is no error. Range status 1 and
// 2 are error warnings while range status 4 and 7 are errors.
// When the range status is 1, there is a sigma failure. This means that the repeatability or standard deviation of the
// measurement is bad due to a decreasing signal noise ratio. Increasing the timing budget can improve the
// standard deviation and avoid a range status 1.
// When the range status is 2, there is a signal failure. This means that the return signal is too week to return a good
// answer. The reason is because the target is too far, or the target is not reflective enough, or the target is too
// small. Increasing the timing buget might help, but there may simply be no target available.
// When the range status is 4, the sensor is "out of bounds". This means that the sensor is ranging in a “nonappropriated” zone and the measured result may be inconsistent. This status is considered as a warning but, in
// general, it happens when a target is at the maximum distance possible from the sensor, i.e. around 5 m. However,
// this is only for very bright targets.
// Range status 7 is called "wraparound". This situation may occur when the target is very reflective and the
// distance to the target/sensor is longer than the physical limited distance measurable by the sensor. Such
// distances include approximately 5 m when the senor is in Long distance mode and approximately 1.3 m when the
// sensor is in Short distance mode. Example: a traffic sign located at 6 m can be seen by the sensor and returns a
// range of 1 m. This is due to “radar aliasing”: if only an approximate distance is required, we may add 6 m to the
// distance returned. However, that is a very approximate estimation.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetRangeStatus(uint16_t dev, uint8_t* rangeStatus);

// Get the measurements and status in a result type for a single read.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetResult(uint16_t dev, VL53L1X_Result_t* result);

// Set the offset correction value in mm.
// Note: this can be calibrated with the VL53L1X_calibration functions.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_SetOffset(uint16_t dev, int16_t offset);

// Get the programmed offset correction in mm.
// Note: this can be calibrated with the VL53L1X_calibration functions.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetOffset(uint16_t dev, int16_t* offset);

// Set the xtalk correction value in cps (counts per second).
// This is the number of photons reflected back from the cover glass in cps.
// Note: this can be calibrated with the VL53L1X_calibration functions.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_SetXtalk(uint16_t dev, uint16_t XtalkValue);

// Get the current programmed xtalk correction (in cps)
// Note: this can be calibrated with the VL53L1X_calibration functions.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetXtalk(uint16_t dev, uint16_t* Xtalk);

// Set the threshold detection mode.
// <threshLow> is the threshold under which the device raises an interrupt if window=0.
// <threshHigh> is the threshold above which the device raises an interrupt if window=1.
// <window> is the detection mode (0 = below, 1 = above, 2 = out, 3 = in)
// <iOnNoTarget> is unused (set to 0)
// Example usage:
// 		- Detect below 100: (dev,100,300,0,1)
// 		- Detect above 300: (dev,100,300,1,1)
//		- Detect out of window: (dev,100,300,2,1)
//		- Detect in window: (dev,100,300,2,1)
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_SetDistanceThreshold(uint16_t dev, uint16_t threshLow, uint16_t threshHigh, uint8_t window, uint8_t iOnNoTarget);

// Get the window detection mode
// (0 = below, 1 = above, 2 = out, 3 = in)
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetDistanceThresholdWindow(uint16_t dev, uint16_t* window);

// Get the low threshold in mm.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetDistanceThresholdLow(uint16_t dev, uint16_t* low);

// Get the high threshold in mm.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetDistanceThresholdHigh(uint16_t dev, uint16_t* high);

// Set the ROI (Region of Interest) width and height.
// The center can be modified with VL53L1X_SetROICenter,
// and these changes only change the size of the ROI.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_SetROI(uint16_t dev, uint16_t X, uint16_t Y);

// Get the width and height of the ROI.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetROI_XY(uint16_t dev, uint16_t* ROI_x, uint16_t* ROI_y);

// Set the new user ROI center.
// WARNING: there is no check in this function, so if the ROI center/ROI size
// is out of the border of the ranging function, it will return error 13.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_SetROICenter(uint16_t dev, uint8_t ROICenter);

// Get the current user ROI center.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetROICenter(uint16_t dev, uint8_t* ROICenter);

// Set a new signal threshold in kcps (default = 1024kcps).
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_SetSignalThreshold(uint16_t dev, uint16_t signal);

// Get current signal threshold in kcps.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetSignalThreshold(uint16_t dev, uint16_t* signal);

// Set a new sigma threshold in mm (default = 15mm).
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_SetSigmaThreshold(uint16_t dev, uint16_t sigma);

// Get the current sigma threshold in mm.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_GetSigmaThreshold(uint16_t dev, uint16_t* sigma);

// Perform temperature calibration on the sensor
// This is recommended to run if the temperature might have changed by >8C
// without ranging activity for some time.
// Returns 0 on success, non-zero on failure.
VL53L1X_Status_t VL53L1X_StartTemperatureUpdate(uint16_t dev);


#endif  // _VL53L1X_API_H_
