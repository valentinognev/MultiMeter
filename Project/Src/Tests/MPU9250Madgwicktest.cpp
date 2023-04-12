// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//		2023-03-10 - Fit to esp-idf v5
//		2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//		2016-04-18 - Eliminated a potential infinite loop
//		2013-05-08 - added seamless Fastwire support
//				   - added note about gyro calibration
//		2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//		2012-06-20 - improved FIFO overflow handling and simplified read process
//		2012-06-19 - completely rearranged DMP initialization code and simplification
//		2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//		2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//		2012-06-05 - add gravity-compensated initial reference frame acceleration output
//				   - add 3D math helper file to DMP6 example sketch
//				   - add Euler output and Yaw/Pitch/Roll output formats
//		2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//		2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//		2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#include "math.h"

#include "helper_3dmath.h"
#include "SWO.h"
#include <stdio.h>
#include "SensorTestRoutines.h"

#include "parameter.h"

static const char *TAG = "IMU";

#define CONFIG_MAGX 0
#define CONFIG_MAGY 0
#define CONFIG_MAGZ 0
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "AK8963.h"

// Source: https://github.com/arduino-libraries/MadgwickAHRS
#include "MadgwickAHRS.h"

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD 0.0174533

#define MAG_ADDRESS 0x0C

static MPU6050 mpu;
static AK8963 mag(MAG_ADDRESS);
static Madgwick madgwick;

// Accel & Gyro scale factor
static float accel_sensitivity;
static float gyro_sensitivity;

// MAG Data Sensitivity adjustment data
// Sensitivity adjustment data for each axis is stored to fuse ROM on shipment.
static uint8_t MagAdjustmentValue[3];
static float magCalibration[3];
static I2Cdev i2cdev;
static char buffer[100];

static bool getMagRaw(int16_t *mx, int16_t *my, int16_t *mz)
{
	uint8_t rawData[7];
	for (int i = 0; i < 7; i++)
	{
		uint8_t reg = i + 0x03;
		uint8_t _raw;
		i2cdev.readByte(MAG_ADDRESS, reg, &_raw);
		rawData[i] = _raw;
		sprintf(buffer, "read_mag(0x%d)=%x", reg, rawData[i]);
		PrintString(buffer);
	}

	if (rawData[6] & 0x08)
	{
		sprintf(buffer, "*****magnetic sensor overflow*****");
		PrintString(buffer);
		return false;
	}

#if 0
	*mx = ((int16_t)rawData[0] << 8) | rawData[1];
	*my = ((int16_t)rawData[2] << 8) | rawData[3];
	*mz = ((int16_t)rawData[4] << 8) | rawData[5];
#else
	*mx = ((int16_t)rawData[1] << 8) | rawData[0];
	*my = ((int16_t)rawData[3] << 8) | rawData[2];
	*mz = ((int16_t)rawData[5] << 8) | rawData[4];
#endif
	sprintf(buffer, "mx=0x%x my=0x%x mz=0x%x", *mx, *my, *mz);
	PrintString(buffer);
	return true;
}

static bool getMagData(int16_t *mx, int16_t *my, int16_t *mz)
{
	sprintf(buffer, "mag.getDeviceID()=0x%x", mag.getDeviceID());
	PrintString(buffer);
	if (mag.getDeviceID() != 0x48)
	{
		sprintf(buffer, "*****AK8963 connection lost*****");
		PrintString(buffer);
		sprintf(buffer, "mag.getDeviceID()=0x%x", mag.getDeviceID());
		PrintString(buffer);
		// Bypass Enable Configuration
		mpu.setI2CBypassEnabled(true);
		LL_mDelay(100);
		return false;
	}

	sprintf(buffer, "mag.getMode()=0x%x", mag.getMode());
	PrintString(buffer);
	if (mag.getMode() != 0x06)
	{
		sprintf(buffer, "*****AK8963 illegal data mode*****");
		PrintString(buffer);
		sprintf(buffer, "mag.getMode()=0x%x", mag.getMode());
		PrintString(buffer);
		// Bypass Enable Configuration
		mpu.setI2CBypassEnabled(true);
		LL_mDelay(100);
		return false;
	}

	// Wait until DataReady
	sprintf(buffer, "mag.getDataReady()=0x%x", mag.getDataReady());
	PrintString(buffer);
	for (int retry = 0; retry < 10; retry++)
	{
		if (mag.getDataReady())
			break;
		LL_mDelay(1);
	}

	if (mag.getDataReady())
	{
		if (getMagRaw(mx, my, mz))
		{
			return true;
		}
		else
		{
			sprintf(buffer, "*****AK8963 magnetic sensor overflow*****");
			PrintString(buffer);
			return false;
		}
	}
	else
	{
		sprintf(buffer, "*****AK8963 data not ready*****");
		PrintString(buffer);
		sprintf(buffer, "mag.getDataReady()=0x%x", mag.getDataReady());
		PrintString(buffer);
		// LL_mDelay(10);
		return false;
	}
	return false;
}
// Get scaled value
static void _getMotion6(float *_ax, float *_ay, float *_az, float *_gx, float *_gy, float *_gz)
{
	int16_t ax, ay, az;
	int16_t gx, gy, gz;
	// read raw accel/gyro measurements from device
	// The accelerometer output is a 16-bit signed integer relative value.
	// The gyroscope output is a relative value in degrees per second (dps).
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	// Convert relative to absolute
#if 1
	*_ax = (float)ax / accel_sensitivity;
	*_ay = (float)ay / accel_sensitivity;
	*_az = (float)az / accel_sensitivity;
#else
	*_ax = (float)ax;
	*_ay = (float)ay;
	*_az = (float)az;
#endif

	// Convert relative degree per second to absolute radian per second
#if 1
	*_gx = ((float)gx / gyro_sensitivity);
	*_gy = ((float)gy / gyro_sensitivity);
	*_gz = ((float)gz / gyro_sensitivity);
#else
	*_gx = (float)gx;
	*_gy = (float)gy;
	*_gz = (float)gz;
#endif
}

// Get time in seconds since boot
// Compatible with ROS's time.toSec() function
static double TimeToSec()
{
	int64_t _time = HAL_GetTick(); // Get time in microseconds since boot
	double __time = (double)_time / HAL_GetTickFreq();
	return __time;
}

void MPU9250Madgwicktest()
{
	// Initialize mpu6050
	mpu.initialize();

	// Get the sample rate
	sprintf(buffer, "getRate()=%d", mpu.getRate());
	PrintString(buffer);
	// Set the sample rate to 8kHz
	if (mpu.getRate() != 0)
		mpu.setRate(0);

	// Get FSYNC configuration value
	sprintf(buffer, "getExternalFrameSync()=%d", mpu.getExternalFrameSync());
	PrintString(buffer);
	// Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering
	if (mpu.getExternalFrameSync() != 0)
		mpu.setExternalFrameSync(0);

	// Set Digital Low Pass Filter
	sprintf(buffer, "getDLPFMode()=%d", mpu.getDLPFMode());
	PrintString(buffer);
	if (mpu.getDLPFMode() != 6)
		mpu.setDLPFMode(6);

	// Get Accelerometer Scale Range
	sprintf(buffer, "getFullScaleAccelRange()=%d", mpu.getFullScaleAccelRange());
	PrintString(buffer);
	// Set Accelerometer Full Scale Range to ±2g
	if (mpu.getFullScaleAccelRange() != 0)
		mpu.setFullScaleAccelRange(0);
	accel_sensitivity = 16384.0;

	// Get Gyro Scale Range
	sprintf(buffer, "getFullScaleGyroRange()=%d", mpu.getFullScaleGyroRange());
	PrintString(buffer);
	// Set Gyro Full Scale Range to ±250deg/s
	if (mpu.getFullScaleGyroRange() != 0)
		mpu.setFullScaleGyroRange(0);
	gyro_sensitivity = 131.0;

	// Bypass Enable Configuration
	mpu.setI2CBypassEnabled(true);

	// Get MAG Device ID
	uint8_t MagDeviceID = mag.getDeviceID();
	sprintf(buffer, "MagDeviceID=0x%x", MagDeviceID);
	PrintString(buffer);
	if (MagDeviceID != 0x48)
	{
		sprintf(buffer, "AK8963 not found");
		PrintString(buffer);
		return;
	}

	// Goto Powerdown Mode
	mag.setMode(0x00);
	// After power-down mode is set, at least 100ms(Twat) is needed before setting another mode.
	LL_mDelay(200);

	// Goto Fuse ROM access mode
	mag.setMode(0x0F);
	LL_mDelay(200);

	// Get Sensitivity adjustment value from fuse ROM
	// Sensitivity adjustment values for each axis are written in the fuse ROM at the time of shipment.
	MagAdjustmentValue[0] = mag.getAdjustmentX();
	MagAdjustmentValue[1] = mag.getAdjustmentY();
	MagAdjustmentValue[2] = mag.getAdjustmentZ();
	sprintf(buffer, "MagAdjustmentValue: %x %x %x", MagAdjustmentValue[0], MagAdjustmentValue[1], MagAdjustmentValue[2]);
	PrintString(buffer);
   
	// Calculate sensitivity
	// from datasheet 8.3.11
	for (int i = 0; i < 3; i++)
	{
		magCalibration[i] = (float)(MagAdjustmentValue[i] - 128) / 256.0f + 1.0f;
		sprintf(buffer, "magCalibration[%d]=%f", i, magCalibration[i]);
		PrintString(buffer);
	}

	// Goto Powerdown Mode
	mag.setMode(0x00);
	LL_mDelay(200);

	// Goto oparation mode
	// mode1:Automatically repeat sensor measurements at 8Hz
	// mode2:Automatically repeat sensor measurements at 100Hz
	// 0x?0:Power doen mode.
	// 0x?1:Single measurement mode.
	// 0x?2:Continuous measurement mode 1.
	// 0x?6:Continuous measurement mode 2.
	// 0x?8:External trigger measurement mode.
	// 0x0?:14 bit output 0.60 uT/LSB --> 1 = 0.60uT
	// 0x1?:16 bit output 0.15 uT/LSB --> 1 = 0.15uT
	mag.setMode(0x16);
	// mag.setMode(0x06);

	double last_time_ = TimeToSec();
	int elasped = 0;
	bool initialized = false;
	float initial_roll = 0.0;
	float initial_pitch = 0.0;
	float initial_yaw = 0.0;

	float roll = 0.0, pitch = 0.0, yaw = 0.0;
	float _roll = 0.0, _pitch = 0.0, _yaw = 0.0;
	while (1)
	{
		// Get scaled value
		float ax, ay, az;
		float gx, gy, gz;
		_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

		int16_t mx, my, mz;
		float _mx, _my, _mz;
		if (getMagData(&mx, &my, &mz))
		{
			sprintf(buffer, "mag=%d %d %d", mx, my, mz);
			PrintString(buffer);
			mx = mx + CONFIG_MAGX;
			my = my + CONFIG_MAGY;
			mz = mz + CONFIG_MAGZ;
			// adjust sensitivity
			// from datasheet 8.3.11
#if 0
			_mx = mx * 0.15 * magCalibration[0];
			_my = my * 0.15 * magCalibration[1];
			_mz = mz * 0.15 * magCalibration[2];
#endif
			_mx = mx * magCalibration[0];
			_my = my * magCalibration[1];
			_mz = mz * magCalibration[2];
			sprintf(buffer, "mag=%f %f %f", _mx, _my, _mz);
			PrintString(buffer);

			// Get the elapsed time from the previous
			float dt = (TimeToSec() - last_time_);
			sprintf(buffer, "dt=%f", dt);
			PrintString(buffer);
			last_time_ = TimeToSec();

			// Get Euler
			madgwick.update(gx, gy, gz, ax, ay, az, _mx, _my, _mz, dt);
			roll = madgwick.getRoll();
			pitch = madgwick.getPitch();
			yaw = madgwick.getYaw();
			sprintf(buffer, "roll=%f pitch=%f yaw=%f", roll, pitch, yaw);
			PrintString(buffer);
		}

		/* Print Data every 10 times */
		if (elasped > 10)
		{
			// Set the first data
			uint32_t nowTicks = HAL_GetTick();
			if (initialized == false && nowTicks > 6000)
			{
				initial_roll = roll;
				initial_pitch = pitch;
				initial_yaw = yaw;
				initialized = true;
			}
			_roll = roll - initial_roll;
			_pitch = pitch - initial_pitch;
			_yaw = yaw - initial_yaw;
			if (_yaw < -180.0)
				_yaw = _yaw + 360.0;
			sprintf(buffer, "roll=%f pitch=%f yaw=%f", roll, pitch, yaw);
			sprintf(buffer, "roll:%f pitch=%f yaw=%f", _roll, _pitch, _yaw);

			if (initialized)
			{
				sprintf(buffer, "roll:%f pitch=%f yaw=%f", _roll, _pitch, _yaw);
				// Send UDP packet
				POSE_t pose;
				pose.roll = _roll;
				pose.pitch = _pitch;
				pose.yaw = _yaw;
				// if (xQueueSend(xQueueTrans, &pose, 100) != pdPASS)
				// {
				// 	ESP_LOGE(pcTaskGetName(NULL), "xQueueSend fail");
				// }

				// // Send WEB request
				// cJSON *request;
				// request = cJSON_CreateObject();
				// cJSON_AddStringToObject(request, "id", "data-request");
				// cJSON_AddNumberToObject(request, "roll", _roll);
				// cJSON_AddNumberToObject(request, "pitch", _pitch);
				// cJSON_AddNumberToObject(request, "yaw", _yaw);
				// char *my_json_string = cJSON_Print(request);
				// sprintf(buffer, "my_json_string\n%s", my_json_string);
				// size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
				// if (xBytesSent != strlen(my_json_string))
				// {
				// 	sprintf(buffer, "xMessageBufferSend fail");
				// }
				// cJSON_Delete(request);
				// cJSON_free(my_json_string);
			}
			else
			{
				sprintf(buffer, "unstable roll:%f pitch=%f yaw=%f", _roll, _pitch, _yaw);
			}

			LL_mDelay(1);
			elasped = 0;
		}

		elasped++;
		LL_mDelay(1);
	} // end while

	// Never reach here
	// vTaskDelete(NULL);
}
