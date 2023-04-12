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

#include "parameter.h"
#include "helper_3dmath.h"
#include "SWO.h"
#include <stdio.h>
#include "SensorTestRoutines.h"

static const char *TAG = "IMU";

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

// #include "MPU6050.h" // not necessary if using MotionApps include file
#include "MPU6050_6Axis_MotionApps20.h"

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD 0.0174533

static MPU6050 mpu;

// MPU control/status vars
static bool dmpReady = false; // set true if DMP init was successful
static uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
static uint8_t devStatus;	  // return status after each device operation (0 = success, !0 = error)
static uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;		// count of all bytes currently in FIFO
static uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
static Quaternion q; // [w, x, y, z]			quaternion container
static VectorInt16 aa; // [x, y, z]			accel sensor measurements
static VectorInt16 aaReal; // [x, y, z]			gravity-free accel sensor measurements
static VectorInt16 aaWorld; // [x, y, z]			world-frame accel sensor measurements
static VectorFloat gravity; // [x, y, z]			gravity vector
static float euler[3];		// [psi, theta, phi]	Euler angle container
static float ypr[3];		// [yaw, pitch, roll]	yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
static uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

// display quaternion values in easy matrix form: w x y z
static void getQuaternion()
{
	char buffer[100];
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	sprintf(buffer,"quat x:%6.2f y:%6.2f z:%6.2f w:%6.2f\n", q.x, q.y, q.z, q.w);
	PrintString(buffer);
}

// display Euler angles in degrees
static void getEuler()
{
	char buffer[100];
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetEuler(euler, &q);
	sprintf(buffer, "euler psi:%6.2f theta:%6.2f phi:%6.2f\n", euler[0] * RAD_TO_DEG, euler[1] * RAD_TO_DEG, euler[2] * RAD_TO_DEG);
	PrintString(buffer);
}

// display Euler angles in degrees
static void getYawPitchRoll()
{
	char buffer[100];
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#if 0
	float _roll = ypr[2] * RAD_TO_DEG;
	float _pitch = ypr[1] * RAD_TO_DEG;
	float _yaw = ypr[0] * RAD_TO_DEG;
	sprintf(buffer, "roll:%f pitch:%f yaw:%f",_roll, _pitch, _yaw);
#endif
	// sprintf(buffer,"ypr roll:%3.1f pitch:%3.1f yaw:%3.1f\n",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
	sprintf(buffer, "roll:%f pitch:%f yaw:%f", ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
	PrintString(buffer);
}

// display real acceleration, adjusted to remove gravity
static void getRealAccel()
{
	char buffer[100];
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	sprintf(buffer,"areal x=%d y:%d z:%d\n", aaReal.x, aaReal.y, aaReal.z);
	PrintString(buffer);
}

// display initial world-frame acceleration, adjusted to remove gravity
// and rotated based on known orientation from quaternion
static void getWorldAccel()
{
	char buffer[100];
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
	sprintf(buffer,"aworld x:%d y:%d z:%d\n", aaWorld.x, aaWorld.y, aaWorld.z);
	PrintString(buffer);
}

void MPU6050DMP6test()
{
	char buffer[100];
	// Initialize mpu6050
	mpu.initialize();

#if 0
	// Get Device ID
	uint8_t DeviceID = mpu.getDeviceID();
	ESP_LOGI(TAG, "DeviceID=0x%x", DeviceID);
	if (DeviceID != 0x34) {
		sprintf(buffer, "MPU6050 not found");
		vTaskDelete(NULL);
	}
#endif

	// Initialize DMP
	devStatus = mpu.dmpInitialize();
	sprintf(buffer, "devStatus=%d", devStatus);
	PrintString(buffer);
	if (devStatus != 0)
	{
		sprintf(buffer, "DMP Initialization failed [%d]", devStatus);
		PrintString(buffer);
		while (1)
		{
			LL_mDelay(1);
		}
	}

	// This need to be setup individually
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXAccelOffset(-2889);
	mpu.setYAccelOffset(-444);
	mpu.setZAccelOffset(698);
	mpu.setXGyroOffset(149);
	mpu.setYGyroOffset(27);
	mpu.setZGyroOffset(17);

	// Calibration Time: generate offsets and calibrate our MPU6050
	mpu.CalibrateAccel(6);
	mpu.CalibrateGyro(6);
	mpu.setDMPEnabled(true);

	while (1)
	{
		if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
		{ // Get the Latest packet
			getYawPitchRoll();
			float _roll = ypr[2] * RAD_TO_DEG;
			float _pitch = ypr[1] * RAD_TO_DEG;
			float _yaw = ypr[0] * RAD_TO_DEG;

			// Send UDP packet
			POSE_t pose;
			pose.roll = _roll;
			pose.pitch = _pitch;
			pose.yaw = _yaw;

			// // Send WEB request
			// cJSON *request;
			// request = cJSON_CreateObject();
			// cJSON_AddStringToObject(request, "id", "data-request");
			// cJSON_AddNumberToObject(request, "roll", _roll);
			// cJSON_AddNumberToObject(request, "pitch", _pitch);
			// cJSON_AddNumberToObject(request, "yaw", _yaw);
			// char *my_json_string = cJSON_Print(request);
			// ESP_LOGD(TAG, "my_json_string\n%s", my_json_string);
			// size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
			// if (xBytesSent != strlen(my_json_string))
			// {
			// 	sprintf(buffer, "xMessageBufferSend fail");
			// }
			// cJSON_Delete(request);
			// cJSON_free(my_json_string);

			// getQuaternion();
			// getEuler();
			// getRealAccel();
			// getWorldAccel();
		}

		// Best result is to match with DMP refresh rate
		// Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
		// Now its 0x13, which means DMP is refreshed with 10Hz rate
		LL_mDelay(100);//LL_mDelay(100 / portTICK_PERIOD_MS);
	}
}
