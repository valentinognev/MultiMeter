// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//        2023-03-10 - Fit to esp-idf v5
//        2019-07-08 - Added Auto Calibration and offset generator
//           - and altered FIFO retrieval sequence to avoid using blocking code
//        2016-04-18 - Eliminated a potential infinite loop
//        2013-05-08 - added seamless Fastwire support
//                   - added note about gyro calibration
//        2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//        2012-06-20 - improved FIFO overflow handling and simplified read process
//        2012-06-19 - completely rearranged DMP initialization code and simplification
//        2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//        2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//        2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                   - add 3D math helper file to DMP6 example sketch
//                   - add Euler output and Yaw/Pitch/Roll output formats
//        2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//        2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//        2012-05-30 - basic DMP initialization working

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

#include "parameter.h"
#include "SensorTestRoutines.h"

static const char *TAG = "IMU";
#define CONFIG_MAGX 0
#define CONFIG_MAGY 0
#define CONFIG_MAGZ 0

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <stdio.h>
#include "SWO.h"

#include "MPU6050.h"
#include "AK8963.h"

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

#define MAG_ADDRESS 0x0C

MPU6050 mpu;
AK8963 mag(MAG_ADDRESS);

// MAG Data Sensitivity adjustment data
// Sensitivity adjustment data for each axis is stored to fuse ROM on shipment.
uint8_t MagAdjustmentValue[3];
float magCalibration[3];
static I2Cdev i2cdev;
static char buf[256];

static bool getMagRaw(int16_t *mx, int16_t *my, int16_t *mz)
{
	uint8_t rawData[7];
    for (int i=0;i<7;i++) {
        uint8_t reg = i + 0x03;
        uint8_t _raw;
        i2cdev.readByte(MAG_ADDRESS, reg, &_raw);
        rawData[i] = _raw;
        sprintf(buf, "read_mag(0x%d)=%x", reg, rawData[i]);
        PrintString(buf);
    }

    if(rawData[6] & 0x08) {
        sprintf(buf, "*****magnetic sensor overflow*****");
        PrintString(buf);
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
    sprintf(buf, "mx=0x%x my=0x%x mz=0x%x", *mx, *my, *mz);
    PrintString(buf);
    return true;
}

static bool getMagData(int16_t *mx, int16_t *my, int16_t *mz)
{
	sprintf(buf, "mag.getDeviceID()=0x%x", mag.getDeviceID());
    if (mag.getDeviceID() != 0x48) {
        sprintf(buf, "*****AK8963 connection lost*****");
        PrintString(buf);
        sprintf(buf, "mag.getDeviceID()=0x%x", mag.getDeviceID());
        PrintString(buf);
        // Bypass Enable Configuration
        mpu.setI2CBypassEnabled(true);
        LL_mDelay(100);
        return false;
    }

    sprintf(buf, "mag.getMode()=0x%x", mag.getMode());
    if (mag.getMode() != 0x06) {
        sprintf(buf, "*****AK8963 illegal data mode*****");
        PrintString(buf);
        sprintf(buf, "mag.getMode()=0x%x", mag.getMode());
        PrintString(buf);
        // Bypass Enable Configuration
        mpu.setI2CBypassEnabled(true);
        LL_mDelay(100);
        return false;
    }

    // Wait until DataReady
    sprintf(buf, "mag.getDataReady()=0x%x", mag.getDataReady());
    PrintString(buf);
    for (int retry=0;retry<10;retry++) {
        if (mag.getDataReady()) break;
        LL_mDelay(1);
    }

    if (mag.getDataReady()) {
        if (getMagRaw(mx, my, mz)) {
            return true;
        } else {
            sprintf(buf, "*****AK8963 magnetic sensor overflow*****");
            PrintString(buf);
            return false;
        }
    } else {
        sprintf(buf, "*****AK8963 data not ready*****");
        PrintString(buf);
        sprintf(buf, "mag.getDataReady()=0x%x", mag.getDataReady());
        PrintString(buf);
        //LL_mDelay(10);
        return false;
    }
    return false;
}

void AK8963test()
{
	// Initialize mpu6050
    mpu.initialize();

    // Bypass Enable Configuration
    mpu.setI2CBypassEnabled(true);

    // Get MAG Device ID
    uint8_t MagDeviceID = mag.getDeviceID();
    sprintf(buf, "MagDeviceID=0x%x", MagDeviceID);
    if (MagDeviceID != 0x48) {
        sprintf(buf, "AK8963 not found");
        PrintString(buf);
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
    sprintf(buf, "MagAdjustmentValue: %x %x %x", MagAdjustmentValue[0], MagAdjustmentValue[1], MagAdjustmentValue[2]);
    PrintString(buf);
    // Calculate sensitivity
    // from datasheet 8.3.11
    for (int i=0;i<3;i++) {
        magCalibration[i] = (float)(MagAdjustmentValue[i] - 128)/256.0f + 1.0f;
        sprintf(buf, "magCalibration[%d]=%f",i, magCalibration[i]);
        PrintString(buf);
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
    //mag.setMode(0x06);

    while(1){

        int16_t mx, my, mz;
        float _mx, _my, _mz;
        if (getMagData(&mx, &my, &mz)) {
            sprintf(buf, "mag=%d %d %d", mx, my, mz);
            PrintString(buf);
            mx = mx + CONFIG_MAGX;
            my = my + CONFIG_MAGY;
            mz = mz + CONFIG_MAGZ;
            // adjust sensitivity
            // from datasheet 8.3.11
            _mx = mx * magCalibration[0];
            _my = my * magCalibration[1];
            _mz = mz * magCalibration[2];
            sprintf(buf, "mag=%f %f %f", _mx, _my, _mz);
            PrintString(buf);
            
            float __mx = _mx * 0.6;
            float __my = _my * 0.6;
            float __mz = _mz * 0.6;
            sprintf(buf, "mag[uT]=%f %f %f", __mx, __my, __mz);
            PrintString(buf);

            // Send WEB request
            // cJSON *request;
            // request = cJSON_CreateObject();
            // cJSON_AddStringToObject(request, "id", "data-request");
            // cJSON_AddNumberToObject(request, "roll", _mx);
            // cJSON_AddNumberToObject(request, "pitch", _my);
            // cJSON_AddNumberToObject(request, "yaw", _mz);
            // char *my_json_string = cJSON_Print(request);
            // sprintf(buf, "my_json_string\n%s",my_json_string);
            // size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
            // if (xBytesSent != strlen(my_json_string)) {
            //     sprintf(buf, "xMessageBufferSend fail");
            // }
            // cJSON_Delete(request);
            // cJSON_free(my_json_string);
            LL_mDelay(10);
        }
        LL_mDelay(1);
    } // end while

    // Never reach here
    // vTaskDelete(NULL);
}
