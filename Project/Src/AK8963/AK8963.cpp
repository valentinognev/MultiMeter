// I2Cdev library collection - AK8963 I2C device class header file
// Based on AKM AK8963 datasheet, 10/2013
// 8/27/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2016-01-02 - initial release based on AK8975 code
//

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

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

#include "AK8963.h"

#if 0
/** Default constructor, uses default I2C address.
 * @see AK8963_DEFAULT_ADDRESS
 */
AK8963::AK8963() {
    devAddr = AK8963_DEFAULT_ADDRESS;
}
#endif

/** Specific address constructor.
 * @param address I2C address
 * @see AK8963_DEFAULT_ADDRESS
 * @see AK8963_ADDRESS_00
 */
AK8963::AK8963(uint8_t address) {
    devAddr = address;
}

/** Power on and prepare for general usage.
 * No specific pre-configuration is necessary for this device.
 */
void AK8963::initialize() {
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool AK8963::testConnection() {
    if (i2cdev.readByte(devAddr, AK8963_RA_WIA, buffer) == 1) {
        return (buffer[0] == 0x48);
    }
    return false;
}

// WIA register

uint8_t AK8963::getDeviceID() {
    i2cdev.readByte(devAddr, AK8963_RA_WIA, buffer);
    return buffer[0];
}

// INFO register

uint8_t AK8963::getInfo() {
    i2cdev.readByte(devAddr, AK8963_RA_INFO, buffer);
    return buffer[0];
}

// ST1 register

bool AK8963::getDataReady() {
    i2cdev.readBit(devAddr, AK8963_RA_ST1, AK8963_ST1_DRDY_BIT, buffer);
    return buffer[0];
}

bool AK8963::getDataOverrun() {
    i2cdev.readBit(devAddr, AK8963_RA_ST1, AK8963_ST1_DOR_BIT, buffer);
    return buffer[0];
}

// H* registers
void AK8963::getHeading(int16_t *x, int16_t *y, int16_t *z) {
    i2cdev.readBytes(devAddr, AK8963_RA_HXL, 6, buffer);
    *x = (((int16_t)buffer[1]) << 8) | buffer[0];
    *y = (((int16_t)buffer[3]) << 8) | buffer[2];
    *z = (((int16_t)buffer[5]) << 8) | buffer[4];
}
int16_t AK8963::getHeadingX() {
    i2cdev.readBytes(devAddr, AK8963_RA_HXL, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}
int16_t AK8963::getHeadingY() {
    i2cdev.readBytes(devAddr, AK8963_RA_HYL, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}
int16_t AK8963::getHeadingZ() {
    i2cdev.readBytes(devAddr, AK8963_RA_HZL, 2, buffer);
    return (((int16_t)buffer[1]) << 8) | buffer[0];
}

// ST2 register
bool AK8963::getOverflowStatus() {
    i2cdev.readBit(devAddr, AK8963_RA_ST2, AK8963_ST2_HOFL_BIT, buffer);
    return buffer[0];
}
bool AK8963::getOutputBit() {
    i2cdev.readBit(devAddr, AK8963_RA_ST2, AK8963_ST2_BITM_BIT, buffer);
    return buffer[0];
}

// CNTL1 register
uint8_t AK8963::getMode() {
    i2cdev.readBits(devAddr, AK8963_RA_CNTL1, AK8963_CNTL1_MODE_BIT, AK8963_CNTL1_MODE_LENGTH, buffer);
    return buffer[0];
}
void AK8963::setMode(uint8_t mode) {
    i2cdev.writeBits(devAddr, AK8963_RA_CNTL1, AK8963_CNTL1_MODE_BIT, AK8963_CNTL1_MODE_LENGTH, mode);
}
uint8_t AK8963::getResolution() {
    i2cdev.readBit(devAddr, AK8963_RA_CNTL1, AK8963_CNTL1_RES_BIT, buffer);
    return buffer[0];
}
void AK8963::setResolution(uint8_t res) {
    i2cdev.writeBit(devAddr, AK8963_RA_CNTL1, AK8963_CNTL1_RES_BIT, res);
}

// CNTL2 register
void AK8963::reset() {
    i2cdev.writeByte(devAddr, AK8963_RA_CNTL2, AK8963_CNTL2_RESET);
}

// ASTC register
void AK8963::setSelfTest(bool enabled) {
    i2cdev.writeBit(devAddr, AK8963_RA_ASTC, AK8963_ASTC_SELF_BIT, enabled);
}

// I2CDIS
void AK8963::disableI2C() {
    i2cdev.writeByte(devAddr, AK8963_RA_I2CDIS, AK8963_I2CDIS_DISABLE);
}

// ASA* registers
void AK8963::getAdjustment(int8_t *x, int8_t *y, int8_t *z) {
    i2cdev.readBytes(devAddr, AK8963_RA_ASAX, 3, buffer);
    *x = buffer[0];
    *y = buffer[1];
    *z = buffer[2];
}
void AK8963::setAdjustment(int8_t x, int8_t y, int8_t z) {
    buffer[0] = x;
    buffer[1] = y;
    buffer[2] = z;
    i2cdev.writeBytes(devAddr, AK8963_RA_ASAX, 3, buffer);
}
uint8_t AK8963::getAdjustmentX() {
    i2cdev.readByte(devAddr, AK8963_RA_ASAX, buffer);
    return buffer[0];
}
void AK8963::setAdjustmentX(uint8_t x) {
    i2cdev.writeByte(devAddr, AK8963_RA_ASAX, x);
}
uint8_t AK8963::getAdjustmentY() {
    i2cdev.readByte(devAddr, AK8963_RA_ASAY, buffer);
    return buffer[0];
}
void AK8963::setAdjustmentY(uint8_t y) {
    i2cdev.writeByte(devAddr, AK8963_RA_ASAY, y);
}
uint8_t AK8963::getAdjustmentZ() {
    i2cdev.readByte(devAddr, AK8963_RA_ASAZ, buffer);
    return buffer[0];
}
void AK8963::setAdjustmentZ(uint8_t z) {
    i2cdev.writeByte(devAddr, AK8963_RA_ASAZ, z);
}
