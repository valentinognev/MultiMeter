/*
  This is a library written for the ST VL53L5CX Time-of-flight sensor
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/18642

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, October 22nd, 2021
  This file implements the VL53L5CX I2C driver class.

  This library uses ST's VL53L5CX driver and parts of Simon Levy's VL53L5CX
  Arduino library available at https://github.com/simondlevy/VL53L5/tree/main/src/st

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "SparkFun_VL53L5CX_IO.h"
#include "SparkFun_VL53L5CX_Library_Constants.h"


bool SparkFun_VL53L5CX_IO::begin(byte address)
{
    _address = address;
    return isConnected();
}

bool SparkFun_VL53L5CX_IO::isConnected()
{
    // _i2cPort->beginTransmission(_address);
    // if (_i2cPort->endTransmission() != 0)
    //     return (false);
    return (true);
}

void SparkFun_VL53L5CX_IO::setAddress(uint8_t newAddress)
{
    _address = newAddress;
}

// Must be able to write 32,768 bytes at a time
uint8_t SparkFun_VL53L5CX_IO::writeMultipleBytes(uint16_t registerAddress, uint8_t *buffer, uint16_t bufferSize)
{
   // Chunk I2C transactions into limit of 32 bytes (or wireMaxPacketSize)
    uint8_t i2cError = 0;
    uint32_t startSpot = 0;
    uint32_t bytesToSend = bufferSize;
    return i2cdev.writeBytes(_address, registerAddress, bufferSize, buffer);
}

uint8_t SparkFun_VL53L5CX_IO::readMultipleBytes(uint16_t registerAddress, uint8_t *buffer, uint16_t bufferSize)
{
    uint8_t i2cError = 0;
    return i2cdev.readBytes(_address, registerAddress, bufferSize, buffer) == bufferSize;
}

uint8_t SparkFun_VL53L5CX_IO::readSingleByte(uint16_t registerAddress)
{
    uint8_t readbyte = 0;
    i2cdev.readByte(_address, registerAddress, &readbyte);
    return readbyte;
}

uint8_t SparkFun_VL53L5CX_IO::writeSingleByte(uint16_t registerAddress, uint8_t const value)
{
    return i2cdev.writeByte(_address, registerAddress, value);
}
