/*
  BMP085 / BMP180 Barometric Pressure & Temperature Sensor. Simple Example (Integer equations)
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/czujniki-cisnienia-bmp085-bmp180.html
  GIT: https://github.com/jarzebski/Arduino-BMP085-BMP180
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
*/

#include <BMP085.h>
#include <stdio.h>
#include <stdint.h>
#include "SensorTestRoutines.h"

BMP085 bmp;

double referencePressure;

static void loop(void);
static void checkSettings() ;

void BMP085test()
{
    // Initialize BMP085 or BMP180 sensor
    PrintString("Initialize BMP085/BMP180 Serial\n");

    // Ultra high resolution: BMP085_ULTRA_HIGH_RES
    // (default) High resolution: BMP085_HIGH_RES
    // Standard: BMP085_STANDARD
    // Ultra low power: BMP085_ULTRA_LOW_POWER
    while (!bmp.begin(BMP085_ULTRA_HIGH_RES))
    {
        PrintString("Could not find a valid BMP085 or BMP180 sensor, check wiring!\n");
        LL_mDelay(500);
    }

    // Enable or disable SOSS (Software oversampling)- Use with BMP085_ULTRA_HIGH_RES !
    // For applications where a low noise level is critical, averaging is recommended if the lower bandwidth is acceptable
    // Conversion time pressure: 76.5ms, RMS noise 0.02 hPA / 0.17 m
    // bmp.setSoftwareOversampling(0);

    // Get reference pressure for relative altitude
    referencePressure = bmp.readPressure();

    // Check settings
    checkSettings();

    while(1)
    {
        loop();
    }
}

static void checkSettings()
{
    char buf[100];
    sprintf(buf, "Chip version: %d.%d (0x%02X)", bmp.getVersion() >> 8, bmp.getVersion() & 0xFF, bmp.getVersion());
    PrintString(buf);

    sprintf(buf, "Oversampling: %d", bmp.getOversampling());
    PrintString(buf);
    sprintf(buf, "Software Oversampling: %d", bmp.getSoftwareOversampling());
    PrintString(buf);
}

static void loop()
{
    // Read raw values
    int rawTemp = bmp.readRawTemperature();
    uint32_t rawPressure = bmp.readRawPressure();

    // Read true temperature & Pressure
    double realTemperature = bmp.readTemperature();
    long realPressure = bmp.readPressure();

    // Calculate altitude
    float absoluteAltitude = bmp.getAltitude(realPressure);
    float relativeAltitude = bmp.getAltitude(realPressure, referencePressure);

    PrintString("--\n");
    char buf[100];
    sprintf(buf, " rawTemp = %d, realTemp = %.2f *C", rawTemp, realTemperature);
    PrintString(buf);

    sprintf(buf, " rawPressure = %d, realPressure = %ld Pa", rawPressure, realPressure);
    PrintString(buf);

    sprintf(buf, " absoluteAltitude = %.2f m, relativeAltitude = %.2f m", absoluteAltitude, relativeAltitude);
    PrintString(buf);

    LL_mDelay(100);
}
