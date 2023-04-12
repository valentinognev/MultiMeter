/*
    ADXL345 Triple Axis Accelerometer. Simple Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-akcelerometr-adxl345.html
    GIT: https://github.com/jarzebski/Arduino-ADXL345
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include "SensorTestRoutines.h"
#include <ADXL345.h>
#include "SWO.h"
#include "stdio.h"
#include "main.h"

static void loop(void);

ADXL345 accelerometer;

void showRange(void)
{
    PrintString("Selected measurement range: ");

    switch (accelerometer.getRange())
    {
    case ADXL345_RANGE_16G:
        PrintString("+/- 16 g\n");
        break;
    case ADXL345_RANGE_8G:
        PrintString("+/- 8 g\n");
        break;
    case ADXL345_RANGE_4G:
        PrintString("+/- 4 g\n");
        break;
    case ADXL345_RANGE_2G:
        PrintString("+/- 2 g\n");
        break;
    default:
        PrintString("Bad range\n");
        break;
    }
}

void showDataRate(void)
{
    PrintString("Selected data rate: ");

    switch (accelerometer.getDataRate())
    {
    case ADXL345_DATARATE_3200HZ:
        PrintString("3200 Hz\n");
        break;
    case ADXL345_DATARATE_1600HZ:
        PrintString("1600 Hz\n");
        break;
    case ADXL345_DATARATE_800HZ:
        PrintString("800 Hz\n");
        break;
    case ADXL345_DATARATE_400HZ:
        PrintString("400 Hz\n");
        break;
    case ADXL345_DATARATE_200HZ:
        PrintString("200 Hz\n");
        break;
    case ADXL345_DATARATE_100HZ:
        PrintString("100 Hz\n");
        break;
    case ADXL345_DATARATE_50HZ:
        PrintString("50 Hz\n");
        break;
    case ADXL345_DATARATE_25HZ:
        PrintString("25 Hz\n");
        break;
    case ADXL345_DATARATE_12_5HZ:
        PrintString("12.5 Hz\n");
        break;
    case ADXL345_DATARATE_6_25HZ:
        PrintString("6.25 Hz\n");
        break;
    case ADXL345_DATARATE_3_13HZ:
        PrintString("3.13 Hz\n");
        break;
    case ADXL345_DATARATE_1_56HZ:
        PrintString("1.56 Hz\n");
        break;
    case ADXL345_DATARATE_0_78HZ:
        PrintString("0.78 Hz\n");
        break;
    case ADXL345_DATARATE_0_39HZ:
        PrintString("0.39 Hz\n");
        break;
    case ADXL345_DATARATE_0_20HZ:
        PrintString("0.20 Hz\n");
        break;
    case ADXL345_DATARATE_0_10HZ:
        PrintString("0.10 Hz\n");
        break;
    default:
        PrintString("Bad data rate\n");
        break;
    }
}

void ADXL345test(void)
{
    // Initialize ADXL345
    PrintString("Initialize ADXL345\n");
    if (!accelerometer.begin())
    {
        PrintString("Could not find a valid ADXL345 sensor, check wiring!\n");
        LL_mDelay(500);
    }

    // Set measurement range
    // +/-  2G: ADXL345_RANGE_2G
    // +/-  4G: ADXL345_RANGE_4G
    // +/-  8G: ADXL345_RANGE_8G
    // +/- 16G: ADXL345_RANGE_16G
    accelerometer.setRange(ADXL345_RANGE_16G);

    // Show current setting
    showRange();
    showDataRate();
    
    while (1)
    {
        loop();
    }
    
}

static void loop(void)
{
    // Read normalized values
    Vector raw = accelerometer.readRaw();

    // Read normalized values
    Vector norm = accelerometer.readNormalize();

    char buf[100];
    // Output raw
    sprintf(buf, " Xraw = %d Yraw = %d Zraw: %d", raw.XAxis, raw.YAxis, raw.ZAxis);
    PrintString(buf);

    // Output normalized m/s^2
    sprintf(buf, " Xnorm = %f Ynorm = %f Znorm = %f", norm.XAxis, norm.YAxis, norm.ZAxis);
    PrintString(buf);

    PrintString("\n");
    LL_mDelay(100);
}