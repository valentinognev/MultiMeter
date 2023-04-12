/*
  HMC5883L Triple Axis Digital Compass. Simple Example.
  Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
  GIT: https://github.com/jarzebski/Arduino-HMC5883L
  Web: http://www.jarzebski.pl
  (c) 2014 by Korneliusz Jarzebski
*/

#include <HMC5883L.h>
#include "SWO.h"
#include "main.h"
#include <stdio.h>
#include "SensorTestRoutines.h"


static HMC5883L compass;
void checkSettings() ;
static void loop();


void HMC5883Ltest()
{
    // Initialize HMC5883L
    PrintString("Initialize HMC5883L");

    // Set measurement range
    // +/- 0.88 Ga: HMC5883L_RANGE_0_88GA
    // +/- 1.30 Ga: HMC5883L_RANGE_1_3GA (default)
    // +/- 1.90 Ga: HMC5883L_RANGE_1_9GA
    // +/- 2.50 Ga: HMC5883L_RANGE_2_5GA
    // +/- 4.00 Ga: HMC5883L_RANGE_4GA
    // +/- 4.70 Ga: HMC5883L_RANGE_4_7GA
    // +/- 5.60 Ga: HMC5883L_RANGE_5_6GA
    // +/- 8.10 Ga: HMC5883L_RANGE_8_1GA
    compass.setRange(HMC5883L_RANGE_1_3GA);

    // Set measurement mode
    // Idle mode:              HMC5883L_IDLE
    // Single-Measurement:     HMC5883L_SINGLE
    // Continuous-Measurement: HMC5883L_CONTINOUS (default)
    compass.setMeasurementMode(HMC5883L_CONTINOUS);

    // Set data rate
    //  0.75Hz: HMC5883L_DATARATE_0_75HZ
    //  1.50Hz: HMC5883L_DATARATE_1_5HZ
    //  3.00Hz: HMC5883L_DATARATE_3HZ
    //  7.50Hz: HMC5883L_DATARATE_7_50HZ
    // 15.00Hz: HMC5883L_DATARATE_15HZ (default)
    // 30.00Hz: HMC5883L_DATARATE_30HZ
    // 75.00Hz: HMC5883L_DATARATE_75HZ
    compass.setDataRate(HMC5883L_DATARATE_15HZ);

    // Set number of samples averaged
    // 1 sample:  HMC5883L_SAMPLES_1 (default)
    // 2 samples: HMC5883L_SAMPLES_2
    // 4 samples: HMC5883L_SAMPLES_4
    // 8 samples: HMC5883L_SAMPLES_8
    compass.setSamples(HMC5883L_SAMPLES_1);

    // Check settings
    checkSettings();

    while (1)
    {
        loop();
    }
}

void checkSettings()
{
    PrintString("Selected range: ");

    switch (compass.getRange())
    {
    case HMC5883L_RANGE_0_88GA:
        PrintString("0.88 Ga\n");
        break;
    case HMC5883L_RANGE_1_3GA:
        PrintString("1.3 Ga\n");
        break;
    case HMC5883L_RANGE_1_9GA:
        PrintString("1.9 Ga\n");
        break;
    case HMC5883L_RANGE_2_5GA:
        PrintString("2.5 Ga\n");
        break;
    case HMC5883L_RANGE_4GA:
        PrintString("4 Ga\n");
        break;
    case HMC5883L_RANGE_4_7GA:
        PrintString("4.7 Ga\n");
        break;
    case HMC5883L_RANGE_5_6GA:
        PrintString("5.6 Ga\n");
        break;
    case HMC5883L_RANGE_8_1GA:
        PrintString("8.1 Ga\n");
        break;
    default:
        PrintString("Bad range!");
    }

    PrintString("Selected Measurement Mode: ");
    switch (compass.getMeasurementMode())
    {
    case HMC5883L_IDLE:
        PrintString("Idle mode\n");
        break;
    case HMC5883L_SINGLE:
        PrintString("Single-Measurement\n");
        break;
    case HMC5883L_CONTINOUS:
        PrintString("Continuous-Measurement");
        break;
    default:
        PrintString("Bad mode!\n");
    }

    PrintString("Selected Data Rate: ");
    switch (compass.getDataRate())
    {
    case HMC5883L_DATARATE_0_75_HZ:
        PrintString("0.75 Hz\n");
        break;
    case HMC5883L_DATARATE_1_5HZ:
        PrintString("1.5 Hz\n");
        break;
    case HMC5883L_DATARATE_3HZ:
        PrintString("3 Hz\n");
        break;
    case HMC5883L_DATARATE_7_5HZ:
        PrintString("7.5 Hz\n");
        break;
    case HMC5883L_DATARATE_15HZ:
        PrintString("15 Hz\n");
        break;
    case HMC5883L_DATARATE_30HZ:
        PrintString("30 Hz\n");
        break;
    case HMC5883L_DATARATE_75HZ:
        PrintString("75 Hz\n");
        break;
    default:
        PrintString("Bad data rate!");
    }

    PrintString("Selected number of samples: ");
    switch (compass.getSamples())
    {
    case HMC5883L_SAMPLES_1:
        PrintString("1\n");
        break;
    case HMC5883L_SAMPLES_2:
        PrintString("2\n");
        break;
    case HMC5883L_SAMPLES_4:
        PrintString("4\n");
        break;
    case HMC5883L_SAMPLES_8:
        PrintString("8\n");
        break;
    default:
        PrintString("Bad number of samples!\n");
    }
}

static void loop()
{
    Vector raw = compass.readRaw();
    Vector norm = compass.readNormalize();

    char buf[100];
    sprintf(buf, "Xnorm = %f Ynorm = %f ZNorm = %f\n", norm.XAxis, norm.YAxis, norm.ZAxis);
    PrintString(buf);

    LL_mDelay(500);
}