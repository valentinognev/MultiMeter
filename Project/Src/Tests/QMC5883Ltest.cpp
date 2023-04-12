
#include <QMC5883L.h>
#include <SWO.h>
#include <stdio.h>
#include <SensorTestRoutines.h>

QMC5883L compass;
static void loop(void);

void QMC5883Ltest()
{
    compass.init();
    compass.setSamplingRate(50);


    PrintString("QMC5883L Compass Demo\n");
    PrintString("Turn compass in all directions to calibrate....\n");

    while(1)
    {
        loop();
    }
}

void loop()
{
    char buffer[80];
    int heading = compass.readHeading();
    if (heading == 0)
    {
        /* Still calibrating, so measure but don't print */
    }
    else
    {
        sprintf(buffer, "Heading: %d\n", heading);
        PrintString(buffer);
    }
    LL_mDelay(100);
}