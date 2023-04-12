#include <stdio.h>
#include <string.h>

#include "main.h"
#include "SWO.h"
#include "projectMain.h"
#include "SensorTestRoutines.h"



void projectMain()
{
	QMC5883Ltest();
	HMC5883Ltest();
	mpu6050test();

    BMP085test();
    ADXL345test ();
    VL53L5CXtest();
    
    while (true)
    {
        PrintString("Hello, world!\n");
        LL_mDelay(1000);
    }
}
