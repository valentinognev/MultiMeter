#include <stdio.h>
#include <string.h>

#include "main.h"
#include "SWO.h"
#include "projectMain.h"
#include "SensorTestRoutines.h"



void projectMain()
{
	HMC5883LTest();
	mpu6050test();
	ADXL345test ();
	BMP085test();
	
	while (true)
	{
        PrintString("Hello, world!\n");
        LL_mDelay(1000);
    }
}
