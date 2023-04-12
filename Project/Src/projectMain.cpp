#include <stdio.h>
#include <string.h>

#include "main.h"
#include "SWO.h"
#include "projectMain.h"
#include "SensorTestRoutines.h"



void projectMain()
{
    AK8963test();
    MPU6050DMP6test();
    MPU6050Kalmantest();
    MPU9250Kalmantest();
    MPU6050Madgwicktest();
    MPU9250Madgwicktest();
    
    QMC5883Ltest();
    HMC5883Ltest();
	MPU6050test();

    BMP085test();
    ADXL345test ();
    VL53L5CXtest();
    
    while (true)
    {
        PrintString("Hello, world!\n");
        LL_mDelay(1000);
    }
}
