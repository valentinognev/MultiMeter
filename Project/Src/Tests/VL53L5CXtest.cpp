/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

*/
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include "SWO.h"
#include "stdio.h"

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0; // Used to pretty print output
int imageWidth = 0;      // Used to pretty print output

static void loop();

void VL53L5CXtest()
{
    PrintString("SparkFun VL53L5CX Imager Example\n");

    PrintString("Initializing sensor board. This can take up to 10s. Please wait.\n");
    if (myImager.begin() == false)
    {
        PrintString("Sensor not found - check your wiring. Freezing\n");
        while (1)
            ;
    }

    myImager.setResolution(8 * 8); // Enable all 64 pads

    imageResolution = myImager.getResolution(); // Query sensor for current resolution - either 4x4 or 8x8
    imageWidth = sqrt(imageResolution);         // Calculate printing width

    myImager.startRanging();
    while (1)
    {
        loop();
    }
}

void loop()
{
    char buf[100];
    // Poll sensor for new data
    if (myImager.isDataReady() == true)
    {
        if (myImager.getRangingData(&measurementData)) // Read distance data into array
        {
            // The ST library returns the data transposed from zone mapping shown in datasheet
            // Pretty-print data with increasing y, decreasing x to reflect reality
            for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth)
            {
                for (int x = imageWidth - 1; x >= 0; x--)
                {
                    sprintf(buf, "\t%d", measurementData.distance_mm[x + y]);
                    PrintString(buf);
                }
                PrintString("\n");
            }
            PrintString("\n");
        }
    }

    LL_mDelay(5); // Small delay between polling
}
