
#include "MPU6050.h"
#include "Kalman.h"
#include "math.h"
#include "parameter.h"
#include "SWO.h"

#include "usb_device.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD 0.0174533

MPU6050 mpu;
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
int16_t raw_ax, raw_ay, raw_az;
int16_t raw_gx, raw_gy, raw_gz;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double roll, pitch; // Roll and pitch are calculated using the accelerometer

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY;   // Calculated angle using a Kalman filter

void mpu6050();
void updateMPU6050();
void updatePitchRoll();

void updateMPU6050()
{
    // Read raw data from imu. Units don't care.
    mpu.getMotion6(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
    accX = raw_ax;
    accY = raw_ay;
    accZ = raw_az;
    gyroX = raw_gx;
    gyroY = raw_gy;
    gyroZ = raw_gz;
}

void updatePitchRoll()
{
    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -πto π(radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    roll = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

void mpu6050test()
{
    // Initialize mpu6050
    mpu.initialize();

    char bufstr[100];
    // Get the sample rate
    volatile float rate = mpu.getRate();
    sprintf(bufstr, "getRate()=%f", rate);
    PrintString(bufstr); // SWO_PrintDefaultN(bufstr,strlen(bufstr));

    // Set the sample rate to 8kHz
    if (mpu.getRate() != 0)
        mpu.setRate(0);

    // Get FSYNC configuration value
    volatile uint8_t fsync = mpu.getExternalFrameSync();
    sprintf(bufstr, "getExternalFrameSync()=%d", fsync);
    PrintString(bufstr); // SWO_PrintDefaultN(bufstr,strlen(bufstr));

    // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering
    if (mpu.getExternalFrameSync() != 0)
        mpu.setExternalFrameSync(0);

    // Set Digital Low Pass Filter
    volatile int8_t dlpf = mpu.getDLPFMode();
    sprintf(bufstr, "getDLPFMode()=%d", dlpf);
    PrintString(bufstr); // SWO_PrintDefaultN(bufstr,strlen(bufstr));
    if (mpu.getDLPFMode() != 6)
        mpu.setDLPFMode(6);

    // Get Accelerometer Scale Range
    volatile uint8_t ascale = mpu.getFullScaleAccelRange();
    sprintf(bufstr, "getFullScaleAccelRange()=%d", ascale);
    PrintString(bufstr); // SWO_PrintDefaultN(bufstr,strlen(bufstr));
    // Set Accelerometer Full Scale Range to ±2g
    if (mpu.getFullScaleAccelRange() != 0)
        mpu.setFullScaleAccelRange(0);

    // Get Gyro Scale Range
    volatile uint8_t gscale = mpu.getFullScaleGyroRange();
    sprintf(bufstr, "getFullScaleGyroRange()=%d", gscale);
    PrintString(bufstr); // SWO_PrintDefaultN(bufstr,strlen(bufstr));
    // Set Gyro Full Scale Range to ±250deg/s
    if (mpu.getFullScaleGyroRange() != 0)
        mpu.setFullScaleGyroRange(0);

    /* Set Kalman and gyro starting angle */
    updateMPU6050();
    updatePitchRoll();

    kalmanX.setAngle(roll); // First set roll starting angle
    gyroXangle = roll;
    compAngleX = roll;

    kalmanY.setAngle(pitch); // Then pitch
    gyroYangle = pitch;
    compAngleY = pitch;

    int elasped = 0;
    uint32_t timer = micros();

    bool initialized = false;
    double initial_roll = 0.0;
    double initial_pitch = 0.0;

    while (1)
    {
        updateMPU6050();

        double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
        timer = micros();

        /* Roll and pitch estimation */
        updatePitchRoll();
        double gyroXrate = gyroX / 131.0; // Convert to deg/s
        double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
        // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
        if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
        {
            kalmanX.setAngle(roll);
            compAngleX = roll;
            kalAngleX = roll;
            gyroXangle = roll;
        }
        else
            kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

        if (abs(kalAngleX) > 90)
            gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
        // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
        if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90))
        {
            kalmanY.setAngle(pitch);
            compAngleY = pitch;
            kalAngleY = pitch;
            gyroYangle = pitch;
        }
        else
            kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

        if (abs(kalAngleY) > 90)
            gyroXrate = -gyroXrate;                        // Invert rate, so it fits the restriced accelerometer reading
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

        /* Estimate angles using gyro only */
        gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
        gyroYangle += gyroYrate * dt;
        // gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
        // gyroYangle += kalmanY.getRate() * dt;

        /* Estimate angles using complimentary filter */
        compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
        compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

        // Reset the gyro angle when it has drifted too much
        if (gyroXangle < -180 || gyroXangle > 180)
            gyroXangle = kalAngleX;
        if (gyroYangle < -180 || gyroYangle > 180)
            gyroYangle = kalAngleY;

        /* Print Data every 10 times */
        if (elasped > 10)
        {
            // Set the first data
            if (!initialized)
            {
                initial_roll = roll;
                initial_pitch = pitch;
                initialized = true;
            }

#if 0
			printf("roll:%f", roll); printf(" ");
			printf("initial_roll:%f", initial_roll); printf(" ");
			printf("roll-initial_roll:%f", roll-initial_roll); printf(" ");
			printf("gyroXangle:%f", gyroXangle); printf(" ");
			printf("compAngleX:%f", compAngleX); printf(" ");
			printf("kalAngleX:%f", kalAngleX); printf(" ");
			printf("\n");

			printf("pitch: %f", pitch); printf(" ");
			printf("initial_pitch: %f", initial_pitch); printf(" ");
			printf("pitch-initial_pitch: %f", pitch-initial_pitch); printf(" ");
			printf("gyroYangle:%f", gyroYangle); printf(" ");
			printf("compAngleY:%f", compAngleY); printf(" ");
			printf("kalAngleY:%f", kalAngleY); printf("\t");
			printf("\n");
#endif

            // Send UDP packet
            float _roll = roll - initial_roll;
            float _pitch = pitch - initial_pitch;
            sprintf(bufstr, "roll:%f pitch=%f \n", _roll, _pitch);
            // PrintString((uint8_t*)bufstr, strlen(bufstr)); //
            PrintString(bufstr);

            POSE_t pose;
            pose.roll = _roll;
            pose.pitch = _pitch;
            pose.yaw = 0.0;

            LL_mDelay(10);
            elasped = 0;
        }

        elasped++;
        LL_mDelay(10);
    } // end while
}
