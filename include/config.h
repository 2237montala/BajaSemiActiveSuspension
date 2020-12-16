/*
 * HEADER NAME : config.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/14/2020
 * DESCRIPTION :
 */
#include "arm_math.h"

#define NUM_SHOCKS 4

// How many milliseconds between shock data samples
#define SHOCK_DATA_COLLECTION_RATE 5

// How many milliseconds of data should be kept in order
// to determine what the car is doing
#define SHOCK_DATA_BUFFER_TIME_LEN 1000

// How many samples will occupy our data buffer
#define SHOCK_DATA_BUFFER_LEN (SHOCK_DATA_BUFFER_TIME_LEN/SHOCK_DATA_COLLECTION_RATE)

// Control system parameters
// PID saturation values
#define CONTROL_SYSTEM_ZETA_MIN 0.75f
#define CONTROL_SYSTEM_ZETA_MAX 4.0f

#define CONTROL_SYSTEM_C_MIN 1342.0f
#define CONTROL_SYSTEM_C_MAX 7155.0f

// PID parameters
#define PID_P_NORMAL 0.7f
#define PID_I_NORMAL 5.0f
#define PID_D_NORMAL -0.6f


struct ShockDamperProfile
{
    float PID_P;
    float PID_I;
    float PID_D;
};

#define NUM_SHOCK_PROFILES 1
const struct ShockDamperProfile shockDamperProfiles[NUM_SHOCK_PROFILES] = {
    {
        // Define the normal shock damping values
        .PID_P = PID_P_NORMAL,
        .PID_I = PID_I_NORMAL,
        .PID_D = PID_D_NORMAL
    }
};

// Sensor Structure
struct ShockSensorData {
    float accelX;
    float accelY;
    float accelZ;
    float linearPos;
};

struct ShockVelocities {
    float dx;
    float dy;
};

struct SingleShockData {
    int mostRecentDataIndex;
    struct ShockSensorData dataBuffer[SHOCK_DATA_BUFFER_LEN];
    struct ShockVelocities velocities[SHOCK_DATA_BUFFER_LEN];
    float32_t lastPidOut;
};
