#include "config.h"
#include "Fifo.h"

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