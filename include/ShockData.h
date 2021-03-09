#pragma once
#include "config.h"
#include "arm_math.h"
#include "stdlib.h"
#include "fifofast.h"

#define SHOCK_SENSOR_DATA_FIFO_NAME shockSensorDataFifo
#define SHOCK_VELOCITY_FIFO_NAME shockVelocityFifo
// Sensor Structure
typedef struct {
    float32_t accelX;
    float32_t accelY;
    float32_t accelZ;
    float32_t linearPos;
} ShockSensorDataStruct_t;

typedef struct  {
    float32_t dx;
    float32_t dy;
    float32_t dLinearPos;
} ShockVelocitiesStruct_t;

struct CarShockData {
    _fff_declare_a(ShockSensorDataStruct_t,SHOCK_SENSOR_DATA_FIFO_NAME,SHOCK_DATA_BUFFER_LEN,NUM_SHOCKS);
    _fff_declare_a(ShockVelocitiesStruct_t,SHOCK_VELOCITY_FIFO_NAME,SHOCK_DATA_BUFFER_LEN,NUM_SHOCKS);
};

