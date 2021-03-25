#pragma once
#include "config.h"
#include "arm_math.h"
#include "stdlib.h"
#include "fifofast.h"

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
