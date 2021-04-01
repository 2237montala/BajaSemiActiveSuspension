#pragma once
#include "config.h"
#include "arm_math.h"
#include "stdlib.h"

#define NUMBER_OF_AXIS 3
#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2

// Sensor Structure
// This needs to be typedef'd for use in the fifo
// typedef struct ShockSensorData{
//     float32_t accels[NUMBER_OF_AXIS];
//     float32_t linearPos;
//     uint8_t inFreefall;
// } ShockSensorDataStruct_t;

typedef struct {
    float32_t accels[NUMBER_OF_AXIS];
    float32_t linearPos;
    uint8_t inFreefall;
} IncomingShockSensorData_t;

// typedef struct  {
//     float32_t dx;
//     float32_t dy;
//     float32_t dLinearPos;
// } ShockVelocitiesStruct_t;

typedef struct {
    float32_t dx;
    float32_t dy;
    float32_t dLinearPos;
    IncomingShockSensorData_t rawData;
} ControlSystemShockData_t;


