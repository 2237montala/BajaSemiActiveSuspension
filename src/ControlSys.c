/*
 * FILE NAME   : ControlSys.c
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/14/2020
 * DESCRIPTION :
 */

#include "ControlSys.h"
#include "stdbool.h"

// Private parameters for the control system
float32_t cc,c_high,c_low;
bool firstRun;

// Function pre definitions for helper functions


static void calculateControlSystemParameters();

/*
 * PURPOSE
 *      Forces the input value to be between the max and min values
 * PARAMETERS
 *      value - the value to clamp
 *      max   - the max the value can be
 *      min   - the min the value can be
 * RETURNS
 *      A float32_t that is the clamped value
 */
static float32_t clamp(float32_t value, float32_t max, float32_t min);

void ControlSystemInit(struct ShockControlSystem *shockControlSystem, int numShocks,
                       struct ShockDamperProfile startingCoefs) {
    calculateControlSystemParameters();

    // Set up all the PID controllers
    for(int i = 0; i < numShocks; i++) {
        PidInstanceInitF32(&(shockControlSystem->shockPidControllers[i]));

        PidInstanceSetParamsF32(&(shockControlSystem->shockPidControllers[i]), 
                                startingCoefs.PID_P,
                                startingCoefs.PID_I,
                                startingCoefs.PID_D);
    }

    firstRun = true;
}

static void calculateControlSystemParameters() {
    // Calculate cc from spring constant and mass of quarter car
    cc = sqrtf(CONTROL_SPRING_K * CONTROL_SYSTEM_MASS_QUARTER_CAR);

    c_high = CONTROL_SYSTEM_ZETA_MAX * cc;
    c_low = CONTROL_SYSTEM_ZETA_MIN * cc;
}

void calculateDampingValue(struct ShockControlSystem *shockControlSystemUnit, 
                                uint32_t shockIndex, float32_t dx, float32_t dy, float32_t dt) {
    // Change is shock vertical acceleration (dy)
    float32_t shockVertVel = dy;

    // The rate of change of shock length (dx)
    float32_t shockLenVel = dx;
    float32_t lastPidOutput = shockControlSystemUnit->previousPidOutputs[shockIndex];

    float32_t fd_ideal = -1.0f * shockVertVel * cc;
    float32_t fd_ideal_star = fd_ideal + lastPidOutput;

    float32_t c_ideal = -1.0f * (fd_ideal_star / shockLenVel);

    // If c_idea is higher than c max then clamp it, same for c min, otherwise keep same value
    float32_t c_real = clamp(c_ideal,c_high,c_low);

    // Calculate the real damper force 
    float32_t fd_real = -1.0f * c_real * shockLenVel;

    // Calculate the error between the real and the idea damping force
    float32_t pidError = fd_ideal - fd_real;

    // If this is the first computation then we need to adjust the error calulation
    // Since we are taking the error of the past computation we need to force the
    // D part of the PID to be zero. This will be done by making the old error equal
    // to the current fd_real
    if(firstRun) {
        float32_t tempPreData[PID_PREV_COUNT] = {0};
        tempPreData[PID_PREV_ERROR] = pidError;

        // Set PID to reference a fake pidError for the first run
        PidInstanceSetInitPrevData(&(shockControlSystemUnit->shockPidControllers[shockIndex]),tempPreData);

        firstRun = false;
    } else {
        
        
    }

    // Save the new PID output for the next cycle
    float32_t pid = lastPidOutput + 
                    PidComputeF32(&(shockControlSystemUnit->shockPidControllers[shockIndex]), pidError, dt)
                     * dt;

    shockControlSystemUnit->previousPidOutputs[shockIndex] = pid;

    shockControlSystemUnit->previousDamperValues[shockIndex] = fd_real;
}

void calculateAllDampingValues(struct ShockControlSystem *shockControlSystems, 
                               ControlSystemShockData_t *controlSystemData) {
  for(int i = 0; i < NUM_SHOCKS; i++) {
    float32_t tempDy, tempDLinearPos;
    tempDLinearPos = controlSystemData->dLinearPos;
    tempDy = controlSystemData->dy;
    calculateDampingValue(shockControlSystems, i, tempDLinearPos, 
                          tempDy, SHOCK_DATA_COLLECTION_RATE_SEC);

    // Take new damper value and convert to valve position
    // TODO: write this function
  }
}

//------------ Helper Function ---------------------------------
static float clamp(float32_t value, float32_t max, float32_t min) {
    if(value > max) {
        return max;
    }else if(value < min) {
        return min;
    }
    return value;
}