/*
 * FILE NAME   : ControlSys.c
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/14/2020
 * DESCRIPTION :
 */

#include "ControlSys.h"

// Function pre definitions for helper functions
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

void ControlSystemInit(int numShocks, arm_pid_instance_f32 *pidControllers,struct ShockDamperProfile startingCoefs) {

    // Set up all the PID controllers
    for(int i = 0; i < numShocks; i++) {
        pidControllers[i].Kp = startingCoefs.PID_P;
        pidControllers[i].Ki = startingCoefs.PID_I;
        pidControllers[i].Kd = startingCoefs.PID_D;

        arm_pid_init_f32(&pidControllers[i],1);
    }   
}

float32_t calculateDampingValue(struct ShockControlSystem *shockControlSystemUnit) {
    int dataIndex = shockControlSystemUnit->shockData.mostRecentDataIndex;
    float32_t dy = shockControlSystemUnit->shockData.shockVelocities[dataIndex].dy;
    float32_t dx = shockControlSystemUnit->shockData.shockVelocities[dataIndex].dx;

    float32_t fd_ideal = dy * 1.0f;

    float32_t c_ideal = ((shockControlSystemUnit->shockData.lastPidOut + fd_ideal) * dx);

    c_ideal = clamp(c_ideal,CONTROL_SYSTEM_C_MAX,CONTROL_SYSTEM_C_MIN);

    float32_t fd_real = c_ideal * dx;

    float32_t pidError = fd_ideal - fd_real;

    // Save the new PID output for the next cycle
    shockControlSystemUnit->shockData.lastPidOut = 
        arm_pid_f32(&(shockControlSystemUnit->shockPidController), pidError);

    return fd_real;
}

void calculateDampingValues(int numShocks,struct ShockControlSystem *shockUnits, 
                            float32_t *controlSystemOutputs) {

    // Copmute the output for each shock PID system
    for(int i = 0; i < numShocks; i++) {
        int dataIndex = shockUnits[i].shockData.mostRecentDataIndex;
        controlSystemOutputs[i] = calculateDampingValue(&shockUnits[i]);
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