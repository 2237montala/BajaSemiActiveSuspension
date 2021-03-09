/*
 * FILE NAME   : ControlSys.c
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/14/2020
 * DESCRIPTION :
 */

#include "ControlSys.h"

// Private parameters for the control system
float32_t cc,c_high,c_low;

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
        shockControlSystem->shockPidControllers[i].Kp = startingCoefs.PID_P;
        shockControlSystem->shockPidControllers[i].Ki = startingCoefs.PID_I;
        shockControlSystem->shockPidControllers[i].Kd = startingCoefs.PID_D;

        arm_pid_init_f32(&(shockControlSystem->shockPidControllers[i]),1);
    }
}

static void calculateControlSystemParameters() {
    // Calculate cc from spring constant and mass of quarter car
    cc = sqrtf(CONTROL_SPRING_K * CONTROL_SYSTEM_MASS_QUARTER_CAR);

    c_high = CONTROL_SYSTEM_ZETA_MAX * cc;
    c_low = CONTROL_SYSTEM_ZETA_MIN * cc;
}

float32_t calculateDampingValue(struct ShockControlSystem *shockControlSystemUnit, 
                                uint32_t shockIndex, float32_t dx, float32_t dy) {
    // Change is shock vertical acceleration (dy)
    float32_t shockVertVel = dy;

    // The rate of change of shock length (dx)
    float32_t shockLenVel = dx;
    float32_t lastPidOutput = shockControlSystemUnit->previousPidOutputs[shockIndex];

    float32_t fd_ideal = shockVertVel * 1.0f;
    float32_t fd_ideal_star = fd_ideal + lastPidOutput;

    float32_t c_ideal = -1.0f * (fd_ideal_star / shockLenVel);

    // If c_idea is higher than c max then clamp it, same for c min, otherwise keep same value
    float32_t c_real = clamp(c_ideal,c_high,c_low);

    // Calculate the real damper force 
    float32_t fd_real = c_real * shockLenVel;

    // Calculate the error between the real and the idea damping force
    float32_t pidError = fd_ideal - fd_real;

    // Save the new PID output for the next cycle
    shockControlSystemUnit->previousPidOutputs[shockIndex] = 
        arm_pid_f32(&(shockControlSystemUnit->shockPidControllers[shockIndex]), pidError);

    return fd_real;
}

// void calculateDampingValues(int numShocks,struct ShockControlSystem *shockUnits, 
//                             float32_t *controlSystemOutputs) {
//     // Copmute the output for each shock PID system
//     for(int i = 0; i < numShocks; i++) {
//         //int dataIndex = shockUnits[i].shockData.mostRecentDataIndex;
//         controlSystemOutputs[i] = calculateDampingValue(&shockUnits[i]);
//     }
// }


//------------ Helper Function ---------------------------------
static float clamp(float32_t value, float32_t max, float32_t min) {
    if(value > max) {
        return max;
    }else if(value < min) {
        return min;
    }
    return value;
}