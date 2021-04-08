#pragma once
/*
 * HEADER NAME : ControlSys.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/14/2020
 * DESCRIPTION :
 */


#include "ShockData.h"
#include "ShockDamperProfile.h"
#include "PID.h"

struct ShockControlSystem {
    //Create field to hold all the shock pid systems
    // struct pid_instance_f32_struct shockPidControllers[NUM_SHOCKS];
    // float32_t previousPidOutputs[NUM_SHOCKS];
    // float32_t previousDamperValues[NUM_SHOCKS];
    struct pid_instance_f32_struct shockPidController;
    float32_t previousPidOutput;
    float32_t previousDamperValue;
};

/*
 * PURPOSE
 *      Initalizes the PID controllers for each the shocks in the system
 *      using the inputed PID coefficients
 * PARAMETERS
 *      numShocks - the number of shocks in the system
 *      pidControllers - pointer to the array of PID controllers to initalize
 *      startingCoefs - stucture containing the p,i,d coefficients for the PID controller
 * RETURNS
 *      None
 */
void ControlSystemInit(struct ShockControlSystem *shockControlSystem, int numShocks,
                       struct ShockDamperProfile startingCoefs);

/*
 * PURPOSE
 *      Calculates the output of all the control systmes in the system
 * PARAMETERS
 *      *shockControlSystem - a pointer to the control system structure 
 *      *controlSystemShockData - a pointer to the structure hold the shock data
 *      dtMs - the time difference between this run and the last run in ms
 * RETURNS
 *      None
 */
void ControlSystemComputeDampingValue(struct ShockControlSystem *shockControlSystem, 
                               ControlSystemShockData_t *controlSystemShockData, uint32_t dtMs);

