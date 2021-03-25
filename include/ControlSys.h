#pragma once
/*
 * HEADER NAME : ControlSys.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/14/2020
 * DESCRIPTION :
 */


#include "ShockData.h"
#include "ShockDamperProfile.h"

struct ShockControlSystem {
    //Create field to hold all the shock pid systems
    arm_pid_instance_f32 shockPidControllers[NUM_SHOCKS];
    float32_t previousPidOutputs[NUM_SHOCKS];
    float32_t previousDamperValues[NUM_SHOCKS];
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
 *      Calculates the output of the control system for a single shock
 * PARAMETERS
 *      shockControlSystemUnit - a pointer to a ShockControlSystem struct containing all the 
 *                               data needed to run the control system
 *      shockIndex             - the index for which shock we are looking at
 *      dx                     - the x velocity
 *      dy                     - the y velocity
 * RETURNS
 *      
 */
void calculateDampingValue(struct ShockControlSystem *shockControlSystemUnit, 
                                uint32_t shockIndex, float32_t dx, float32_t dy, float32_t dt);

/*
 * PURPOSE
 *      Calculates the output of all the control systmes in the system
 * PARAMETERS
 *      numShocks - the number of shocks in the system
 *      shockUnits - a pointer to an array of ShockControlSystem sturtures
 *      controlSystemOutputs - a pointer to an array where the output of each control 
 *                             system will go
 * RETURNS
 *      None
 */
void calculateDampingValues(int numShocks,struct ShockControlSystem *shockUnits, 
                            float32_t *controlSystemOutputs);


