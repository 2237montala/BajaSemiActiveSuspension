#pragma once
/*
 * HEADER NAME : ControlSys.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/14/2020
 * DESCRIPTION :
 */


#include "ShockData.h"

struct ShockControlSystem {
    struct SingleShockData shockData;
    arm_pid_instance_f32 shockPidController;
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
void ControlSystemInit(int numShocks, arm_pid_instance_f32 *pidControllers,
                       struct ShockDamperProfile startingCoefs);


/*
 * PURPOSE
 *      Calculates the output of the control system for a single shock
 * PARAMETERS
 *      shockControlSystemUnit - a pointer to a ShockControlSystem struct containing all the 
 *                               data needed to run the control system
 * RETURNS
 *      A float32_t value that represents the force the damper needs to apply
 */
float32_t calculateDampingValue(struct ShockControlSystem *shockControlSystemUnit);

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


