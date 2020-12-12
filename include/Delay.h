#pragma once
#include "targetCommon.h"

/*
 * HEADER NAME : Delay.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/03/2020
 * DESCRIPTION:
 */


volatile extern uint32_t sysTicks;

/*
 * PURPOSE
 *      Returns the local sysTick value to any one calls this
 * PARAMETERS
 *      None
 * RETURNS
 *      sysTick - Current millisecond count of the system
 */
inline uint32_t millis(void) {
    return sysTicks;
}

/*
 * PURPOSE
 *      Delays the program by any number of milliseconds
 * PARAMETERS
 *      delayLength: The length of the delay in milliseconds
 * RETURNS
 *      None
 */
void delay(uint32_t delayLength);
