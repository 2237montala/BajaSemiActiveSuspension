/*
 * FILE NAME   : Delay.c
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/03/2020
 * DESCRIPTION :
 */

#include "Delay.h"


volatile uint32_t sysTicks = 0; 

void delay(uint32_t delayLength) {
    uint32_t startTime = millis();
    while(millis() < startTime + delayLength);
}

void SysTick_Handler(void) {
    sysTicks++;
}