#pragma once
/*
 * HEADER NAME : config.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/14/2020
 * DESCRIPTION :
 */

// How many milliseconds between shock data samples
#define SHOCK_DATA_COLLECTION_RATE 5

// How many milliseconds of data should be kept in order
// to determine what the car is doing
#define SHOCK_DATA_BUFFER_TIME_LEN 1000

// How many samples will occupy our data buffer
#define SHOCK_DATA_BUFFER_LEN (SHOCK_DATA_BUFFER_TIME_LEN/SHOCK_DATA_COLLECTION_RATE)

// Control system parameters
// PID saturation values
#define CONTROL_SYSTEM_ZETA_MIN 0.75f
#define CONTROL_SYSTEM_ZETA_MAX 4.0f

#define CONTROL_SYSTEM_C_MIN 1342.0f
#define CONTROL_SYSTEM_C_MAX 7155.0f

// PID parameters
#define PID_P_NORMAL 0.7f
#define PID_I_NORMAL 5.0f
#define PID_D_NORMAL -0.6f

// Defines how many different profiles are going to be available
#define NUM_SHOCK_PROFILES 1


#define NUM_SHOCKS 1

// CANOpen defines ----------------------------------------------------------------------
#define CAN_BAUD_RATE 500U
// Main controller ID
#define MAIN_CONTROLLER_ID 0x05

// Shock controller CAN ids
// These are set by the dip switches on the controller itself and must be 
// listed here
#define SHOCK_CONTROLLER_ONE_ID 0x20