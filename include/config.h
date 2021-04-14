#pragma once
/*
 * HEADER NAME : config.h
 * CREATOR     : Anthony Montalbano
 * CREATE DATE : 12/14/2020
 * DESCRIPTION :
 */

// How many milliseconds between shock data samples
#define SHOCK_DATA_COLLECTION_RATE 5
#define SHOCK_DATA_COLLECTION_RATE_SEC (SHOCK_DATA_COLLECTION_RATE/1000.0f)

// How many milliseconds of data should be kept in order
// to determine what the car is doing
#define SHOCK_DATA_BUFFER_TIME_LEN 1000

// How many samples will occupy our data buffer
#define SHOCK_DATA_BUFFER_LEN (SHOCK_DATA_BUFFER_TIME_LEN/SHOCK_DATA_COLLECTION_RATE)

// Control system parameters ------------------------------------------------------------
// PID saturation values
#define CONTROL_SYSTEM_ZETA_MIN 0.75f
#define CONTROL_SYSTEM_ZETA_MAX 4.0f

#define CONTROL_SPRING_K 40000.0f
#define CONTROL_SYSTEM_MASS_QUARTER_CAR 80.0f

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
// listed here. Set these values to 0 if not used
#define SHOCK_CONTROLLER_ONE_ID 0x20
#define SHOCK_CONTROLLER_TWO_ID 0x0
#define SHOCK_CONTROLLER_THREE_ID 0x0
#define SHOCK_CONTROLLER_FOUR_ID 0x0

#define SHOCK_CONTROLLER_HEARTBEAT_INTERVAL 1000U
#define SHOCK_CONTROLLER_HEARTBEAT_INTERVAL_OFFSET 200U

// Velocity integration paramters ---------------------------------
// How long the fifo is for the income shock data 
// This value should be a power of 2
// INCOMING_SHOCK_DATA_FIFO_LEN * SHOCK_DATA_COLLECTION_RATE will be a length of time of valid data
#define INCOMING_SHOCK_DATA_FIFO_LEN 16

// How many previous acceleration values will be used to calcuate the new velocity in the same axis
//#define NUM_PREVIOUS_ACCELS_PER_INTEGRATION 1

#define DY_VELOCITY_AXIS Y_INDEX
#define DX_VELOCTIY_AXIS X_INDEX

#define SOFTWARE_TEST