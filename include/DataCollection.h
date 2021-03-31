#pragma once

#include "config.h"
#include "ShockData.h"
#include "fifofast.h"

#ifndef CANopen_H
    #include "CANopen.h"
#endif

#define SHOCK_SENSOR_DATA_FIFO_NAME shockSensorDataFifo
#define SHOCK_VELOCITY_FIFO_NAME shockVelocityFifo

// Sensor Data Fifos -------------------------------------------------------------------------
// These need to be in here as they are referenced using the ID passed into the structure macro
_fff_declare_a(ShockSensorDataStruct_t,SHOCK_SENSOR_DATA_FIFO_NAME,SHOCK_DATA_BUFFER_LEN,NUM_SHOCKS);
_fff_declare_a(ShockVelocitiesStruct_t,SHOCK_VELOCITY_FIFO_NAME,SHOCK_DATA_BUFFER_LEN,NUM_SHOCKS);



// Struct to hold pointer and data length for variables inside the OD
struct VariableToOdMappingStruct {
  uint32_t odIndex;
  void *odDataPtr;
  uint32_t dataLengthInBytes;
};

struct ShockSensorDataOdStruct {
    ShockSensorDataStruct_t sensorData;
    uint8_t senderCanId;
};

/*
 * PURPOSE
 *      Initalizes the data collection variables. It links local variables to the Object Directory
 *      variables so when we need to copy the OD data we already have the pointers
 * PARAMETERS
 *      CO* - pointer to the CO object 
 *      shockSensorAccelOdIndex - The OD index for the shock acceleration data
 *      shockSensorStatusOdIndex - The OD index for the shock status variable
 *      shockSensorRpwOdIndex - The OD index for the roll pitch and yaw data
 *      shockSensorIdOdIndex - The OD index for the sender CAN ID variable
 * RETURNS
 *      true if no error, false otherwise
 */
bool DataCollectionInit(CO_t *CO, uint32_t shockSensorAccelOdIndex, uint32_t shockSensorStatusOdIndex,
                        uint32_t shockSensorRpwOdIndex, uint32_t shockSensorIdOdIndex);

/*
 * PURPOSE
 *      Copies data from the OD and pushes it to the sensor data fifo
 * PARAMETERS
 *      None
 * RETURNS
 *      true if no error, false otherwise
 */
bool PushNewDataOntoFifo(void);

/*
 * PURPOSE
 *      Copies the data from the OD and stores it into a passed in sensor data struct
 * PARAMETERS
 *      *senderCanId - a pointer to a uint8_t variable that will hold the CAN id for who sent the data
 *      *shockDataStruct - a pointer to a sensorDataStruct where the OD data will be stored
 * RETURNS
 *      true if no errors, false otherwise
 */
bool CopyShockDataFromOD(uint8_t *senderCanId, ShockSensorDataStruct_t *shockDataStruct);



bool DoesOdContainNewData(void);
