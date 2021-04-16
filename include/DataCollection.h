#pragma once

#include "config.h"
#include "ShockData.h"
#include "fifofast.h"

#ifndef CANopen_H
    #include "CANopen.h"
#endif


#define IMCOMING_SHOCK_SENSOR_DATA_FIFO_NAME incomingShockDataFifo

// This fifo will hold incoming shock data to be used for the control system and calculating 
// velocities from accelerations. This fifo will be used as a queue
_fff_declare_a(IncomingShockSensorData_t,IMCOMING_SHOCK_SENSOR_DATA_FIFO_NAME,
               INCOMING_SHOCK_DATA_FIFO_LEN,NUM_SHOCKS);


// Struct to hold pointer and data length for variables inside the OD
struct VariableToOdMappingStruct {
  uint32_t odIndex;
  void *odDataPtr;
  uint32_t dataLengthInBytes;
};

struct ShockSensorDataOdStruct {
    IncomingShockSensorData_t sensorData;
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
bool DataCollectionInit(CO_t *CO, uint32_t shockSensorAccelOdIndex,
                        uint32_t shockSensorStatusOdIndex, uint32_t shockSensorRpwOdIndex,
                        uint32_t shockSensorIdOdIndex, uint32_t shockSensorPositionIndex);

/*
 * PURPOSE
 *      Copies data from the OD and pushes it to the sensor data fifo
 * PARAMETERS
 *      None
 * RETURNS
 *      fifoIndex - If there is a fifo for that sender and it copied data correctly the 
 *                  function returns the fifo index. Otherwise a -1 for error
 */
int PushNewDataOntoFifo(void);

/*
 * PURPOSE
 *      Copies the data from the OD and stores it into a passed in sensor data struct
 * PARAMETERS
 *      *shockOdData - a pointer to a shockSensorDataOdStruct that will hold the data copied
 *                     from the OD
 * RETURNS
 *      true if no errors, false otherwise
 */
bool CopyShockDataFromOD(struct ShockSensorDataOdStruct *shockOdData);


/*
 * PURPOSE
 *      Determines if there is new data contained in the OD compared to the last known data added
 *      to the fifo. Since there is only one location for sensor data we need to constantly check
 *      if the data is new so we can copy it out
 * PARAMETERS
 *      None
 * RETURNS
 *      True is data in OD is new, false if it is old data
 */
bool DoesOdContainNewData(void);


#ifdef SOFTWARE_TEST
bool DataCollectionLoadNewTestValues(struct ShockSensorDataOdStruct *dataToLoad);
#endif