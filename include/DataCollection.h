#pragma once
#include "config.h"
#include "ShockData.h"
#include "fifofast.h"
#include "CANopen.h"

#define SHOCK_SENSOR_DATA_FIFO_NAME shockSensorDataFifo
#define SHOCK_VELOCITY_FIFO_NAME shockVelocityFifo

// Sensor Data Fifos -------------------------------------------------------------------------
// These need to be in here as they are referenced using the ID passed into the structure macro
_fff_declare_a(ShockSensorDataStruct_t,SHOCK_SENSOR_DATA_FIFO_NAME,SHOCK_DATA_BUFFER_LEN,NUM_SHOCKS);
_fff_declare_a(ShockVelocitiesStruct_t,SHOCK_VELOCITY_FIFO_NAME,SHOCK_DATA_BUFFER_LEN,NUM_SHOCKS);

// Initalize the data fifos
_fff_init_a(SHOCK_SENSOR_DATA_FIFO_NAME,NUM_SHOCKS);
_fff_init_a(SHOCK_VELOCITY_FIFO_NAME,NUM_SHOCKS);

// Struct to hold pointer and data length for variables inside the OD
struct VariableToOdMappingStruct {
  uint32_t odIndex;
  void *odDataPtr;
  uint32_t dataLengthInBytes;
};

bool DataCollectionInit(CO_t *CO, uint32_t shockSensorAccelOdIndex, uint32_t shockSensorStatusOdIndex,
                        uint32_t shockSensorRpwOdIndex, uint32_t shockSensorIdOdIndex);

bool PushNewDataOntoFifo();

bool CopyShockDataFromOD(uint8_t *senderCanId, ShockSensorDataStruct_t *shockDataStruct);

/*
 * PURPOSE
 *    Get the pointer to the data stored in the Object Directory. CAN Open stores recent
 *    messages in the Object Directory RAM struct. The pointer to the data is known at 
 *    compile time but this function makes it sort of dymanic
 * PARAMETERS
 *    odIndex - The index of the item in the Object Directory
 *    **dataPtr - A pointer to the pointer where the OD data pointer will be stored
 *    *dataLenInBytes - The length of the array in bytes
 * RETURNS
 *    true if no error occur, false otherwise
 */
//static bool CopyArrayDataFromOD(CO_t *CO, uint32_t odIndex, void **dataPtr, uint32_t *dataLenInBytes);

/*
 * PURPOSE
 *    Get the pointer to the data stored in the Object Directory. CAN Open stores recent
 *    messages in the Object Directory RAM struct. The pointer to the data is known at 
 *    compile time but this function makes it sort of dymanic
 * PARAMETERS
 *    odIndex - the index of the data entry from the Object Directory
 *    **varPtr - a double pointer where the OD data address will be stored
 *    *dataLenInBytes - a pointer to a uint32 that will hold the variable's length in bytes
 * RETURNS
 *  true if no error, false otherwise
 */
//static bool CopyVarFromOD(CO_t *CO, uint32_t odIndex, void **varPtr, uint32_t *varByteSize);