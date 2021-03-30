#include "DataCollection.h"
#include "stdbool.h"


bool PushNewDataOntoFifo(CO_t *CO, uint32_t shockSensorAccelOdIndex, uint32_t shockSensorStatusOdIndex,
                         uint32_t shockSensorRpwOdIndex) {

    static ShockSensorDataStruct_t tempData;
    // Get the newest data from the OD
    bool status = CopyShockDataFromOD(CO,&tempData,shockSensorAccelOdIndex,shockSensorAccelOdIndex,
                                      shockSensorRpwOdIndex);

    if(!status) {
        // Error copying data
        return false;
    }

    // Push data onto the fifo
    // If fifo is full then pop off an element
    if(_fff_is_full(SHOCK_SENSOR_DATA_FIFO_NAME[0])) {
        _fff_remove(SHOCK_SENSOR_DATA_FIFO_NAME[0],1);
    }

    // Add new sample to the fifo
    _fff_write_lite(SHOCK_SENSOR_DATA_FIFO_NAME[0],tempData);

    return true;
}

bool CopyShockDataFromOD(CO_t *CO, ShockSensorDataStruct_t *shockDataStruct, uint32_t shockSensorAccelOdIndex,
                         uint32_t shockSensorStatusOdIndex, uint32_t shockSensorRpwOdIndex) {
  // Testing to make sure it gets the new values
  // CO_OD_RAM.readShockAccel[0] = 1;
  // CO_OD_RAM.readShockAccel[1] = 2;
  // CO_OD_RAM.readShockAccel[2] = 3;

  // CO_OD_RAM.readShockAccelStatus = 0x1;

  uint32_t dataLenInBytes = 0;
  void* accelArrayVoidPtr = NULL;

  if(!CopyArrayDataFromOD(CO,shockSensorAccelOdIndex, &accelArrayVoidPtr, &dataLenInBytes)) {
    // Problem getting accel data array information
    return false;
  }

  if(accelArrayVoidPtr == NULL) {
    // Something went wrong with getting the data pointer, shouldn't be null
    return false;
  }

  // Copy OD accel data to the local struct
  // OD_getLength returns the length of 
  memcpy(shockDataStruct->accels,accelArrayVoidPtr, dataLenInBytes);

  // Create pointer to hold the location of the OD variable
  void *accelStatusVoidPtr = NULL;

  // Get the pointer to the data and its length
  // Should be a single byte  since the accel status is a byte
  if(!CopyVarFromOD(CO, shockSensorStatusOdIndex, &accelStatusVoidPtr, &dataLenInBytes)) {
    return false;
  }

  if(accelStatusVoidPtr == NULL) {
    return false;
  }

  // Copy the data over to the array
  memcpy(&(shockDataStruct->inFreefall),accelStatusVoidPtr,dataLenInBytes);


  // TODO: Add roll pitch and yaw

  return true;
}

static bool CopyArrayDataFromOD(CO_t *CO, uint32_t odIndex, void **dataPtr, uint32_t *dataLenInBytes) {
  uint32_t dataIndex = CO_OD_find(CO->SDO[0],odIndex);

  if(dataIndex == 0xFFFF) {
    // OD index not found
    return false;
  }

  // Get the number of items in the OD index
  // Value is returned as a pointer to the variable so cast is as that variable type
  uint8_t *numElementsStored = (uint8_t *) CO_OD_getDataPointer(CO->SDO[0],dataIndex,0);

  // Get the pointer to the data where CAN Open stores the most recent shock accel data 
  // This is subIndex > 1
  *dataPtr = CO_OD_getDataPointer(CO->SDO[0],dataIndex,1);

  // Array element length 
  if(*numElementsStored == 0) {
    *dataLenInBytes = 0;
  } else {
    uint32_t dateElementLen = CO_OD_getLength(CO->SDO[0],dataIndex,1);
    *dataLenInBytes = *numElementsStored * dateElementLen;
  }
  return true;
}


static bool CopyVarFromOD(CO_t *CO, uint32_t odIndex, void **varPtr, uint32_t *varByteSize) {
  uint32_t dataIndex = CO_OD_find(CO->SDO[0],odIndex);

  if(dataIndex == 0xFFFF) {
    // OD index not found
    return false;
  }

  *varPtr = CO_OD_getDataPointer(CO->SDO[0],dataIndex,0);

  *varByteSize = CO_OD_getLength(CO->SDO[0],dataIndex,1);

  return true;
}