#include "DataCollection.h"
#include "stdbool.h"

// Global stucts to hold mapping from variables to OD data
struct VariableToOdMappingStruct accelDataMapping;
struct VariableToOdMappingStruct shockPosDataMapping;
struct VariableToOdMappingStruct rpyDataMapping;
struct VariableToOdMappingStruct statusMapping;
struct VariableToOdMappingStruct idMapping;

uint32_t shockControllerOneIndex, shockControllerTwoIndex, shockControllerThreeIndex, shockControllerFourIndex;

// Variable holding the last read data from the OD 
struct ShockSensorDataOdStruct lastOdSensorData;

// Private functions ----------------------------------------------------------------------------
static int GetFifoIndexFromId(uint8_t id);

// Start of functions ---------------------------------------------------------------------------

bool DataCollectionInit(CO_t *CO, uint32_t shockSensorAccelOdIndex,
                        uint32_t shockSensorStatusOdIndex, uint32_t shockSensorRpwOdIndex,
                        uint32_t shockSensorIdOdIndex, uint32_t shockSensorPositionIndex) {

    // Initialize some variables
    memset(&lastOdSensorData,0x0,sizeof(lastOdSensorData));

    // Initalize the data fifos
    _fff_init_a(IMCOMING_SHOCK_SENSOR_DATA_FIFO_NAME,NUM_SHOCKS);
    //_fff_init_a(SHOCK_VELOCITY_FIFO_NAME,NUM_SHOCKS);

    // Create index map based on which
    for(int i = 0; i < NUM_SHOCKS; i++) {
        if(SHOCK_CONTROLLER_ONE_ID != 0) {
            shockControllerOneIndex = i;
            // Shock controller one is here
        } else if(SHOCK_CONTROLLER_TWO_ID != 0) {
            shockControllerTwoIndex = i;
            // Shock controller two is here
        } else if(SHOCK_CONTROLLER_THREE_ID != 0) {
            shockControllerThreeIndex = i;
            // Shock controller three is here
        } else if(SHOCK_CONTROLLER_FOUR_ID != 0) {
            shockControllerFourIndex = i;
            // Shock controller four is here
        }
    }

    // Get the index and data pointers for the requested data
    bool noError = true;
    noError = FillOdMapping(CO->SDO[0], &accelDataMapping,shockSensorAccelOdIndex);
    if(!noError) {
      return false;
    }

    noError = FillOdMapping(CO->SDO[0], &statusMapping, shockSensorStatusOdIndex);
    if(!noError) {
      return false;
    }

    noError = FillOdMapping(CO->SDO[0], &rpyDataMapping, shockSensorRpwOdIndex);
    if(!noError) {
      return false;
    }

    noError = FillOdMapping(CO->SDO[0], &idMapping,shockSensorIdOdIndex);
    if(!noError) {
      return false;
    }

    noError = FillOdMapping(CO->SDO[0], &shockPosDataMapping,shockSensorPositionIndex);
    if(!noError) {
      return false;
    }
    return true;
}

int PushNewDataOntoFifo(void) {
    // Get the newest data from the OD
    bool status = CopyShockDataFromOD(&lastOdSensorData);
    if(!status) {
        // Error copying data
        return -1;
    }

    // TODO: Change adding to fifo based on who sent the data
    int fifoIndex = GetFifoIndexFromId(lastOdSensorData.senderCanId);

    if(fifoIndex >= 0) {
        // Push data onto the fifo
        // If fifo is full then pop off an element
        if(_fff_is_full(IMCOMING_SHOCK_SENSOR_DATA_FIFO_NAME[fifoIndex])) {
            _fff_remove(IMCOMING_SHOCK_SENSOR_DATA_FIFO_NAME[fifoIndex],1);
        }

        // Add new sample to the fifo
        _fff_write_lite(IMCOMING_SHOCK_SENSOR_DATA_FIFO_NAME[fifoIndex],lastOdSensorData.sensorData);

        return fifoIndex;
    } else {
        // Index couldn't be found
        // The sender ID wasn't known
        return -1;
    }
}

bool CopyShockDataFromOD(struct ShockSensorDataOdStruct *shockOdData) {
  if(idMapping.odDataPtr == NULL) {
    return false;
  }

  // uint8_t *tempId = (uint8_t *)idMapping.odDataPtr; 

  // shockOdData->senderCanId = *tempId;

  if(accelDataMapping.odDataPtr == NULL) {
    // Something went wrong with getting the data pointer, shouldn't be null
    return false;
  }

  // Copy OD accel data to the local struct
  // OD_getLength returns the length of 
  memcpy(shockOdData->sensorData.accels,accelDataMapping.odDataPtr,
         accelDataMapping.dataLengthInBytes);


  if(statusMapping.odDataPtr == NULL) {
    return false;
  }

  // Copy the data over to the array
  memcpy(&(shockOdData->sensorData.inFreefall),statusMapping.odDataPtr,
         statusMapping.dataLengthInBytes);


  // TODO: Add roll pitch and yaw

  // Copy over shock position data
  if(shockPosDataMapping.odDataPtr == NULL) {
    return false;
  }

  memcpy(&(shockOdData->sensorData.linearPos),shockPosDataMapping.odDataPtr,
         shockPosDataMapping.dataLengthInBytes);

  // Copy over the sender ID
  if(idMapping.odDataPtr == NULL) {
    return false;
  }

  memcpy(&(shockOdData->senderCanId),idMapping.odDataPtr,
         idMapping.dataLengthInBytes);

  return true;
}

bool FillOdMapping(CO_SDO_t *SDO, struct VariableToOdMappingStruct *mappingStruct, uint32_t odIndexToMap) {
  if(SDO == NULL || mappingStruct == NULL || odIndexToMap == 0xFFFF) {
    return false;  
  }

  int32_t dataIndex = CO_OD_find(SDO,odIndexToMap);
  mappingStruct->odIndex = dataIndex;

  // Get the number of items in the OD index
  // Value is returned as a pointer to the variable so cast is as that variable type
  uint8_t *numElementsStored = (uint8_t *) CO_OD_getDataPointer(CO->SDO[0],dataIndex,0);

  // Get the pointer to the data where CAN Open stores the most recent shock accel data 
  // This is subIndex > 1
  mappingStruct->odDataPtr = CO_OD_getDataPointer(CO->SDO[0],dataIndex,1);

  // Array element length 
  if(*numElementsStored == 0) {
    mappingStruct->dataLengthInBytes = CO_OD_getLength(CO->SDO[0],dataIndex,0);
  } else {
    uint32_t dateElementLen = CO_OD_getLength(CO->SDO[0],dataIndex,1);
    mappingStruct->dataLengthInBytes = *numElementsStored * dateElementLen;
  }


  if(mappingStruct->odDataPtr == NULL) {
    // Something went wrong with getting the data pointer, shouldn't be null
    return false;
  }

  return true;
}

static int GetFifoIndexFromId(uint8_t id) {
    switch(id) {
        case SHOCK_CONTROLLER_ONE_ID:
            return shockControllerOneIndex;
        #if SHOCK_CONTROLLER_TWO_ID != 0
        case SHOCK_CONTROLLER_TWO_ID:
            return shockControllerTwoIndex;
        #endif
        #if SHOCK_CONTROLLER_THREE_ID != 0
        case SHOCK_CONTROLLER_THREE_ID:
            return shockControllerThreeIndex;
        #endif
        #if SHOCK_CONTROLLER_FOUR_ID != 0
        case SHOCK_CONTROLLER_FOUR_ID:
            return shockControllerFourIndex;
        #endif
        default:
            return -1;
    }
}

bool DoesOdContainNewData(void) {
    // Compare data in the OD with the data at the head of sender's fifo
    // Get the OD data
    struct ShockSensorDataOdStruct currOdData;

    // Get the newest data from the OD
    bool status = CopyShockDataFromOD(&currOdData);
    if(!status) {
        // Error copying data
        return false;
    }

    // Compare the current OD data with the last copy of the OD values
    // The copy is only updated when different sensor data is saved
    // Zero is the default state for the senderCanId. It is also not possible in this setup
    if(currOdData.senderCanId > 0x0) {
      if(currOdData.senderCanId == lastOdSensorData.senderCanId) {
        // Senders are the same so check the data structs
        // If they are equal, memcmp returns 0, then they are the same data so return false
        return memcmp(&(currOdData.sensorData),&(lastOdSensorData.sensorData),
                      sizeof(currOdData.sensorData)) != 0;
      } else {
          // Data in OD is from a different sender so it's new
          return true;
      }
    }

    return false;
}

#ifdef SOFTWARE_TEST
bool DataCollectionLoadNewTestValues(struct ShockSensorDataOdStruct *dataToLoad) {
  if(accelDataMapping.odDataPtr == NULL) {
    // Something went wrong with getting the data pointer, shouldn't be null
    return false;
  }

  // Copy the data over from array to od
  memcpy(accelDataMapping.odDataPtr,dataToLoad->sensorData.accels,
         accelDataMapping.dataLengthInBytes);


  if(statusMapping.odDataPtr == NULL) {
    return false;
  }

  // Copy the data over from array to od
  memcpy(statusMapping.odDataPtr,&(dataToLoad->sensorData.inFreefall),
         statusMapping.dataLengthInBytes);

  // Copy over shock position data to od
  if(shockPosDataMapping.odDataPtr == NULL) {
    return false;
  }

  memcpy(shockPosDataMapping.odDataPtr,&(dataToLoad->sensorData.linearPos),
         shockPosDataMapping.dataLengthInBytes);


  // TODO: Copy over roll pitch and yaw

  // Copy over the sender ID
  if(idMapping.odDataPtr == NULL) {
    return false;
  }

  memcpy(idMapping.odDataPtr,&(dataToLoad->senderCanId),
         idMapping.dataLengthInBytes);

  return true;
}
#endif