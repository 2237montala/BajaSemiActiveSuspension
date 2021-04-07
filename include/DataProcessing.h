#pragma once

#include "config.h"
#include "stdbool.h"
#include "ShockData.h"
#include "fifofast.h"



#define CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME controlSystemShockData

_fff_declare_a(ControlSystemShockData_t,CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME,SHOCK_DATA_BUFFER_LEN,NUM_SHOCKS);

int DataProcessingInit();

int DataProcessingComputeVelocities(bool firstRun, uint32_t dt);

int DataProcessingComputeVelocity(ControlSystemShockData_t *dataToBeStored, uint32_t fifoIndex,
                                  float32_t dt, bool firstRun);

