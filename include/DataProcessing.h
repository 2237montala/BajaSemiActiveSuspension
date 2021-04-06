#pragma once

#include "config.h"
#include "ShockData.h"
#include "fifofast.h"

// Data collection is the producer and this class is the consumer
#include "DataCollection.h"

#define CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME controlSystemShockData

_fff_declare_a(ControlSystemShockData_t,CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME,SHOCK_DATA_BUFFER_LEN,NUM_SHOCKS);

int DataProcessingInit();

int DataProcessingComputeVelocities();

int DataProcessingComputeVelocity(uint32_t fifoIndex, float32_t *computedVelocity);

