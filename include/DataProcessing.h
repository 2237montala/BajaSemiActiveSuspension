#pragma once

#include "config.h"
#include "ShockData.h"
#include "fifofast.h"

#define CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME controlSystemShockData

_fff_declare_a(ShockVelocitiesStruct_t,CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME,SHOCK_DATA_BUFFER_LEN,NUM_SHOCKS);