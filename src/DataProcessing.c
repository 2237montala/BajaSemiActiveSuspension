#include "DataProcessing.h"



int DataProcessingInit() {
    _fff_init_a(CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME,NUM_SHOCKS);
    return 0;
}

int DataProcessingComputeVelocities() {
    // Iterate through all the fifos and compute the velocity over the last 
    // n samples. Store this new velocity in the control system data fifo

    for(int i = 0; i < NUM_SHOCKS; i++) {
        // Rebase fifo

    }
}

int DataProcessingComputeVelocity(uint32_t fifoIndex, float32_t *computedVelocity) {
    // Compute 
}