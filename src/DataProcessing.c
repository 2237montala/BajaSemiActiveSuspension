#include "DataProcessing.h"

// Data collection is the producer and this class is the consumer
// We need the name of the incoming data fifo
#include "DataCollection.h"

/*
 * PURPOSE
 *      Approximation of computing the velocity from two accelerations and the previous velocity
 * PARAMETERS
 *      v1 - floating point number of the previous velocity value
 *      a1 - floating point number of the previous acceleration
 *      a2 - floating point number of the current acceleration
 * RETURNS
 * 
 */
static float32_t computeVelocityFromAccel(float32_t v1, float32_t a1, float32_t a2);

/*
 * PURPOSE
 *      Compute the velocity from two different x positions and the time between them
 * PARAMETERS
 *      d1 - floating point number of the previous x position
 *      d2 - floating point number of the current x position
 *      deltaT - integer of number of milliseconds between samples
 * RETURNS
 *      float - the velocity if no errors.
 */
static float32_t computeVelocityFromDisp(float32_t d1, float32_t d2, uint32_t deltaT);

int DataProcessingInit() {
    _fff_init_a(CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME,NUM_SHOCKS);
    return 1;
}

int DataProcessingComputeVelocities(bool firstRun, uint32_t dt) {
    // Iterate through all the fifos and compute the velocity over the last 
    // n samples. Store this new velocity in the control system data fifo
    ControlSystemShockData_t dataToBeStored = { 0 };
    uint8_t samplesNotComputed = 0;
    

    for(int i = 0; i < NUM_SHOCKS; i++) {
        if(DataProcessingComputeVelocity(&dataToBeStored,i,dt,firstRun) == 0) {
            // No error so add the data to the fifo
            // Check if there is enough room in the fifo
            if(_fff_is_full(CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME[i])) {
                // Pop off the oldest value
                _fff_remove(CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME[i],1);
            }
            _fff_write_lite(CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME[i],dataToBeStored);
        } else {
            // Increment a counter of failed velocity computations
            samplesNotComputed++;
        }
    }

    return samplesNotComputed;
}


int DataProcessingComputeVelocity(ControlSystemShockData_t *dataToBeStored, uint32_t fifoIndex,
                                  uint32_t dt, bool firstRun) {
    if(_fff_is_empty(IMCOMING_SHOCK_SENSOR_DATA_FIFO_NAME[fifoIndex])) {
        // No data in fifo so no point in continuing
        return -1;
    }
    IncomingShockSensorData_t sensorData = _fff_read(IMCOMING_SHOCK_SENSOR_DATA_FIFO_NAME[fifoIndex]);

    if(firstRun) {
        // Compute velocities using 0 as a starting point
        dataToBeStored->dx = computeVelocityFromAccel(0.0f, 0.0f,
                                                    sensorData.accels[DX_VELOCTIY_AXIS]);
        dataToBeStored->dy = computeVelocityFromAccel(0.0f,0.0f,
                                                    sensorData.accels[DY_VELOCITY_AXIS]);

        dataToBeStored->dLinearPos = computeVelocityFromDisp(0.0f,sensorData.linearPos,dt);
    } else {
        // Compute velocity with previous values
        ControlSystemShockData_t prevControlSysData = 
            _fff_peek(CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME[fifoIndex],
                    _fff_mem_level(CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME[fifoIndex]));

        dataToBeStored->dx = computeVelocityFromAccel(prevControlSysData.dx,
                                            prevControlSysData.rawData.accels[DX_VELOCTIY_AXIS],
                                            sensorData.accels[DX_VELOCTIY_AXIS]);
        dataToBeStored->dy = computeVelocityFromAccel(prevControlSysData.dy,
                                            prevControlSysData.rawData.accels[DY_VELOCITY_AXIS],
                                            sensorData.accels[DY_VELOCITY_AXIS]);

        dataToBeStored->dLinearPos = computeVelocityFromDisp(prevControlSysData.dLinearPos,
                                                            prevControlSysData.rawData.linearPos,
                                                            sensorData.linearPos);
    }

    // Save the old data to the previous data field of the new data
    memcpy(&(dataToBeStored->rawData),&sensorData,sizeof(IncomingShockSensorData_t));

    // No errors
    return 0;
}

ControlSystemShockData_t DataProcessingGetNewestData(uint32_t index) {
    return _fff_peek(CONTROL_SYSTEM_SHOCK_DATA_FIFO_NAME[index],0);
}

static float32_t computeVelocityFromAccel(float32_t v1, float32_t a1, float32_t a2) {
    // Simple approximation of acceleration integration
    return v1 * ((a1 + a2)/2);
}

static float32_t computeVelocityFromDisp(float32_t d1, float32_t d2, uint32_t deltaT) {
    // Compute the derivative the of the displacement by doing rise over run

    // Check if the difference will equal zero becasue then we'd end up with inf
    // Since they are floating point numbers there is a small chance that the number
    // ends up being exactly 0
    if ((d1 - d2) == 0) {
        return 0;
    }
    return (d1 - d2) / (float32_t)deltaT;
}