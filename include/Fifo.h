#pragma once
// // Include arm math libraies for variable sizes
#include "arm_math.h"
#include "stdbool.h"
// #include "DataCollection.h"

struct fifoObject {
    uint32_t *head;
    uint32_t *tail;
    size_t itemSize;
    uintptr_t *data;
};

struct fifoObject FifoInit(size_t numByteOfStoredItem, void * staticBuff ,size_t length);

bool FifoEnqueue(struct fifoObject fifo, void * newItem);

struct ShockSensorData FifoPeek();

struct ShockSensorData FifoPeekIndex(int);

// Helper functions ----------
bool FifoFull(struct fifoObject fifo);

bool FifoEmpty(struct fifoObject fifo);