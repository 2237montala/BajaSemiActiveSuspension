#include "Fifo.h"


struct fifoObject FifoInit(size_t numByteOfStoredItem, void * staticBuff ,size_t length) {
    // Create new static array for the fifo
    struct fifoObject newFifo;

    newFifo.data = staticBuff;
    newFifo.head = staticBuff;
    newFifo.tail = newFifo.head;
    newFifo.itemSize = numByteOfStoredItem;

    // Clear the memory space
    memset(staticBuff,0x0,numByteOfStoredItem*length);

    return newFifo;
}

bool FifoEnqueue(struct fifoObject fifo, void * newItem) {
    if(!FifoFull(fifo)) {
        // Add item to the next value of the array
        // Copy the item to the tail by using the memory address of the array
        memcpy(fifo.data + fifo.tail, newItem, fifo.itemSize);
    }
}