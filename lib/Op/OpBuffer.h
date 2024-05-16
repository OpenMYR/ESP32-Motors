#ifndef _OpBuffer_H_
#define _OpBuffer_H_
#include <ESPAsyncWebServer.h>
#include "Op.h"

#define OP_BUFFER_SIZE 128  // OpCode pointers in each OpCode buffer
#define OP_BUFFER_COUNT 16 // Number of Opcode buffers for motors + 1

class OpBuffer
{
public:
    OpBuffer();
    static OpBuffer* getInstance();
    int8_t storeOp(Op *);
    Op *getOp(uint8_t);
    //static int removeFirstOp(uint8_t);
    void clear(uint8_t);
    bool isEmpty(uint8_t);
    bool isFull(uint8_t);
    void reset();
    uint32_t opBufferCapacity(uint8_t);
    void killCurrentOp(uint8_t);
    Op *peekOp(uint8_t);

private:
    static OpBuffer* instance;
    //static uint16_t length[OP_BUFFER_COUNT];
    Op opQueue[OP_BUFFER_COUNT][OP_BUFFER_SIZE];
    static void init();
    bool validIndex(uint8_t);
};

#endif /* _OpBuffer_H_ */
