#include "OpBuffer.h"
#include "esp_log.h"

OpBuffer *OpBuffer::instance = NULL;
SemaphoreHandle_t xMutex = NULL;
uint16_t head[OP_BUFFER_COUNT];
uint16_t tail[OP_BUFFER_COUNT];
uint16_t length[OP_BUFFER_SIZE];

OpBuffer::OpBuffer()
{
	Serial.write("OpBuffer const\n");
	delay(200);
	if (xMutex == NULL)
	{
		xMutex = xSemaphoreCreateMutex();
		if (xMutex != NULL)
		{
			xSemaphoreGive(xMutex);
		}
	}
	init();
}

void OpBuffer::init()
{

	OpBuffer::instance->reset();
}

OpBuffer *OpBuffer::getInstance()
{
	if (OpBuffer::instance == NULL)
	{
		OpBuffer::instance = new OpBuffer();
	}
	return OpBuffer::instance;
}

int8_t OpBuffer::storeOp(Op *op)
{

	int8_t error = -1;

	if (xSemaphoreTake(xMutex, 200) == pdTRUE)
	{
		if (validIndex(op->motorID))
		{
			if (length[op->motorID] < OP_BUFFER_SIZE)
			{
				opQueue[op->motorID][(tail[op->motorID] + 1) % OP_BUFFER_SIZE] = *op;
				tail[op->motorID] = (tail[op->motorID] + 1) % OP_BUFFER_SIZE;
				length[op->motorID]++;
				error = 0;
			}
			else
			{
				log_e("OpBuffer Failed: Buffer %d Full\n", op->motorID);
				error = -1;
			}
		}

		xSemaphoreGive(xMutex);
	}
	else
	{
		log_e("OpBuffer Failed: Mutex Locked\n");
		error = -2;
	}
	return error;
}

Op *OpBuffer::getOp(uint8_t id)
{
	if (!validIndex(id))
		return NULL;
	Op *tempOp;
	tempOp = NULL;

	if (length[id] > 0)
	{
		tempOp = &opQueue[id][head[id]];

		head[id] = (head[id] + 1) % OP_BUFFER_SIZE;
		length[id]--;
	}

	return tempOp;
}

Op *OpBuffer::peekOp(uint8_t id)
{
	if (!validIndex(id)){
		return NULL;
	}
	Op *tempOp;
	tempOp = NULL;

	if (length[id] > 0)
	{
		tempOp = &opQueue[id][head[id]];
	}

	return tempOp;
}

void OpBuffer::clear(uint8_t id)
{
	if (!validIndex(id))
		return;

	if (xSemaphoreTake(xMutex, 200) == pdTRUE)
	{
		//tail[id]
		head[id] = (tail[id] + 1) % OP_BUFFER_SIZE;
		length[id] = 0;
		xSemaphoreGive(xMutex);
	}
	else
	{
		log_e("OpBuffer Failed: Mutex Locked\n");
	}
}

void OpBuffer::killCurrentOp(uint8_t id)
{
	if (!validIndex(id))
		return;
	uint8_t data[] = {0x00, 0x00, 'K', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, id};
	Op StopOp = Op(data);
	if (storeOp(&StopOp) >= 0)
	{
		log_v("kill cool %d", 'K');
	}
	else
	{
		log_e("kill error %d", 'K');
	}
}

bool OpBuffer::isEmpty(uint8_t id)
{
	if (!validIndex(id))
		return true;

	return length[id] == 0;
}

bool OpBuffer::isFull(uint8_t id)
{
	if (!validIndex(id))
		return true;
	bool isBufferFull = true;
	if (xSemaphoreTake(xMutex, 200) == pdTRUE)
	{
		isBufferFull = length[id] == OP_BUFFER_SIZE;

		xSemaphoreGive(xMutex);
	}
	else
	{
		log_e("OpBuffer Failed: Mutex Locked\n");
	}

	return isBufferFull;
}

void OpBuffer::reset()
{
	//memset(opQueue, 0, sizeof(opQueue));
	for (int i = 0; i < OP_BUFFER_COUNT; i++)
	{
		clear(i);
	}
}

uint32_t OpBuffer::opBufferCapacity(uint8_t id)
{
	if (!validIndex(id))
		return 0;
	return (uint32_t)OP_BUFFER_SIZE;
}

bool OpBuffer::validIndex(uint8_t id)
{
	if (id < OP_BUFFER_COUNT)
		return true;
	return false;
}
