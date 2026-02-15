#include "OpBuffer.h"

#include "esp_log.h"

namespace {
const char *TAG = "OpBuffer";
SemaphoreHandle_t gMutex = nullptr;
uint16_t gHead[OP_BUFFER_COUNT];
uint16_t gTail[OP_BUFFER_COUNT];
uint16_t gLength[OP_BUFFER_COUNT];
} // namespace

OpBuffer *OpBuffer::instance = nullptr;

OpBuffer::OpBuffer() {
    if (gMutex == nullptr) {
        gMutex = xSemaphoreCreateMutex();
    }
    init();
}

void OpBuffer::init() {
    OpBuffer::instance->reset();
}

OpBuffer *OpBuffer::getInstance() {
    if (OpBuffer::instance == nullptr) {
        OpBuffer::instance = new OpBuffer();
    }
    return OpBuffer::instance;
}

int8_t OpBuffer::storeOp(Op *op) {
    int8_t error = -1;

    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (validIndex(op->motorID)) {
            if (gLength[op->motorID] < OP_BUFFER_SIZE) {
                opQueue[op->motorID][(gTail[op->motorID] + 1) % OP_BUFFER_SIZE] = *op;
                gTail[op->motorID] = (gTail[op->motorID] + 1) % OP_BUFFER_SIZE;
                gLength[op->motorID]++;
                error = 0;
            } else {
                ESP_LOGE(TAG, "storeOp failed: buffer %u full", static_cast<unsigned int>(op->motorID));
                error = -1;
            }
        }

        xSemaphoreGive(gMutex);
    } else {
        ESP_LOGE(TAG, "storeOp failed: mutex timeout");
        error = -2;
    }
    return error;
}

Op *OpBuffer::getOp(uint8_t id) {
    if (!validIndex(id)) return nullptr;

    Op *tempOp = nullptr;
    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (gLength[id] > 0) {
            tempOp = &opQueue[id][gHead[id]];
            gHead[id] = (gHead[id] + 1) % OP_BUFFER_SIZE;
            gLength[id]--;
        }
        xSemaphoreGive(gMutex);
    } else {
        ESP_LOGE(TAG, "getOp failed: mutex timeout");
    }

    return tempOp;
}

Op *OpBuffer::peekOp(uint8_t id) {
    if (!validIndex(id)) return nullptr;

    Op *tempOp = nullptr;
    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        if (gLength[id] > 0) {
            tempOp = &opQueue[id][gHead[id]];
        }
        xSemaphoreGive(gMutex);
    } else {
        ESP_LOGE(TAG, "peekOp failed: mutex timeout");
    }

    return tempOp;
}

void OpBuffer::clear(uint8_t id) {
    if (!validIndex(id)) return;

    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        gHead[id] = (gTail[id] + 1) % OP_BUFFER_SIZE;
        gLength[id] = 0;
        xSemaphoreGive(gMutex);
    } else {
        ESP_LOGE(TAG, "clear failed: mutex timeout");
    }
}

void OpBuffer::killCurrentOp(uint8_t id) {
    if (!validIndex(id)) return;

    uint8_t data[] = {0x00, 0x00, 'K', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, id};
    Op stopOp(data);
    if (storeOp(&stopOp) >= 0) {
        ESP_LOGV(TAG, "kill op enqueued for motor %u", static_cast<unsigned int>(id));
    } else {
        ESP_LOGE(TAG, "kill op enqueue failed for motor %u", static_cast<unsigned int>(id));
    }
}

bool OpBuffer::isEmpty(uint8_t id) {
    if (!validIndex(id)) return true;

    bool isBufferEmpty = true;
    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        isBufferEmpty = gLength[id] == 0;
        xSemaphoreGive(gMutex);
    } else {
        ESP_LOGE(TAG, "isEmpty failed: mutex timeout");
    }
    return isBufferEmpty;
}

bool OpBuffer::isFull(uint8_t id) {
    if (!validIndex(id)) return true;

    bool isBufferFull = true;
    if (xSemaphoreTake(gMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        isBufferFull = gLength[id] == OP_BUFFER_SIZE;
        xSemaphoreGive(gMutex);
    } else {
        ESP_LOGE(TAG, "isFull failed: mutex timeout");
    }

    return isBufferFull;
}

void OpBuffer::reset() {
    for (int i = 0; i < OP_BUFFER_COUNT; i++) {
        clear(i);
    }
}

uint32_t OpBuffer::opBufferCapacity(uint8_t id) {
    if (!validIndex(id)) return 0;

    return static_cast<uint32_t>(OP_BUFFER_SIZE);
}

bool OpBuffer::validIndex(uint8_t id) {
    return id < OP_BUFFER_COUNT;
}
