#ifndef _CommandLayer_H_
#define _CommandLayer_H_

#include <stdint.h>
#include "MotorDriver.h"
#include "OpBuffer.h"

class CommandLayer
{
public:
    CommandLayer();
    static CommandLayer *getInstance();
    static MotorDriver *driver;
    static void init();
    static void opcodeMove(signed int step_num, unsigned short step_rate, uint8_t motor_id);
    static void opcodeGoto(signed int step_num, unsigned short step_rate, uint8_t motor_id);
    static void opcodeStop(signed int wait_time, unsigned short precision, uint8_t motor_id);
    static void opcodeMotorSetting(MotorDriver::config_setting setting, uint32_t data1, uint32_t data2, uint8_t motor_id);
    static void opcodeAbortCommand(uint8_t motor_id);
    static void getNextOp(uint8_t driverId);
    static void peekNextOp(uint8_t driverId);

private:
    static CommandLayer *instance;
    static void fetchMotorOpCode(uint8_t);
    static void FillDriverFromQueue();
    TaskHandle_t *motorTaskOpCode;
    static void parseSubmittOp(uint8_t, Op *);
};

struct motor_public_to_driver_id_map
{
    uint16_t publicId;
    uint16_t driverId;
    MotorDriver *driver;
};
#endif /* _CommandLayer_H_ */
