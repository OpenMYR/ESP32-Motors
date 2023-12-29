#ifndef _ServoDriver_H_
#define _ServoDriver_H_

#include "MotorDriver.h"
#include <stdint.h>
#include "ESP32Servo.h"
#include "Op.h"
#include "CommandLayer.h"

#define MAX_MOTORS 15

class ServoDriver : public MotorDriver
{
public:
    ServoDriver();

    bool isMotorRunning(uint8_t motor_id);

    void isrStartIoDriver();
    void isrStopIoDriver();

    void motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID);
    void motorMove(int32_t deltaAngle, uint16_t rate, uint8_t motorID);
    void motorStop(int32_t wait_time, uint16_t precision, uint8_t motorID);
    void motorSleep(signed int wait_time, unsigned short precision, uint8_t motor_id);
    void abortCommand(uint8_t motorID);

    static ServoDriver *IRAM_ATTR getInstance();
    void changeMotorSettings(MotorDriver::config_setting setting, uint32_t data1, uint32_t data2, uint8_t motorID);

private:
    void initMotorGpio();

    static void IRAM_ATTR isrIo(void *);
    void IRAM_ATTR driver();

    static ServoDriver *instance;
    CommandLayer *commandInstance;
    TaskHandle_t motorTaskHandle;
    Servo servo[MAX_MOTORS];

    struct servo_conf
    {
        uint32_t min = 0;
        uint32_t max = 180;
    };

    servo_conf confs[MAX_MOTORS];
    bool motorDwell[MAX_MOTORS] = {0};
    bool motorSleeping[MAX_MOTORS] = {0};

    bool commandDone[MAX_MOTORS] = {1};
    uint16_t currentAngle[MAX_MOTORS] = {0};

    uint16_t startAngle[MAX_MOTORS] = {0};
    int16_t commandDeltaAngle[MAX_MOTORS] = {180};
    uint64_t startTime[MAX_MOTORS] = {90};
    uint64_t commandDeltaTime[MAX_MOTORS] = {0};

    bool isValidOpCode(Op *);
    void getNextOpForDriver(uint8_t id);
    void peekOpForDriver(uint8_t id);
};

#endif /* _ServoDriver_H_ */
