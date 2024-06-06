#ifndef _BrushedMotorDriver_H_
#define _BrushedMotorDriver_H_

#include "MotorDriver.h"
#include <stdint.h>
#include "Op.h"
#include "CommandLayer.h"
#include "ESP32PWM.h"

#define MAX_BrushedMotor_MOTORS 3

class BrushedMotorDriver : public MotorDriver
{
public:
    BrushedMotorDriver();

    bool isMotorRunning(uint8_t motor_id);

    void isrStartIoDriver();
    void isrStopIoDriver();

    void motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID);
    void motorMove(int32_t targetAngle, uint16_t rate, uint8_t motorID);
    void motorStop(int32_t wait_time, uint16_t precision, uint8_t motorID);
    void abortCommand(uint8_t motorID);
    static BrushedMotorDriver *IRAM_ATTR getInstance();
    void changeMotorSettings(MotorDriver::config_setting setting, uint32_t data1, uint32_t data2, uint8_t motorID);

private:
    void initMotorGpio();

    static void IRAM_ATTR isrIoBDC(void *);
    void IRAM_ATTR driver();

    static BrushedMotorDriver *instance;
    CommandLayer *commandInstance;
    TaskHandle_t motorTaskHandle;
    static void IRAM_ATTR endstop_a_interrupt();
    static void IRAM_ATTR endstop_b_interrupt();

    struct BrushedMotor_conf
    {
        double_t min = 0;
        double_t max = 3000;
    };

    BrushedMotor_conf confs[MAX_BrushedMotor_MOTORS];
    bool motorDwell = false;

    bool commandDone[MAX_BrushedMotor_MOTORS] = {1};
    double_t currentAngle[MAX_BrushedMotor_MOTORS] = {0};

    double_t startAngle[MAX_BrushedMotor_MOTORS] = {0};
    double_t commandDeltaAngle[MAX_BrushedMotor_MOTORS] = {180};
    uint64_t startTime[MAX_BrushedMotor_MOTORS] = {90};
    uint64_t commandDeltaTime[MAX_BrushedMotor_MOTORS] = {0};

    bool positiveDirection;
    uint32_t direction;
    uint32_t paused;
    uint32_t endstop_a;
    uint32_t endstop_b;
    int motorsControlled;
    uint8_t peekTicks;
    const uint8_t peekRate = 5;
    ESP32PWM bdcPWM1;
    ESP32PWM bdcPWM2;

    static void setSpeed(uint8_t power, int8_t direction);

    bool isValidOpCode(Op *);
    void getNextOpForDriver(uint8_t id);
    void peekOpForDriver(uint8_t id);
};

#endif /* _ServoDriver_H_ */
