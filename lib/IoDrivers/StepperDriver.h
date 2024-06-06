#ifndef _StepperDriver_H_
#define _StepperDriver_H_

#include "MotorDriver.h"
#include <stdint.h>
#include "Op.h"
#include "CommandLayer.h"

#define MAX_STEPPER_MOTORS 3

class StepperDriver : public MotorDriver
{
public:
    StepperDriver();

    bool isMotorRunning(uint8_t motor_id);

    void isrStartIoDriver();
    void isrStopIoDriver();

    void motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID);
    void motorMove(int32_t targetAngle, uint16_t rate, uint8_t motorID);
    void motorStop(int32_t wait_time, uint16_t precision, uint8_t motorID);
    void motorSleep(int32_t wait_time, uint16_t precision, uint8_t motorID);
    void abortCommand(uint8_t motorID);
    static StepperDriver *IRAM_ATTR getInstance();
    void changeMotorSettings(MotorDriver::config_setting setting, uint32_t data1, uint32_t data2, uint8_t motorID);

private:
    void initMotorGpio();

    static void IRAM_ATTR isrIoStep(void *);
    void IRAM_ATTR driver();

    static StepperDriver *instance;
    CommandLayer *commandInstance;
    TaskHandle_t motorTaskHandle;
    static void IRAM_ATTR endstop_a_interrupt();
    static void IRAM_ATTR endstop_b_interrupt();
    static bool IRAM_ATTR  isEndstopTripped();
    static void setStepRate(int32_t rate);
    static void setSleep(boolean sleep);
    static void addSteps(uint32_t steps);
    static void addSteps(uint32_t steps, uint16_t microstepRate);
    static uint16_t getMicroStepRate();
    static void setMicroStepRate(uint16_t microstepRate);
    static void setMicroStepRate(bool MS1, bool MS2, bool MS3);
    void checkLocation();

    static void pcnt_setup_init(uint8_t pin);
    static void IRAM_ATTR pcnt_intr_handler(void *arg);

    struct stepper_conf
    {
        double_t min = 0;
        double_t max = 3000;
    };

    stepper_conf confs[MAX_STEPPER_MOTORS];
    bool motorDwell = false;
    bool motorSleeping = false;

    bool commandDone[MAX_STEPPER_MOTORS] = {1};
    double_t currentAngle[MAX_STEPPER_MOTORS] = {0};  // angle is integer of steps in stepper driver.
    double_t homeSoftAngle[MAX_STEPPER_MOTORS] = {0};  // 
    double_t homeRealAngle[MAX_STEPPER_MOTORS] = {0};  // 

    double_t startAngle[MAX_STEPPER_MOTORS] = {0};
    double_t commandDeltaAngle[MAX_STEPPER_MOTORS] = {180};
    uint64_t startTime[MAX_STEPPER_MOTORS] = {90};
    uint64_t commandDeltaTime[MAX_STEPPER_MOTORS] = {0};
    uint64_t degreesToSteps(double);

    bool isValidOpCode(Op *);
    void getNextOpForDriver(uint8_t id);
    void peekOpForDriver(uint8_t id);


};

#endif /* _ServoDriver_H_ */
