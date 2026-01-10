#ifndef MYR_BRUSHEDMOTORDRIVER_H
#define MYR_BRUSHEDMOTORDRIVER_H

#include <stdint.h>
#include <ESP32PWM.h>

/**
 * @file BrushedMotorDriver.h
 * @brief Declaration of the brushed motor driver singleton interface.
 */

#include "MotorDriver.h"
#include "Op.h"
#include "CommandLayer.h"

#define MAX_BrushedMotor_MOTORS 3

/**
 * @brief Driver implementation for controlling brushed DC motors via PWM.
 */
class BrushedMotorDriver : public MotorDriver
{
public:
    /**
     * @brief Create the brushed motor driver singleton and prepare GPIOs.
     */
    BrushedMotorDriver();

    /**
     * @brief Report whether a motor still has work queued.
     * @param motor_id Zero-based motor index.
     * @return True when the motor is active.
     */
    bool isMotorRunning(uint8_t motor_id);

    /**
     * @brief Launch the driver FreeRTOS task on CORE_1.
     */
    void isrStartIoDriver();
    
    /**
     * @brief Shutdown the driver task and disable outputs.
     */
    void isrStopIoDriver();

    /**
     * @brief Command a direction/speed target for a specific motor.
     */
    void motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID);

    /**
     * @brief Alias for motorGoTo that follows the MotorDriver interface.
     */
    void motorMove(int32_t targetAngle, uint16_t rate, uint8_t motorID);

    /**
     * @brief Pause a motor for a fixed duration then release.
     */
    void motorStop(int32_t wait_time, uint16_t precision, uint8_t motorID);

    /**
     * @brief Immediately halt the motor by cutting PWM.
     */
    void abortCommand(uint8_t motorID);

    /**
     * @brief Return the singleton BrushedMotorDriver instance.
     * @return Global driver pointer.
     */
    static BrushedMotorDriver *IRAM_ATTR getInstance();

    /**
     * @brief Apply driver configuration changes (UI placeholder).
     */
    void changeMotorSettings(MotorDriver::config_setting setting, uint32_t data1, uint32_t data2, uint8_t motorID);

private:
    /**
     * @brief Configure the GPIOs needed by the H-bridge and endstops.
     */
    void initMotorGpio();

    /**
     * @brief Task entry that initializes PWM and calls driver().
     */
    static void IRAM_ATTR isrIoBDC(void *);

    /**
     * @brief Driver loop that polls for commands and manages dwell times.
     */
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

    /**
     * @brief Program the PWM duty cycle and direction outputs.
     * @param power Duty cycle in 0..255.
     * @param direction Direction flag (1 forward, 0 reverse).
     */
    static void setSpeed(uint8_t power, int8_t direction);

    bool isValidOpCode(Op *);
    void getNextOpForDriver(uint8_t id);
    void peekOpForDriver(uint8_t id);
};

#endif // MYR_BRUSHEDMOTORDRIVER_H
