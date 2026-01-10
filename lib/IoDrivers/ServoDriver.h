#ifndef _ServoDriver_H_
#define _ServoDriver_H_

/**
 * @file ServoDriver.h
 * @brief Declaration of the servo motor driver singleton.
 */

#include <ESP32Servo.h>

#include "MotorDriver.h"
#include "Op.h"
#include "CommandLayer.h"

#define MAX_MOTORS 15

/**
 * @brief Driver for managing PWM-controlled hobby servos.
 */
class ServoDriver : public MotorDriver
{
public:
    /**
     * @brief Construct the ServoDriver and attach servo objects.
     */
    ServoDriver();

    /**
     * @brief Determine if a servo is still executing a command.
     * @param motor_id One-based servo index.
     * @return True when the servo remains active.
     */
    bool isMotorRunning(uint8_t motor_id);

    /**
     * @brief Launch the servo driver task on CORE_1.
     */
    void isrStartIoDriver();
    void isrStopIoDriver();

    /**
     * @brief Command an absolute angle target for a servo.
     */
    void motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID);

    /**
     * @brief Increment the servo angle by a delta value.
     */
    void motorMove(int32_t deltaAngle, uint16_t rate, uint8_t motorID);

    /**
     * @brief Pause the servo for the provided wait cycles.
     */
    void motorStop(int32_t wait_time, uint16_t precision, uint8_t motorID);

    /**
     * @brief Put the motor to sleep for the wait duration.
     */
    void motorSleep(signed int wait_time, unsigned short precision, uint8_t motor_id);

    /**
     * @brief Abort the current command immediately.
     */
    void abortCommand(uint8_t motorID);

    /**
     * @brief Return the singleton ServoDriver instance.
     * @return Global driver pointer.
     */
    static ServoDriver *IRAM_ATTR getInstance();

    /**
     * @brief Update servo configuration such as bounds.
     */
    void changeMotorSettings(MotorDriver::config_setting setting, uint32_t data1, uint32_t data2, uint8_t motorID);

private:
    /**
     * @brief Attach servo objects to their GPIO pins.
     */
    void initMotorGpio();

    /**
     * @brief Entry point for the servo RTOS task.
     */
    static void IRAM_ATTR isrIo(void *);
    
    /**
     * @brief Driver loop that tracks servo positions and polls commands.
     */
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

    /**
     * @brief Fetch the next queued command from the command layer.
     * @param id Zero-based motor index.
     */
    void getNextOpForDriver(uint8_t id);

    /**
     * @brief Peek at the next operation without removing it.
     * @param id Zero-based motor index.
     */
    void peekOpForDriver(uint8_t id);
};

#endif /* _ServoDriver_H_ */
