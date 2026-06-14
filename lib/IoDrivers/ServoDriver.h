#ifndef MYR_SERVODRIVER_H
#define MYR_SERVODRIVER_H

/**
 * @file ServoDriver.h
 * @brief Declaration of the servo motor driver singleton.
 */

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
    bool isMotorRunning(uint8_t motor_id) override;

    /**
     * @brief Launch the servo driver task on CORE_1.
     */
    void isrStartIoDriver() override;
    void isrStopIoDriver() override;

    /**
     * @brief Command an absolute target in servo driver units.
     */
    void motorGoTo(int32_t targetUnits, uint16_t rate, uint8_t motorID) override;

    /**
     * @brief Increment the servo target by a delta in driver units.
     */
    void motorMove(int32_t deltaUnits, uint16_t rate, uint8_t motorID) override;

    /**
     * @brief Pause the servo for the provided Stop duration.
     */
    void motorStop(signed int wait_time, unsigned short interval_us, uint8_t motorID) override;

    /**
     * @brief Put the motor to sleep for the wait duration.
     */
    void motorSleep(signed int wait_time, unsigned short precision, uint8_t motor_id) override;

    // ServoDriver satisfies these MotorDriver hooks with no-op/not-supported behavior.
    void setOpcodeContext(uint32_t op_seq, uint8_t motor_id) override;

    /**
     * @brief Abort the current command immediately.
     */
    void abortCommand(uint8_t motorID) override;

    /**
     * @brief Return the singleton ServoDriver instance.
     * @return Global driver pointer.
     */
    static ServoDriver *getInstance();

    /**
     * @brief Update servo configuration such as bounds.
     */
    void changeMotorSettings(MotorDriver::config_setting setting, uint32_t data1, uint32_t data2, uint8_t motorID) override;

    bool isEndstopTripped(uint8_t motor_id) override;
    esp_err_t setEndstopTrippedPinSetting(uint8_t setting, uint8_t motor_id) override;
    bool getEndstopTrippedPinSetting(uint8_t motor_id) override;

private:
    /**
     * @brief Attach servo objects to their GPIO pins.
     */
    void initMotorGpio() override;

    /**
     * @brief Entry point for the servo RTOS task.
     */
    static void isrIo(void *);
    
    /**
     * @brief Driver loop that tracks servo positions and polls commands.
     */
    void driver() override;

    static ServoDriver *instance;
    CommandLayer *commandInstance;
    TaskHandle_t motorTaskHandle;

    struct servo_conf
    {
        uint32_t min = 0;
        uint32_t max = 180;
    };

    servo_conf confs[MAX_MOTORS];
    bool motorDwell[MAX_MOTORS] = {0};
    bool motorSleeping[MAX_MOTORS] = {0};

    bool commandDone[MAX_MOTORS] = {1};
    int32_t currentAngleMillideg[MAX_MOTORS] = {0};

    int32_t startAngleMillideg[MAX_MOTORS] = {0};
    int32_t commandDeltaAngleMillideg[MAX_MOTORS] = {0};
    uint64_t startTime[MAX_MOTORS] = {90};
    uint64_t commandDeltaTime[MAX_MOTORS] = {0};
    bool pwmAttached[MAX_MOTORS] = {0};

    bool isValidOpCode(Op *);
    void attachPwmChannel(uint8_t motorIndex);
    void detachPwmChannel(uint8_t motorIndex);
    void writeServoAngleMillideg(uint8_t motorIndex, int32_t angleMillideg);

    /**
     * @brief Fetch the next queued command from the command layer.
     * @param id Zero-based motor index.
     */
    void getNextOpForDriver(uint8_t id) override;

    /**
     * @brief Peek at the next operation without removing it.
     * @param id Zero-based motor index.
     */
    void peekOpForDriver(uint8_t id) override;
};

#endif // MYR_SERVODRIVER_H
