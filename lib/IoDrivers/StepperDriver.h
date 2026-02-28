#ifndef MYR_STEPPERDRIVER_H
#define MYR_STEPPERDRIVER_H

/**
 * @file StepperDriver.h
 * @brief Declaration of the stepper motor driver singleton.
 */

#include <stdint.h>

#include "MotorDriver.h"
#include "Op.h"
#include "CommandLayer.h"

#define MAX_STEPPER_MOTORS 3

/**
 * @brief Driver for managing stepper motor.
 */
class StepperDriver : public MotorDriver
{
public:
    struct MotionPlan
    {
        int32_t goalStep = 0;
        uint32_t steps = 0;
        uint64_t durationUs = 0;
    };

    static MotionPlan planRelativeMove(int32_t currentStep, int32_t deltaStep, uint16_t stepRate);
    static MotionPlan planAbsoluteMove(int32_t currentStep, int32_t targetStep, uint16_t stepRate);
    static uint64_t planDwellDurationUs(int32_t waitCycles, uint16_t cycleRateHz);
    static bool shouldRejectForEndstop(char opcode, bool endstopTripped);

    /** @brief Create the singleton and initialize GPIO. */
    StepperDriver();

    /**
     * @brief Determine if a motor is still executing a command.
     * @param motor_id One-based motor index.
     * @return True when the motor is active.
     */
    bool isMotorRunning(uint8_t motorID);

    /** @brief Launch the stepper driver task on CORE_1. */
    void isrStartIoDriver();

    /** @brief Stop the stepper task and disable the driver. */
    void isrStopIoDriver();

    /**
     * @brief Move a motor to an absolute target position.
     */
    void motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID);

    /**
     * @brief Move a motor by a relative delta.
     */
    void motorMove(int32_t deltaAngle, uint16_t rate, uint8_t motorID);

    /**
     * @brief Pause the motor for the provided wait cycles.
     */
    void motorStop(signed int wait_time, unsigned short precision, uint8_t motorID);

    /**
     * @brief Put the motor to sleep for the wait duration.
     */
    void motorSleep(signed int wait_time, unsigned short precision, uint8_t motorID);

    /** @brief Cancel the current motor command immediately. */
    void abortCommand(uint8_t motorID);
    void setOpcodeContext(uint32_t op_seq, uint8_t motor_id) override;

    /** @brief Return the singleton instance. */
    static StepperDriver *IRAM_ATTR getInstance();

    /**
     * @brief Update driver configuration for a specific motor.
     */
    void changeMotorSettings(MotorDriver::config_setting setting, uint32_t data1, uint32_t data2, uint8_t motorID);

    /** @brief Query whether motor-specific endstop input is active. */
    bool IRAM_ATTR isEndstopTripped(uint8_t motor_id) override;

    /** @brief Query whether stepper endstop input for motor 1 is active (legacy API). */
    static bool IRAM_ATTR  isEndstopTripped();

    /**
     * @brief Configure the endstop polarity for a specific motor.
     * @param setting Target setting value.
     * @param motor_id One-based motor identifier.
     */
    esp_err_t setEndstopTrippedPinSetting(uint8_t setting, uint8_t motor_id) override;

    /**
     * @brief Configure the endstop polarity for motor 1 (legacy API).
     * @param setting Target setting value.
     */
    static esp_err_t setEndstopTrippedPinSetting(uint8_t setting);

    /** @brief Return configured endstop polarity for a specific motor. */
    bool getEndstopTrippedPinSetting(uint8_t motor_id) override;

    /** @brief Return configured endstop polarity for motor 1 (legacy API). */
    static bool getEndstopTrippedPinSetting();


private:
    void initMotorGpio();

    static void IRAM_ATTR isrIoStep(void *);
    void driver();

    static void IRAM_ATTR onPulseRunComplete(uint32_t pulsesCompleted, uint32_t runToken, void *userCtx);
    void applyPulseProgress(uint32_t pulsesCompleted);
    void stopActiveCommandForEndstop();
    bool tryStartPendingPulse(uint8_t motorIndex);
    uint32_t consumeOpcodeContextSeq(uint8_t motorIndex);

    static StepperDriver *instance;
    CommandLayer *commandInstance;
    TaskHandle_t motorTaskHandle;
    static void setSleep(bool sleep);
    static void addSteps(uint32_t steps);
    static void addSteps(uint32_t steps, uint16_t microstepRate);
    static uint16_t getMicroStepRate();
    static void setMicroStepRate(uint16_t microstepRate);
    static void setMicroStepRate(bool MS1, bool MS2, bool MS3);

    struct stepper_conf
    {
        double min = 0;
        double max = 3000;
    };

    stepper_conf confs[MAX_STEPPER_MOTORS];
    bool motorDwell = false;
    bool motorSleeping = false;

    bool commandDone[MAX_STEPPER_MOTORS] = {1};
    double currentAngle[MAX_STEPPER_MOTORS] = {0};  // angle is integer of steps in stepper driver.
    double homeSoftAngle[MAX_STEPPER_MOTORS] = {0};  // 
    double homeRealAngle[MAX_STEPPER_MOTORS] = {0};  // 

    double startAngle[MAX_STEPPER_MOTORS] = {0};
    double commandDeltaAngle[MAX_STEPPER_MOTORS] = {180};
    uint64_t startTime[MAX_STEPPER_MOTORS] = {90};
    uint64_t commandDeltaTime[MAX_STEPPER_MOTORS] = {0};
    bool pulseStartPending[MAX_STEPPER_MOTORS] = {0};
    uint32_t pendingPulseSteps[MAX_STEPPER_MOTORS] = {0};
    uint16_t pendingPulseRateHz[MAX_STEPPER_MOTORS] = {0};
    uint64_t pendingPulseDurationUs[MAX_STEPPER_MOTORS] = {0};
    uint32_t opcodeContextSeq[MAX_STEPPER_MOTORS] = {0};
    uint32_t pendingPulseToken[MAX_STEPPER_MOTORS] = {0};
    uint32_t activePulseToken[MAX_STEPPER_MOTORS] = {0};
    uint32_t activeOpcodeSeq[MAX_STEPPER_MOTORS] = {0};
    uint32_t abortWatermarkSeq[MAX_STEPPER_MOTORS] = {0};
    uint64_t degreesToSteps(double);

    bool isValidOpCode(Op *);
    void getNextOpForDriver(uint8_t id);
    void peekOpForDriver(uint8_t id);
    int motorsControlled;
};

#endif // MYR_STEPPERDRIVER_H
