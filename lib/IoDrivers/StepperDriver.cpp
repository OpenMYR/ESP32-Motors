/**
 * @file StepperDriver.cpp
 * @brief Implements the singleton stepper motor driver.
 */
#include <reent.h>
#include <math.h>
#include <string.h>

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "PulseEngine.h"
#include "EndStop.h"
#include "StepperDriver.h"
#include "OpBuffer.h"

#define UPDATE_FREQ 1000
#define MOTOR_LOOP_STACK_BYTES 4096

#define UPDATE_DWELL 1000 / UPDATE_FREQ
#define motorInterfaceType 1

namespace {
const char *TAG = "StepperDriver";
bool gGpioIsrServiceInstalled = false;
constexpr uint32_t kCommandTimingMarginUs = 100;
constexpr uint64_t kMicrosecondsPerSecond = 1000000ULL;
constexpr uint64_t kCommandTimeoutMinGraceUs = 50000ULL;
constexpr uint64_t kCommandTimeoutGraceDivisor = 4ULL;
constexpr UBaseType_t kMotorTaskPriority = 5;
constexpr BaseType_t kMotorTaskCore = 1;
constexpr bool kPulseInitOnDriverCore = true;

uint32_t steps_between(int32_t a, int32_t b) {
    return a >= b ? static_cast<uint32_t>(a - b) : static_cast<uint32_t>(b - a);
}

uint64_t timeout_grace_us(uint64_t scheduledWindowUs)
{
    const uint64_t proportionalGrace = scheduledWindowUs / kCommandTimeoutGraceDivisor;
    return proportionalGrace > kCommandTimeoutMinGraceUs ? proportionalGrace : kCommandTimeoutMinGraceUs;
}

bool sequence_is_after(uint32_t seq, uint32_t watermark)
{
    return static_cast<int32_t>(seq - watermark) > 0;
}

bool sequence_is_stale(uint32_t seq, uint32_t watermark)
{
    if (seq == 0) return false;
    return !sequence_is_after(seq, watermark);
}
} // namespace

bool StepperDriver::tryStartPendingPulse(uint8_t motorIndex)
{
    if (motorIndex >= static_cast<uint8_t>(motorsControlled)) return false;
    if (!pulseStartPending[motorIndex]) return true;
    if (pendingPulseSteps[motorIndex] == 0 || pendingPulseRateHz[motorIndex] == 0)
    {
        pulseStartPending[motorIndex] = false;
        commandDone[motorIndex] = true;
        return false;
    }

    PulseEngine::StartConfig pulseConfig = {};
    pulseConfig.pulseCount = pendingPulseSteps[motorIndex];
    pulseConfig.startSpeedHz = pendingPulseRateHz[motorIndex];
    pulseConfig.endSpeedHz = pendingPulseRateHz[motorIndex];
    pulseConfig.runToken = pendingPulseToken[motorIndex];

    const esp_err_t pulseErr = PulseEngine::startPulses(pulseConfig);
    if (pulseErr == ESP_OK)
    {
        startTime[motorIndex] = esp_timer_get_time();
        commandDeltaTime[motorIndex] = pendingPulseDurationUs[motorIndex] == UINT64_MAX
                                           ? UINT64_MAX
                                           : (startTime[motorIndex] + pendingPulseDurationUs[motorIndex] + kCommandTimingMarginUs);
        activePulseToken[motorIndex] = pendingPulseToken[motorIndex];
        pulseStartPending[motorIndex] = false;
        return true;
    }

    if (pulseErr == ESP_ERR_INVALID_STATE)
    {
        commandDeltaTime[motorIndex] = UINT64_MAX;
        return false;
    }

    commandDone[motorIndex] = true;
    pulseStartPending[motorIndex] = false;
    pendingPulseToken[motorIndex] = 0;
    ESP_LOGW(
        TAG,
        "Pulse start failed terminally: motor=%u steps=%u rate=%u err=%s",
        static_cast<unsigned>(motorIndex + 1),
        static_cast<unsigned>(pulseConfig.pulseCount),
        static_cast<unsigned>(pulseConfig.startSpeedHz),
        esp_err_to_name(pulseErr));
    return false;
}

StepperDriver *StepperDriver::instance = nullptr;
static uint8_t peekTicks = 5;
static uint8_t peekRate = 5;

#define GPIO_STEP 16
#define GPIO_STEP_ENABLE 27
#define GPIO_STEP_DIR 13
#define GPIO_USTEP_MS1 12
#define GPIO_USTEP_MS2 14
#define GPIO_USTEP_MS3 2
#define MAXMICROSTEPS 32

bool const positiveDirection = true;
uint32_t direction = 0;
uint32_t DRAM_ATTR paused = 0;

uint16_t stepsPerRev = 200;
uint16_t mircoSteps = 1;
int32_t DRAM_ATTR location = 0;

/**
 * @brief Construct the stepper driver, initialize GPIO, and reset the pending command state.
 */
StepperDriver::StepperDriver() : MotorDriver()
{
    ESP_LOGV(TAG, "StepperDriver ctor");
    initMotorGpio();
    memset(commandDone, 1, MAX_STEPPER_MOTORS);
}

/**
 * @brief Return the singleton StepperDriver instance, creating it if necessary.
 * @return Pointer to the global StepperDriver.
 */
StepperDriver *IRAM_ATTR StepperDriver::getInstance()
{
    if (instance == nullptr)
    {
        instance = new StepperDriver();
    }

    return instance;
}

StepperDriver::MotionPlan StepperDriver::planRelativeMove(int32_t currentStep, int32_t deltaStep, uint16_t stepRate)
{
    MotionPlan plan = {};
    plan.goalStep = currentStep + deltaStep;
    plan.steps = steps_between(currentStep, plan.goalStep);
    if (plan.steps == 0) return plan;
    if (stepRate == 0)
    {
        plan.durationUs = UINT64_MAX;
        return plan;
    }
    plan.durationUs = (static_cast<uint64_t>(plan.steps) * 1000000ULL) / static_cast<uint64_t>(stepRate);
    return plan;
}

StepperDriver::MotionPlan StepperDriver::planAbsoluteMove(int32_t currentStep, int32_t targetStep, uint16_t stepRate)
{
    MotionPlan plan = {};
    plan.goalStep = targetStep;
    plan.steps = steps_between(currentStep, targetStep);
    if (plan.steps == 0) return plan;
    if (stepRate == 0)
    {
        plan.durationUs = UINT64_MAX;
        return plan;
    }
    plan.durationUs = (static_cast<uint64_t>(plan.steps) * 1000000ULL) / static_cast<uint64_t>(stepRate);
    return plan;
}

uint64_t StepperDriver::planDwellDurationUs(int32_t waitCycles, uint16_t cycleRateHz)
{
    if (cycleRateHz == 0) return 0;
    const int64_t cycles = waitCycles >= 0 ? static_cast<int64_t>(waitCycles) : -static_cast<int64_t>(waitCycles);
    return (static_cast<uint64_t>(cycles) * kMicrosecondsPerSecond) / static_cast<uint64_t>(cycleRateHz);
}

bool StepperDriver::shouldRejectForEndstop(char opcode, bool endstopTripped)
{
    return endstopTripped && (opcode == 'M' || opcode == 'G');
}

/**
 * @brief Configure the GPIO pins for the stepper interface and attach endstop interrupts.
 */
void StepperDriver::initMotorGpio()
{
    motorsControlled = 1;
    Endstop::configureGpioAndAttachIsr(gGpioIsrServiceInstalled);

    gpio_config_t outputConfig = {};
    outputConfig.pin_bit_mask =
        (1ULL << GPIO_STEP) |
        (1ULL << GPIO_STEP_ENABLE) |
        (1ULL << GPIO_STEP_DIR) |
        (1ULL << GPIO_USTEP_MS1) |
        (1ULL << GPIO_USTEP_MS2) |
        (1ULL << GPIO_USTEP_MS3);
    outputConfig.mode = GPIO_MODE_OUTPUT;
    outputConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    outputConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    outputConfig.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&outputConfig));

    gpio_set_level(static_cast<gpio_num_t>(GPIO_STEP_DIR), direction);
    gpio_set_level(static_cast<gpio_num_t>(GPIO_STEP), 0);
    gpio_set_level(static_cast<gpio_num_t>(GPIO_STEP_ENABLE), 0);
    gpio_set_level(static_cast<gpio_num_t>(GPIO_USTEP_MS1), 0);
    gpio_set_level(static_cast<gpio_num_t>(GPIO_USTEP_MS2), 0);
    gpio_set_level(static_cast<gpio_num_t>(GPIO_USTEP_MS3), 0);
    gpio_set_level(static_cast<gpio_num_t>(GPIO_STEP_DIR), 0);

    if (!kPulseInitOnDriverCore)
    {
        const esp_err_t pulseInitErr = PulseEngine::init(static_cast<gpio_num_t>(GPIO_STEP));
        ESP_ERROR_CHECK_WITHOUT_ABORT(pulseInitErr);
        if (pulseInitErr == ESP_OK)
            PulseEngine::registerCompletionCallback(&StepperDriver::onPulseRunComplete, this);
    }

    Endstop::primeStateFromPins();
}

/**
 * @brief Move a motor to an absolute angle at the requested rate, scheduling the required timeout.
 * @param targetAngle Absolute position goal in encoder units.
 * @param rate Requested speed for the motion.
 * @param motorID 1-based ID of the motor to command.
 */
void StepperDriver::motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    uint8_t motorIndex = 0;
    if (!tryResolveMotorIndex(motorID, motorsControlled, motorIndex)) return;
    const uint32_t opcodeSeq = consumeOpcodeContextSeq(motorIndex);
    if (sequence_is_stale(opcodeSeq, abortWatermarkSeq[motorIndex]))
    {
        commandDone[motorIndex] = true;
        pulseStartPending[motorIndex] = false;
        return;
    }
    activeOpcodeSeq[motorIndex] = opcodeSeq;

    if (shouldRejectForEndstop('G', isEndstopTripped(motorID)))
    {
        return;
    }

    const int32_t currentStep = static_cast<int32_t>(currentAngle[motorIndex]);
    const MotionPlan plan = planAbsoluteMove(currentStep, targetAngle, rate);

    if (currentStep > plan.goalStep)
    {
        direction = !positiveDirection;
    }
    else if (currentStep < plan.goalStep)
    {
        direction = positiveDirection;
    }
    gpio_set_level(static_cast<gpio_num_t>(GPIO_STEP_DIR), direction);

    if(motorSleeping){
        motorSleeping = false;
        setSleep(motorSleeping);
    }

    motorDwell = false;

    if (plan.steps > 0)
    {
        startAngle[motorIndex] = currentStep;
        commandDeltaAngle[motorIndex] = plan.goalStep;
        commandDone[motorIndex] = false;
        pendingPulseSteps[motorIndex] = plan.steps;
        pendingPulseRateHz[motorIndex] = rate;
        pendingPulseDurationUs[motorIndex] = plan.durationUs;
        pendingPulseToken[motorIndex] = opcodeSeq;
        pulseStartPending[motorIndex] = true;
        commandDeltaTime[motorIndex] = UINT64_MAX;
        (void)tryStartPendingPulse(motorIndex);
        if (plan.durationUs == UINT64_MAX)
        {
            ESP_LOGI(
                TAG,
                "Motion plan goto: motor=%u from=%ld to=%ld steps=%u rate=%u duration=unknown",
                static_cast<unsigned>(motorIndex + 1),
                static_cast<long>(currentStep),
                static_cast<long>(plan.goalStep),
                static_cast<unsigned>(plan.steps),
                static_cast<unsigned>(rate));
        }
        else
        {
            ESP_LOGI(
                TAG,
                "Motion plan goto: motor=%u from=%ld to=%ld steps=%u rate=%u duration_ms=%llu",
                static_cast<unsigned>(motorIndex + 1),
                static_cast<long>(currentStep),
                static_cast<long>(plan.goalStep),
                static_cast<unsigned>(plan.steps),
                static_cast<unsigned>(rate),
                static_cast<unsigned long long>(plan.durationUs / 1000ULL));
        }
    }
    else
    {
        commandDone[motorIndex] = true;
    }
}

/**
 * @brief Shift a motor by a relative angle from its current position using the target rate.
 * @param targetAngle Angle delta to apply to the motor.
 * @param rate Requested speed for the motion.
 * @param motorID 1-based ID of the motor to command.
 */
void StepperDriver::motorMove(int32_t deltaAngle, uint16_t rate, uint8_t motorID)
{
    uint8_t motorIndex = 0;
    if (!tryResolveMotorIndex(motorID, motorsControlled, motorIndex)) return;

    if (shouldRejectForEndstop('M', isEndstopTripped(motorID))) return;

    const int32_t currentStep = static_cast<int32_t>(currentAngle[motorIndex]);
    const int32_t goalStep = currentStep + deltaAngle;

    // Keep a single execution path for motion scheduling/timing by routing relative moves through goto.
    motorGoTo(goalStep, rate, motorID);
}

/**
 * @brief Hold the motor in place for a number of wait cycles before resuming.
 * @param wait_time Number of cycles to wait.
 * @param precision Wait cycles per second.
 * @param motorID 1-based ID of the motor to command.
 */
void StepperDriver::motorStop(signed int wait_time, unsigned short precision, uint8_t motorID)
{
    // wait_time, cycles to wait
    // precision, wait cycles per second
    uint8_t motorIndex = 0;
    if (!tryResolveMotorIndex(motorID, motorsControlled, motorIndex)) return;
    const uint32_t opcodeSeq = consumeOpcodeContextSeq(motorIndex);
    if (sequence_is_stale(opcodeSeq, abortWatermarkSeq[motorIndex]))
    {
        commandDone[motorIndex] = true;
        pulseStartPending[motorIndex] = false;
        return;
    }
    activeOpcodeSeq[motorIndex] = opcodeSeq;

    if (shouldRejectForEndstop('S', isEndstopTripped(motorID)))
    {
        return;
    }

    motorDwell = true;
    startTime[motorIndex] = esp_timer_get_time();
    const uint64_t dwellDurationUs = planDwellDurationUs(wait_time, precision);
    commandDeltaTime[motorIndex] = startTime[motorIndex] + dwellDurationUs;
    commandDone[motorIndex] = false;
    pulseStartPending[motorIndex] = false;
    pendingPulseToken[motorIndex] = 0;
    activePulseToken[motorIndex] = 0;
    
    if(motorSleeping){
        motorSleeping = false;
        setSleep(motorSleeping);
    }
	
    PulseEngine::stop();

}

/**
 * @brief Hold the motor, then transition the driver into sleep mode after the wait period.
 * @param wait_time Number of cycles to wait.
 * @param precision Wait cycles per second.
 * @param motorID 1-based ID of the motor to command.
 */
void StepperDriver::motorSleep(signed int wait_time, unsigned short precision, uint8_t motorID)
{
    // wait_time, cycles to wait
    // precision, wait cycles per second
    uint8_t motorIndex = 0;
    if (!tryResolveMotorIndex(motorID, motorsControlled, motorIndex)) return;
    const uint32_t opcodeSeq = consumeOpcodeContextSeq(motorIndex);
    if (sequence_is_stale(opcodeSeq, abortWatermarkSeq[motorIndex]))
    {
        commandDone[motorIndex] = true;
        pulseStartPending[motorIndex] = false;
        return;
    }
    activeOpcodeSeq[motorIndex] = opcodeSeq;

    if (shouldRejectForEndstop('I', isEndstopTripped(motorID)))
    {
        return;
    }

    motorDwell = true;
    startTime[motorIndex] = esp_timer_get_time();
    const uint64_t dwellDurationUs = planDwellDurationUs(wait_time, precision);
    commandDeltaTime[motorIndex] = startTime[motorIndex] + dwellDurationUs;
    commandDone[motorIndex] = false;
    pulseStartPending[motorIndex] = false;
    pendingPulseToken[motorIndex] = 0;
    activePulseToken[motorIndex] = 0;

    motorSleeping = true;
    setSleep(motorSleeping);

}

/**
 * @brief Immediately disable the step output and mark the command as completed.
 * @param motorID 1-based ID of the motor to abort.
 */
void StepperDriver::abortCommand(uint8_t motorID)
{
    uint8_t motorIndex = 0;
    if (!tryResolveMotorIndex(motorID, motorsControlled, motorIndex)) return;
    const uint32_t abortSeq = consumeOpcodeContextSeq(motorIndex);
    if (sequence_is_after(abortSeq, abortWatermarkSeq[motorIndex])) abortWatermarkSeq[motorIndex] = abortSeq;
    if (abortSeq == 0 && sequence_is_after(activeOpcodeSeq[motorIndex], abortWatermarkSeq[motorIndex]))
        abortWatermarkSeq[motorIndex] = activeOpcodeSeq[motorIndex];

    applyPulseProgress(PulseEngine::stop());
    pulseStartPending[motorIndex] = false;
    pendingPulseToken[motorIndex] = 0;
    activePulseToken[motorIndex] = 0;
    commandDone[motorIndex] = true;
}

void StepperDriver::setOpcodeContext(uint32_t op_seq, uint8_t motor_id)
{
    uint8_t motorIndex = 0;
    if (!tryResolveMotorIndex(motor_id, motorsControlled, motorIndex)) return;
    opcodeContextSeq[motorIndex] = op_seq;
}

/**
 * @brief Create a pinned FreeRTOS task to run the motor driver loop on the configured core.
 */
void StepperDriver::isrStartIoDriver()
{
    vTaskDelay(0);

    // Verified stable with IDF driver path and command processing.
    xTaskCreatePinnedToCore(
        isrIoStep,
        "motorloopstep",
        MOTOR_LOOP_STACK_BYTES,
        (void *)1,
        kMotorTaskPriority,
        &motorTaskDriver,
        kMotorTaskCore);
}

bool StepperDriver::isEndstopTripped(uint8_t motor_id)
{
    if (motor_id != 1)
    {
        return false;
    }
    return Endstop::isTripped();
}

bool IRAM_ATTR StepperDriver::isEndstopTripped()
{
    return StepperDriver::getInstance()->isEndstopTripped(1);
}


/**
 * @brief Stop the IO driver from stepping; currently just zeroes the step rate.
 */
void StepperDriver::isrStopIoDriver()
{
    applyPulseProgress(PulseEngine::stop());
}

/**
 * @brief Entry point for the motor IO task: initialize pulse counting then run the driver loop.
 * @param pvParameters Task parameters (unused).
 */
void StepperDriver::isrIoStep(void *pvParameters)
{
    (void)pvParameters;
    StepperDriver *driver = StepperDriver::getInstance();
    if (kPulseInitOnDriverCore)
    {
        const esp_err_t pulseInitErr = PulseEngine::init(static_cast<gpio_num_t>(GPIO_STEP));
        ESP_ERROR_CHECK_WITHOUT_ABORT(pulseInitErr);
        if (pulseInitErr == ESP_OK)
            PulseEngine::registerCompletionCallback(&StepperDriver::onPulseRunComplete, driver);
    }
    driver->driver();
}

void IRAM_ATTR StepperDriver::onPulseRunComplete(uint32_t pulsesCompleted, uint32_t runToken, void *userCtx)
{
    StepperDriver *driver = static_cast<StepperDriver *>(userCtx);
    if (driver == nullptr) return;
    if (runToken != driver->activePulseToken[0]) return;

    driver->applyPulseProgress(pulsesCompleted);
    driver->activePulseToken[0] = 0;
    driver->pulseStartPending[0] = false;
    driver->commandDone[0] = true;
}

void StepperDriver::stopActiveCommandForEndstop()
{
    if (motorDwell) return;

    bool stoppedAny = false;
    for (int i = 0; i < motorsControlled; ++i)
    {
        if (commandDone[i]) continue;

        applyPulseProgress(PulseEngine::stop());
        pulseStartPending[i] = false;
        pendingPulseToken[i] = 0;
        activePulseToken[i] = 0;
        commandDone[i] = true;
        stoppedAny = true;
    }

    if (!stoppedAny) return;
    motorSleeping = true;
    gpio_set_level(static_cast<gpio_num_t>(GPIO_STEP_ENABLE), 1);
    ESP_LOGW(TAG, "Command: Stopped by endstop");
}

void StepperDriver::applyPulseProgress(uint32_t pulsesCompleted)
{
    const int32_t startStep = static_cast<int32_t>(startAngle[0]);
    const int32_t signedDelta = direction == positiveDirection ? static_cast<int32_t>(pulsesCompleted) : -static_cast<int32_t>(pulsesCompleted);
    location = startStep + signedDelta;
    currentAngle[0] = location;
}

/**
 * @brief Report whether the requested motor still has an outstanding command.
 * @param motor_id 1-based ID of the motor.
 * @return True if the motor is still executing a command.
 */
bool StepperDriver::isMotorRunning(uint8_t motorID)
{
    uint8_t motorIndex = 0;
    if (!tryResolveMotorIndex(motorID, motorsControlled, motorIndex)) return false;
    return !commandDone[motorIndex];
}

/**
 * @brief Loop that monitors command timeouts, polls for new operations, and keeps the motors fed.
 */
void IRAM_ATTR StepperDriver::driver()
{
    while (true)
    {
        if (peekTicks == 0)
        {
            peekTicks = peekRate;
        }
        peekTicks--;

        for (int i = 0; (i < motorsControlled); i++)
        {
            if (!commandDone[i])
            {
                if (pulseStartPending[i])
                {
                    (void)tryStartPendingPulse(static_cast<uint8_t>(i));
                    if (pulseStartPending[i])
                    {
                        if (peekTicks == 0)
                        {
                            peekOpForDriver(i);
                        }
                        continue;
                    }
                }

                PulseEngine::service();
                if (commandDone[i]) continue;

                if (!motorDwell && isEndstopTripped(static_cast<uint8_t>(i + 1)))
                {
                    stopActiveCommandForEndstop();
                    continue;
                }

                const uint64_t nowUs = static_cast<uint64_t>(esp_timer_get_time());
                if (nowUs >= commandDeltaTime[i])
                {
                    if (motorDwell)
                    {
                    }
                    else
                    {
                        const uint64_t scheduledWindowUs =
                            commandDeltaTime[i] > startTime[i] ? (commandDeltaTime[i] - startTime[i]) : 0;
                        const uint64_t graceUs = timeout_grace_us(scheduledWindowUs);
                        const uint64_t overdueUs = nowUs - commandDeltaTime[i];

                        // RMT completion can lag a planned deadline by frame refill overhead; only force-stop after a grace window.
                        if (PulseEngine::isRunning() && overdueUs < graceUs)
                        {
                            continue;
                        }

                        const uint32_t pulsesCompleted = PulseEngine::stop();
                        applyPulseProgress(pulsesCompleted);
                        activePulseToken[i] = 0;
                        ESP_LOGE(TAG, "Command: Timed out");
                        ESP_LOGE(
                            TAG,
                            "command: motorMove %d %d %u ",
                            static_cast<int>(startAngle[i]),
                            static_cast<int>(commandDeltaAngle[i]),
                            static_cast<unsigned int>(startTime[i]));
                        // #region FIXME(STEPPER-MISSED-STEP-TRACE): Temporary timeout diagnostics to quantify commanded-vs-completed pulse gap under load.
                        const uint32_t pulsesRequested =
                            steps_between(static_cast<int32_t>(startAngle[i]), static_cast<int32_t>(commandDeltaAngle[i]));
                        const uint32_t pulsesMissing = pulsesRequested > pulsesCompleted ? pulsesRequested - pulsesCompleted : 0;
                        if (pulsesMissing > 0)
                            ESP_LOGW(
                                TAG,
                                "missed-step detect: requested=%u completed=%u missing=%u elapsed_us=%llu deadline_us=%llu",
                                static_cast<unsigned>(pulsesRequested),
                                static_cast<unsigned>(pulsesCompleted),
                                static_cast<unsigned>(pulsesMissing),
                                static_cast<unsigned long long>(esp_timer_get_time() - startTime[i]),
                                static_cast<unsigned long long>(commandDeltaTime[i] - startTime[i]));
                        // #endregion
                    }
                    commandDone[i] = true;
                    ESP_LOGV(TAG, "Command: Done");
                }

                if (peekTicks == 0)
                {
                    peekOpForDriver(i);
                }
            }
            else
            {
                getNextOpForDriver(i);
            }
        }
        // Always yield one tick so IDLE1 can service the task watchdog.
        // TODO(STEPPER-DRIVER-EVENT-LOOP): Replace polling + tick delay with event-driven wake (queue/task notification).
        vTaskDelay(1);
    }
    vTaskDelete(nullptr);
}

uint32_t StepperDriver::consumeOpcodeContextSeq(uint8_t motorIndex)
{
    const uint32_t seq = opcodeContextSeq[motorIndex];
    opcodeContextSeq[motorIndex] = 0;
    return seq;
}

/**
 * @brief Request the next queued operation for the specified motor from the command layer.
 * @param id Zero-based motor index.
 */
void StepperDriver::getNextOpForDriver(uint8_t id)
{
    CommandLayer::getInstance()->getNextOp(id + 1);
}

/**
 * @brief Ask the command layer to peek at the next operation without consuming it.
 * @param id Zero-based motor index.
 */
void StepperDriver::peekOpForDriver(uint8_t id)
{
    CommandLayer::getInstance()->peekNextOp(id + 1);
}

/**
 * @brief Apply motor-specific configuration changes such as microstepping modes.
 * @param setting Configuration enum identifying the change.
 * @param data1 Primary configuration value.
 * @param data2 Secondary configuration value (unused for microstepping).
 * @param motorID 1-based ID of the motor.
 */
void StepperDriver::changeMotorSettings(config_setting setting, uint32_t data1, uint32_t data2, uint8_t motorID)
{
    uint8_t motorIndex = 0;
    if (!tryResolveMotorIndex(motorID, motorsControlled, motorIndex)) return;

    if (setting == config_setting::MICROSTEPPING)
    {
        gpio_set_level(static_cast<gpio_num_t>(GPIO_USTEP_MS2), data1 > 0);
        gpio_set_level(static_cast<gpio_num_t>(GPIO_USTEP_MS1), data1 > 0);
        ESP_LOGI(TAG, "changeMotorSettings %d %d %d ", data1, data2, motorIndex);
    }
}

/**
 * @brief Toggle the sleep line on the driver, ensuring stepping is disabled first.
 * @param sleep True to assert sleep (disable driver), false to wake.
 */
void StepperDriver::setSleep(bool sleep)
{
    if(sleep){
        PulseEngine::stop();
        gpio_set_level(static_cast<gpio_num_t>(GPIO_STEP_ENABLE), 1);
    }
    else
    {
        gpio_set_level(static_cast<gpio_num_t>(GPIO_STEP_ENABLE), 0);
    }

}

/**
 * @brief Return the configured logic level that represents an engaged endstop.
 * @return True if a high pin state indicates a tripped endstop.
 */
bool StepperDriver::getEndstopTrippedPinSetting()
{
    return StepperDriver::getInstance()->getEndstopTrippedPinSetting(1);
}

/**
 * @brief Set whether a high or low pin state indicates that an endstop is tripped.
 * @param setting Boolean value interpreted as the tripped pin level.
 * @return ESP_OK on success or ESP_ERR_INVALID_ARG for bad settings.
 */
esp_err_t StepperDriver::setEndstopTrippedPinSetting(uint8_t setting) 
{
    return StepperDriver::getInstance()->setEndstopTrippedPinSetting(setting, 1);
}

bool StepperDriver::getEndstopTrippedPinSetting(uint8_t motor_id)
{
    if (motor_id != 1)
    {
        return false;
    }

    return Endstop::getTrippedPinSetting();
}

esp_err_t StepperDriver::setEndstopTrippedPinSetting(uint8_t setting, uint8_t motor_id)
{
    if (motor_id != 1)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return Endstop::setTrippedPinSetting(setting);
}


/* 
void StepperDriver::addSteps(uint32_t steps)
{
    addSteps(steps, getMicroStepRate());
}

void StepperDriver::addSteps(uint32_t steps, uint16_t microstepRate)
{
    //
}

uint16_t StepperDriver::getMicroStepRate()
{
    return mircoSteps;
}

void StepperDriver::setMicroStepRate(uint16_t microstepRate)
{
    mircoSteps = microstepRate;
}

void StepperDriver::setMicroStepRate(bool MS1, bool MS2, bool MS3)
{
    uint8_t msMask = MS3 << 2 | MS2 << 1 | MS1;

    switch (msMask)
    {
    case 0x00:
    case 0x04:
        mircoSteps = 1; // Full Step
        break;
    case 0x01:
    case 0x05:
        mircoSteps = 2; // Half Step
        break;
    case 0x02:
        mircoSteps = 16; // Sixteenth Step
        break;
    case 0x03:
        mircoSteps = 32; // Thirty-seconth Step
        break;
    case 0x06:
        mircoSteps = 4; // Quarter Step
        break;
    case 0x07:
        mircoSteps = 8; // Eighth Step
        break;
    }
} */
