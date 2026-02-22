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
} // namespace

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

//end stop
#define GPIO_IO_A 21
#define GPIO_IO_B 22
#define MYR_DEFAULT_DEBOUNCE_MS 10 // TODO: NVS config

bool const positiveDirection = true;
uint32_t direction = 0;
uint32_t DRAM_ATTR paused = 0;


bool DRAM_ATTR                  isEndstopTrippedHigh        = false; // Value of endstop when it is engaged.
uint32_t DRAM_ATTR static       debounceTimeMs              = MYR_DEFAULT_DEBOUNCE_MS;    // millis // TODO: NVS config

static DRAM_ATTR portMUX_TYPE   endstopAMux                 = portMUX_INITIALIZER_UNLOCKED;
bool DRAM_ATTR static           isEndstopA_ActiveNow        = false;
uint32_t DRAM_ATTR volatile     numberOfEndstopAIsr         = 0;
bool DRAM_ATTR                  lastStateEndstopAIsr        = 0;
uint32_t DRAM_ATTR volatile     debounceTimeoutEndstopAIsr  = 0;

static DRAM_ATTR portMUX_TYPE   endstopBMux                 = portMUX_INITIALIZER_UNLOCKED;
bool DRAM_ATTR static           isEndstopB_ActiveNow        = false;
uint32_t DRAM_ATTR volatile     numberOfEndstopBIsr         = 0;
bool DRAM_ATTR volatile         lastStateEndstopBIsr        = 0;
uint32_t DRAM_ATTR volatile     debounceTimeoutEndstopBIsr  = 0;

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
    gpio_config_t ioAConfig = {};
    ioAConfig.pin_bit_mask = (1ULL << GPIO_IO_A);
    ioAConfig.mode = GPIO_MODE_INPUT;
    ioAConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    ioAConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ioAConfig.intr_type = GPIO_INTR_ANYEDGE;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&ioAConfig));

    gpio_config_t ioBConfig = {};
    ioBConfig.pin_bit_mask = (1ULL << GPIO_IO_B);
    ioBConfig.mode = GPIO_MODE_INPUT;
    ioBConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    ioBConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ioBConfig.intr_type = GPIO_INTR_ANYEDGE;
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&ioBConfig));

    if (!gGpioIsrServiceInstalled) {
        esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
            gGpioIsrServiceInstalled = true;
        } else {
            ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        }
    }
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_add(static_cast<gpio_num_t>(GPIO_IO_A), &endstop_a_interrupt, nullptr));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_isr_handler_add(static_cast<gpio_num_t>(GPIO_IO_B), &endstop_b_interrupt, nullptr));

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

    endstop_a_interrupt(nullptr);
    endstop_b_interrupt(nullptr);
}

/**
 * @brief Move a motor to an absolute angle at the requested rate, scheduling the required timeout.
 * @param targetAngle Absolute position goal in encoder units.
 * @param rate Requested speed for the motion.
 * @param motorID 1-based ID of the motor to command.
 */
void StepperDriver::motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    if (motorID > motorsControlled)
        return;
    motorID--;

    if (shouldRejectForEndstop('G', isEndstopTripped()))
    {
        return;
    }

    const int32_t currentStep = static_cast<int32_t>(currentAngle[motorID]);
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
        startAngle[motorID] = currentStep;
        commandDeltaAngle[motorID] = plan.goalStep;
        startTime[motorID] = esp_timer_get_time();
        commandDeltaTime[motorID] =
            plan.durationUs == UINT64_MAX ? UINT64_MAX : startTime[motorID] + plan.durationUs + kCommandTimingMarginUs;
        commandDone[motorID] = false;

        PulseEngine::StartConfig pulseConfig = {};
        pulseConfig.pulseCount = plan.steps;
        pulseConfig.startSpeedHz = rate;
        pulseConfig.endSpeedHz = rate;
        esp_err_t pulseErr = PulseEngine::startPulses(pulseConfig);
        if (pulseErr != ESP_OK)
        {
            commandDone[motorID] = true;
            ESP_LOGW(
                TAG,
                "Pulse start failed: motor=%u steps=%u start=%u end=%u err=%s",
                static_cast<unsigned>(motorID + 1),
                static_cast<unsigned>(pulseConfig.pulseCount),
                static_cast<unsigned>(pulseConfig.startSpeedHz),
                static_cast<unsigned>(pulseConfig.endSpeedHz),
                esp_err_to_name(pulseErr));
            return;
        }
        if (plan.durationUs == UINT64_MAX)
        {
            ESP_LOGI(
                TAG,
                "Motion plan goto: motor=%u from=%ld to=%ld steps=%u rate=%u duration=unknown",
                static_cast<unsigned>(motorID + 1),
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
                static_cast<unsigned>(motorID + 1),
                static_cast<long>(currentStep),
                static_cast<long>(plan.goalStep),
                static_cast<unsigned>(plan.steps),
                static_cast<unsigned>(rate),
                static_cast<unsigned long long>(plan.durationUs / 1000ULL));
        }
    }
    else
    {
        commandDone[motorID] = true;
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
    if (motorID > motorsControlled)
        return;
    if (shouldRejectForEndstop('M', isEndstopTripped())) return;

    const uint8_t motorIndex = motorID - 1;
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
    if (motorID > motorsControlled)
        return;
    motorID--;

    if (shouldRejectForEndstop('S', isEndstopTripped()))
    {
        return;
    }

    motorDwell = true;
    startTime[motorID] = esp_timer_get_time();
    const uint64_t dwellDurationUs = planDwellDurationUs(wait_time, precision);
    commandDeltaTime[motorID] = startTime[motorID] + dwellDurationUs;
    commandDone[motorID] = false;
    
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
    if (motorID > motorsControlled)
        return;
    motorID--;

    if (shouldRejectForEndstop('I', isEndstopTripped()))
    {
        return;
    }

    motorDwell = true;
    startTime[motorID] = esp_timer_get_time();
    const uint64_t dwellDurationUs = planDwellDurationUs(wait_time, precision);
    commandDeltaTime[motorID] = startTime[motorID] + dwellDurationUs;
    commandDone[motorID] = false;

    motorSleeping = true;
    setSleep(motorSleeping);

}

/**
 * @brief Immediately disable the step output and mark the command as completed.
 * @param motorID 1-based ID of the motor to abort.
 */
void StepperDriver::abortCommand(uint8_t motorID)
{
    if (motorID > motorsControlled)
        return;
    motorID--;

    applyPulseProgress(PulseEngine::stop());
    commandDone[motorID] = true;
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

/**
 * @brief ISR that records the latest state and debounce timestamp for endstop A.
 */
void IRAM_ATTR StepperDriver::endstop_a_interrupt(void *arg)
{
    (void)arg;
    //endstopMux
    portENTER_CRITICAL_ISR(&endstopAMux);
    numberOfEndstopAIsr = numberOfEndstopAIsr + 1;
    lastStateEndstopAIsr = gpio_get_level(static_cast<gpio_num_t>(GPIO_IO_A));
    debounceTimeoutEndstopAIsr = xTaskGetTickCountFromISR();
    portEXIT_CRITICAL_ISR(&endstopAMux);
}

/**
 * @brief ISR that records the latest state and debounce timestamp for endstop B.
 */
void IRAM_ATTR StepperDriver::endstop_b_interrupt(void *arg)
{
    (void)arg;
    portENTER_CRITICAL_ISR(&endstopBMux);
    numberOfEndstopBIsr = numberOfEndstopBIsr + 1;
    lastStateEndstopBIsr = gpio_get_level(static_cast<gpio_num_t>(GPIO_IO_B));
    debounceTimeoutEndstopBIsr = xTaskGetTickCountFromISR();
    portEXIT_CRITICAL_ISR(&endstopBMux);
}

/**
 * @brief Debounce both endstops and return true if either is currently tripped.
 * @return True when a stable endstop engagement has been detected.
 */
bool IRAM_ATTR StepperDriver::isEndstopTripped()
{
    uint32_t saveDebounceTimeout;
    bool saveLastState;
    uint32_t hasChanged;
    bool currentState = false;
    bool retunValue = false;

    //endstopA
    portENTER_CRITICAL_ISR(&endstopAMux);
    hasChanged  = numberOfEndstopAIsr;
    saveDebounceTimeout = debounceTimeoutEndstopAIsr;
    saveLastState  = lastStateEndstopAIsr;
    portEXIT_CRITICAL_ISR(&endstopAMux);
    
    currentState = gpio_get_level(static_cast<gpio_num_t>(GPIO_IO_A));

    // if Interrupt Has triggered AND pin is in same state AND the debounce time has expired THEN endstop is stable
    if ((hasChanged != 0) &&
        (currentState == saveLastState) &&
        ((xTaskGetTickCount() - saveDebounceTimeout) > pdMS_TO_TICKS(debounceTimeMs)))
    { 
        portENTER_CRITICAL_ISR(&endstopAMux);
        numberOfEndstopAIsr = 0; // clear counter
        portEXIT_CRITICAL_ISR(&endstopAMux);
        
        if (currentState == isEndstopTrippedHigh)
        {
            isEndstopA_ActiveNow = true;
        }
        else
        {
            isEndstopA_ActiveNow = false;
        }
    }

    retunValue |= isEndstopA_ActiveNow;

    //endstopB
    portENTER_CRITICAL_ISR(&endstopBMux);
    hasChanged  = numberOfEndstopBIsr;
    saveDebounceTimeout = debounceTimeoutEndstopBIsr;
    saveLastState  = lastStateEndstopBIsr;
    portEXIT_CRITICAL_ISR(&endstopBMux);
    
    currentState = gpio_get_level(static_cast<gpio_num_t>(GPIO_IO_B));

    // if Interrupt Has triggered AND pin is in same state AND the debounce time has expired THEN endstop is stable
    if ((hasChanged != 0) &&
        (currentState == saveLastState) &&
        ((xTaskGetTickCount() - saveDebounceTimeout) > pdMS_TO_TICKS(debounceTimeMs)))
    { 
        portENTER_CRITICAL_ISR(&endstopBMux);
        numberOfEndstopBIsr = 0; // clear counter
        portEXIT_CRITICAL_ISR(&endstopBMux);
      
        if (currentState == isEndstopTrippedHigh)
        {
            isEndstopB_ActiveNow = true;
        }
        else
        {
            isEndstopB_ActiveNow = false;
        }
    }

    retunValue |= isEndstopB_ActiveNow;
    
    return retunValue;
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

void IRAM_ATTR StepperDriver::onPulseRunComplete(uint32_t pulsesCompleted, void *userCtx)
{
    StepperDriver *driver = static_cast<StepperDriver *>(userCtx);
    if (driver == nullptr) return;

    driver->applyPulseProgress(pulsesCompleted);
    driver->commandDone[0] = true;
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
bool StepperDriver::isMotorRunning(uint8_t motor_id)
{
    if (motor_id > motorsControlled)
        return false;
    motor_id--;
    return !commandDone[motor_id];
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
                PulseEngine::service();
                if (commandDone[i]) continue;

                if (!motorDwell && isEndstopTripped())
                {
                    applyPulseProgress(PulseEngine::stop());
                    commandDone[i] = true;
                    ESP_LOGW(TAG, "Command: Stopped by endstop");
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
        // TODO(STEPPER-DRIVER-EVENT-LOOP): Replace polling + tick delay with event-driven wake (queue/task notification).
        vTaskDelay(1);
    }
    vTaskDelete(nullptr);
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
    if (motorID > motorsControlled)
        return;
    motorID--;

    if (setting == config_setting::MICROSTEPPING)
    {
        gpio_set_level(static_cast<gpio_num_t>(GPIO_USTEP_MS2), data1 > 0);
        gpio_set_level(static_cast<gpio_num_t>(GPIO_USTEP_MS1), data1 > 0);
        ESP_LOGI(TAG, "changeMotorSettings %d %d %d ", data1, data2, motorID);
    }
}

/**
 * @brief Toggle the sleep line on the driver, ensuring stepping is disabled first.
 * @param sleep True to assert sleep (disable driver), false to wake.
 */
void StepperDriver::setSleep(bool sleep)
{
    PulseEngine::stop();

    if(sleep){
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
    return isEndstopTrippedHigh;

}

/**
 * @brief Set whether a high or low pin state indicates that an endstop is tripped.
 * @param setting Boolean value interpreted as the tripped pin level.
 * @return ESP_OK on success or ESP_ERR_INVALID_ARG for bad settings.
 */
esp_err_t StepperDriver::setEndstopTrippedPinSetting(uint8_t setting) 
{
    esp_err_t error = ESP_OK;
    if (setting <= 1)
    {
        isEndstopTrippedHigh = static_cast<bool>(setting);
        // TODO: Add endstop update logic?
    } else {
        return ESP_ERR_INVALID_ARG;
    }
    
    return error;
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
