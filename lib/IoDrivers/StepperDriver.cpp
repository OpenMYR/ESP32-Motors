/**
 * @file StepperDriver.cpp
 * @brief Implements the singleton stepper motor driver.
 */
#include <reent.h>
#include <math.h>
#include <ESP32PWM.h>
#include <esp32-hal-ledc.h>
#include <driver/periph_ctrl.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/pcnt.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <soc/gpio_sig_map.h>
#include <soc/pcnt_struct.h>
#include <soc/pcnt_reg.h>

#include "StepperDriver.h"
#include "OpBuffer.h"

#define CORE_1 1
#define UPDATE_FREQ 1000

#define UPDATE_DWELL 1000 / UPDATE_FREQ
#define motorInterfaceType 1

//static hw_timer_t *timerDriver = NULL;
StepperDriver *StepperDriver::instance = NULL;
//static CommandLayer *commandInstance = NULL;
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

#define PCNT_UNIT PCNT_UNIT_0
#define PCNT_H_LIM_VAL 100
#define PCNT_L_LIM_VAL -100
#define PCNT_THRESH1_VAL 1
#define PCNT_THRESH0_VAL -1

bool const positiveDirection = true;
uint32_t direction = 0;
uint32_t DRAM_ATTR paused = 0;
uint32_t DRAM_ATTR command_done = 1;


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
ESP32PWM pwm;
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

/**
 * @brief Construct the stepper driver, initialize GPIO, and reset the pending command state.
 */
StepperDriver::StepperDriver() : MotorDriver()
{
    Serial.write("StepperDriver const\n");
    initMotorGpio();
    memset(commandDone, 1, MAX_STEPPER_MOTORS);
}

/**
 * @brief Return the singleton StepperDriver instance, creating it if necessary.
 * @return Pointer to the global StepperDriver.
 */
StepperDriver *IRAM_ATTR StepperDriver::getInstance()
{
    if (instance == NULL)
    {
        instance = new StepperDriver();
    }

    return instance;
}

/**
 * @brief Configure the GPIO pins for the stepper interface and attach endstop interrupts.
 */
void StepperDriver::initMotorGpio()
{
    motorsControlled = 1;
    pinMode(GPIO_IO_A, INPUT_PULLUP);
    pinMode(GPIO_IO_B, INPUT_PULLUP);
    attachInterrupt(GPIO_IO_A, endstop_a_interrupt, CHANGE);
    attachInterrupt(GPIO_IO_B, endstop_b_interrupt, CHANGE);
    pinMode(GPIO_STEP_DIR, OUTPUT);
    digitalWrite(GPIO_STEP_DIR, direction);
    pinMode(GPIO_STEP, OUTPUT);
    digitalWrite(GPIO_STEP, LOW);
    pinMode(GPIO_STEP_ENABLE, OUTPUT);
    digitalWrite(GPIO_STEP_ENABLE, LOW);
    pinMode(GPIO_USTEP_MS1, OUTPUT);
    digitalWrite(GPIO_USTEP_MS1, LOW);
    pinMode(GPIO_USTEP_MS2, OUTPUT);
    digitalWrite(GPIO_USTEP_MS2, LOW);
    pinMode(GPIO_USTEP_MS3, OUTPUT);
    digitalWrite(GPIO_USTEP_MS3, LOW);
    pinMode(GPIO_STEP_DIR, OUTPUT);
    digitalWrite(GPIO_STEP_DIR, LOW);

    endstop_a_interrupt();
    endstop_b_interrupt();
}

/**
 * @brief Move a motor to an absolute angle at the requested rate, scheduling the required timeout.
 * @param targetAngle Absolute position goal in encoder units.
 * @param rate Requested speed for the motion.
 * @param motorID 1-based ID of the motor to command.
 */
void StepperDriver::motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    //double goToDegrees = (double)targetAngle;
    if (motorID > motorsControlled)
        return;
    motorID--; //motors are 1-15, we want 0-14

    if (isEndstopTripped())
    {
        return;
    }

    uint32_t limit = 0;

    if (currentAngle[motorID] > targetAngle)
    {
        limit = (currentAngle[motorID] - targetAngle);
        direction = !positiveDirection;
    }
    else if (currentAngle[motorID] < targetAngle)
    {
        limit = (targetAngle - currentAngle[motorID]);
        direction = positiveDirection;
    }
    digitalWrite(GPIO_STEP_DIR, direction);

    if(motorSleeping){
        motorSleeping = false;
        setSleep(motorSleeping);
    }
    
    motorDwell = false;

    if (limit > 0)
    {
        startAngle[motorID] = currentAngle[motorID];
        commandDeltaAngle[motorID] = targetAngle;
        startTime[motorID] = esp_timer_get_time();
        if (rate > 0)
        {
            uint64_t duration_us = (uint64_t)(fabs(commandDeltaAngle[motorID]) * 1000000.0 / (double)rate);
            commandDeltaTime[motorID] = startTime[motorID] + duration_us + 100; // small safety margin
        }
        else
        {
            commandDeltaTime[motorID] = UINT64_MAX; // no timeout if rate is zero/invalid
        }
        commandDone[motorID] = false;

        setStepRate(rate);
    }
    else
    {
        commandDone[motorID] = true;
    }

    log_d("command: motorGoTo %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
}

/**
 * @brief Shift a motor by a relative angle from its current position using the target rate.
 * @param targetAngle Angle delta to apply to the motor.
 * @param rate Requested speed for the motion.
 * @param motorID 1-based ID of the motor to command.
 */
void StepperDriver::motorMove(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    if (motorID > motorsControlled)
        return;
    motorID--;

    if (isEndstopTripped())
    {
        return;
    }

    uint32_t limit = abs(targetAngle);
    direction = targetAngle > 0 ? positiveDirection : !positiveDirection;
    digitalWrite(GPIO_STEP_DIR, direction);
	    
    if(motorSleeping){
        motorSleeping = false;
        setSleep(motorSleeping);
    }

    motorDwell = false;

    if (limit != 0)
    {
        startAngle[motorID] = currentAngle[motorID];
        commandDeltaAngle[motorID] = targetAngle + currentAngle[motorID];
        startTime[motorID] = esp_timer_get_time();
        if (rate > 0)
        {
            uint64_t duration_us = (uint64_t)(fabs(commandDeltaAngle[motorID]) * 1000000.0 / (double)rate);
            commandDeltaTime[motorID] = startTime[motorID] + duration_us + 100; // small safety margin
        }
        else
        {
            commandDeltaTime[motorID] = UINT64_MAX; // no timeout if rate is zero/invalid
        }
        commandDone[motorID] = false;

        setStepRate(rate);
    }
    else
    {
        commandDone[motorID] = true;
    }

    log_d("command: motorMove %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
}

/**
 * @brief Hold the motor in place for a number of wait cycles before resuming.
 * @param wait_time Number of cycles to wait.
 * @param precision Duration of each wait cycle in milliseconds.
 * @param motorID 1-based ID of the motor to command.
 */
void StepperDriver::motorStop(signed int wait_time, unsigned short precision, uint8_t motorID)
{
    // wait_time, cycles to wait
    // precision, duration of wait cycle in milliseconds
    if (motorID > motorsControlled)
        return;
    motorID--;

    if (isEndstopTripped())
    {
        return;
    }

    motorDwell = true;
    startTime[0] = esp_timer_get_time();
    commandDeltaTime[motorID] = startTime[0] + (uint64_t)abs(wait_time) * (uint64_t)precision;
    commandDone[motorID] = false;
    
    if(motorSleeping){
        motorSleeping = false;
        setSleep(motorSleeping);
    }
	
    setStepRate(0);

    log_d("command %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
}

/**
 * @brief Hold the motor, then transition the driver into sleep mode after the wait period.
 * @param wait_time Number of cycles to wait.
 * @param precision Duration of each wait cycle in milliseconds.
 * @param motorID 1-based ID of the motor to command.
 */
void StepperDriver::motorSleep(signed int wait_time, unsigned short precision, uint8_t motorID)
{
    // wait_time, cycles to wait
    // precision, duration of wait cycle in milliseconds
    if (motorID > motorsControlled)
        return;
    motorID--;

    if (isEndstopTripped())
    {
        return;
    }

    motorDwell = true;
    startTime[0] = esp_timer_get_time();
    commandDeltaTime[motorID] = startTime[0] + (abs(wait_time) * precision);
    commandDone[motorID] = false;

    setSleep(true);
    motorSleeping = true;
    setSleep(motorSleeping);

    log_d("command %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
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

    setStepRate(0);
    commandDone[motorID] = true;
}

/**
 * @brief Create a pinned FreeRTOS task to run the motor driver loop on CORE_1.
 */
void StepperDriver::isrStartIoDriver()
{
    vTaskDelay(0);

    xTaskCreatePinnedToCore(
        isrIoStep,
        "motorloopstep",
        2000,
        (void *)1,
        0,
        &motorTaskDriver,
        CORE_1);
}

/**
 * @brief ISR that records the latest state and debounce timestamp for endstop A.
 */
void IRAM_ATTR StepperDriver::endstop_a_interrupt()
{
    //endstopMux
    portENTER_CRITICAL_ISR(&endstopAMux);
    numberOfEndstopAIsr++;
    lastStateEndstopAIsr = digitalRead(GPIO_IO_A);
    debounceTimeoutEndstopAIsr = xTaskGetTickCount(); // ISR safe version of millis() 
    portEXIT_CRITICAL_ISR(&endstopAMux);
}

/**
 * @brief ISR that records the latest state and debounce timestamp for endstop B.
 */
void IRAM_ATTR StepperDriver::endstop_b_interrupt()
{
    portENTER_CRITICAL_ISR(&endstopBMux);
    numberOfEndstopBIsr++;
    lastStateEndstopBIsr = digitalRead(GPIO_IO_B);
    debounceTimeoutEndstopBIsr = xTaskGetTickCount(); // ISR safe version of millis() 
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
    
    currentState = digitalRead(GPIO_IO_A);

    // if Interrupt Has triggered AND pin is in same state AND the debounce time has expired THEN endstop is stable
    if ((hasChanged != 0) && (currentState == saveLastState) && (millis() - saveDebounceTimeout > debounceTimeMs ))
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
    
    currentState = digitalRead(GPIO_IO_B);

    // if Interrupt Has triggered AND pin is in same state AND the debounce time has expired THEN endstop is stable
    if ((hasChanged != 0) && (currentState == saveLastState) && (millis() - saveDebounceTimeout > debounceTimeMs ))
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
    setStepRate(0); // TODO: clean me
}

/**
 * @brief Entry point for the motor IO task: initialize pulse counting then run the driver loop.
 * @param pvParameters Task parameters (unused).
 */
void StepperDriver::isrIoStep(void *pvParameters)
{
    pcnt_setup_init(GPIO_STEP);
    StepperDriver::getInstance()->driver();
}

/**
 * @brief Compare the tracked position against the current command and stop when the target is reached.
 */
void StepperDriver::checkLocation()
{
    currentAngle[0] = location;
    if (currentAngle[0] == commandDeltaAngle[0])
    {
        setStepRate(0);
        commandDone[0] = true;
    }
    else if (isEndstopTripped())
    {
        setStepRate(0);
        commandDone[0] = true;
    }
}

/**
 * @brief PCNT interrupt that updates the step count based on edge thresholds and reevaluates location.
 * @param arg Unused ISR argument attached during registration.
 */
void IRAM_ATTR StepperDriver::pcnt_intr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (int i = 0; i < PCNT_UNIT_MAX; i++)
    {
        if (intr_status & (BIT(i)))
        {
            if (i == PCNT_UNIT)
            {
                if (PCNT.status_unit[i].val == PCNT_STATUS_THRES1_M)
                {
                    if (direction == positiveDirection)
                    {
                        location++;
                    }
                    else
                    {
                        location--;
                    }
                }
                if (PCNT.status_unit[i].val == PCNT_STATUS_THRES0_M)
                {
                    if (direction == positiveDirection)
                    {
                        location++;
                    }
                    else
                    {
                        location--;
                    }
                }
                pcnt_counter_clear(PCNT_UNIT);
            }

            PCNT.int_clr.val = BIT(i);

            if (HPTaskAwoken == pdTRUE)
            {
                portYIELD_FROM_ISR();
            }
            StepperDriver::getInstance()->checkLocation();
        }
    }
}

/**
 * @brief Configure the Pulse Counter (PCNT) hardware for the step pin and enable interrupts.
 * @param pin GPIO pin that supplies the step pulses.
 */
void StepperDriver::pcnt_setup_init(uint8_t pin)
{

    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = pin,
        .ctrl_gpio_num = -1,
        .lctrl_mode = PCNT_MODE_REVERSE, // No Control Pin Configured
        .hctrl_mode = PCNT_MODE_KEEP,    // No Control Pin Configured
        .pos_mode = PCNT_COUNT_INC,      // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,      // Keep the counter value on the negative edge
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
        .unit = PCNT_UNIT,
        .channel = PCNT_CHANNEL_0,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_UNIT, 100);
    pcnt_filter_disable(PCNT_UNIT);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_1);
    pcnt_set_event_value(PCNT_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_THRES_0);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_UNIT);
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
                if (esp_timer_get_time() >= commandDeltaTime[i])
                {
                    if (motorDwell)
                    {
                        // No issue
                    }
                    else
                    {
                        log_e("Command: Timed out");
                        log_e("command: motorMove %d %d %u ", (int)startAngle[i], (int)commandDeltaAngle[i], (uint)startTime[i]);
                    }
                    commandDone[i] = true;
                    log_v("Command: Done");
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
        vTaskDelay(10);
    }
    vTaskDelete(NULL);
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
        digitalWrite(GPIO_USTEP_MS2, data1 > 0);
        digitalWrite(GPIO_USTEP_MS1, data1 > 0);
        log_i("changeMotorSettings %d %d %d ", data1, data2, motorID);
    }
}

/**
 * @brief Configure or disable the PWM driving the step pin based on the desired rate.
 * @param rate Frequency to apply to the step pin; zero disables the PWM.
 */
void StepperDriver::setStepRate(int32_t rate)
{
    // todo document this bug
    #ifndef UNITY_INCLUDE_CONFIG_H // ESP32PWM attachPin causes Unit Tests to hang
    
    if (abs(rate) != 0)
    {
        if (pwm.attached())
        {
            pwm.adjustFrequency(rate, 0.5);
        }
        else
        {
            pwm.attachPin(GPIO_STEP, rate, 10);
            pwm.writeScaled(0.5);
        }
    }
    else
    {
        pwm.detachPin(GPIO_STEP);
        digitalWrite(GPIO_STEP, LOW);
    }
    #endif // !UNITTEST
}

/**
 * @brief Toggle the sleep line on the driver, ensuring stepping is disabled first.
 * @param sleep True to assert sleep (disable driver), false to wake.
 */
void StepperDriver::setSleep(boolean sleep)
{
    setStepRate(0);

    if(sleep){
        digitalWrite(GPIO_STEP_ENABLE, HIGH);
    }
    else
    {
        digitalWrite(GPIO_STEP_ENABLE, LOW);
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
    if (setting >= 0 && setting <= 1)
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
