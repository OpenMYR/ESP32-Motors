/**
 * @file BrushedMotorDriver.cpp
 * @brief Implements the singleton for controlling brushed motor bridges.
 */
#include <reent.h>
#include <math.h>
#include <ESP32PWM.h>
#include <ESP32Servo.h>
#include <esp32-hal-ledc.h>
#include <driver/periph_ctrl.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/pcnt.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <soc/gpio_sig_map.h>

#include "BrushedMotorDriver.h"
#include "OpBuffer.h"

#define CORE_1 1
#define UPDATE_FREQ 1000

#define UPDATE_DWELL 1000 / UPDATE_FREQ
#define motorInterfaceType 1

//static hw_timer_t *timerDriver = NULL;
BrushedMotorDriver *BrushedMotorDriver::instance = NULL;
//static CommandLayer *commandInstance = NULL;

#define GPIO_IN1 16
#define GPIO_IN2 13
#define GPIO_nFAULT 14
#define GPIO_nSLEEP 27

//end stop
#define GPIO_IO_A 21
#define GPIO_IO_B 22

//ESP32PWM pwm;

/**
 * @brief Construct the brushed motor driver, initialize GPIO, and reset command state.
 */
BrushedMotorDriver::BrushedMotorDriver() : MotorDriver()
{
    Serial.write("BrushedMotorDriver const\n");
    peekTicks = peekRate;
    initMotorGpio();
    memset(commandDone, 1, MAX_BrushedMotor_MOTORS);
}

/**
 * @brief Return the singleton BrushedMotorDriver instance.
 * @return Pointer to the global BrushedMotorDriver.
 */
BrushedMotorDriver *IRAM_ATTR BrushedMotorDriver::getInstance()
{
    if (instance == NULL)
    {
        instance = new BrushedMotorDriver();
    }

    return instance;
}

/**
 * @brief Configure the GPIOs for the brushed motor bridge and endstop inputs.
 */
void BrushedMotorDriver::initMotorGpio()
{
    motorsControlled = 2;
    attachInterrupt(GPIO_IO_A, endstop_a_interrupt, CHANGE);
    attachInterrupt(GPIO_IO_B, endstop_b_interrupt, CHANGE);
    pinMode(GPIO_IN1, OUTPUT);
    digitalWrite(GPIO_IN1, LOW);
    pinMode(GPIO_IN2, OUTPUT);
    digitalWrite(GPIO_IN2, LOW);
    pinMode(GPIO_nSLEEP, OUTPUT);
    digitalWrite(GPIO_nSLEEP, HIGH);
    pinMode(GPIO_nFAULT, INPUT);
    digitalWrite(GPIO_nFAULT, LOW);
    positiveDirection = true;
    direction = 0;
    paused = 0;
    endstop_a = 0;
    endstop_b = 0;
}

//
// targetAngle: direction, 1 forward, 0 coast, -1 reverse
// rate: speed 0-255
//
/**
 * @brief Move the brushed motor to a specific target based on the signed speed value.
 * @param motorID One-based motor identifier.
 */
void BrushedMotorDriver::motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    //double goToDegrees = (double)targetAngle;
    if (motorID > motorsControlled)
        return;
    motorID--; //motors are 1-3, we want 0-2

    if (endstop_a || endstop_b)
        return;

    motorDwell = false;

    startAngle[motorID] = currentAngle[motorID];
    commandDeltaAngle[motorID] = targetAngle;
    startTime[motorID] = esp_timer_get_time();
    commandDeltaTime[motorID] = 0xFFFFFFFF;
    commandDone[motorID] = false;

    if (targetAngle >= 1)
    {
        // forward
        direction = positiveDirection;
    }
    else if (targetAngle <= -1)
    {
        //reverse
        direction = !positiveDirection;
    }
    else
    {
        // coast
    }

    if ((targetAngle < 1) && (targetAngle > -1))
    {
        setSpeed(0, 0);
    }
    else
    {
        setSpeed((uint8_t)(abs(targetAngle)), direction);
    }

    log_i("command: motorGoTo %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
}

/**
 * @brief Apply the given signed speed directly, mimicking a move command.
 * @param motorID One-based motor identifier.
 */
void BrushedMotorDriver::motorMove(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    //double goToDegrees = (double)targetAngle;
    if (motorID > motorsControlled)
        return;
    motorID--; //motors are 1-3, we want 0-2

    if (endstop_a || endstop_b)
        return;

    motorDwell = false;

    startAngle[motorID] = currentAngle[motorID];
    commandDeltaAngle[motorID] = targetAngle;
    startTime[motorID] = esp_timer_get_time();
    commandDeltaTime[motorID] = 0xFFFFFFFF;
    commandDone[motorID] = false;

    if (targetAngle >= 1)
    {
        // forward
        direction = positiveDirection;
    }
    else if (targetAngle <= -1)
    {
        //reverse
        direction = !positiveDirection;
    }
    else
    {
    }

    if ((targetAngle < 1) && (targetAngle > -1))
    {
        setSpeed(0, 0);
    }
    else
    {
        setSpeed((uint8_t)(abs(targetAngle)), direction);
    }

    log_i("command: motorGoTo %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
}

/**
 * @brief Hold the motors for the specified duration before allowing new commands.
 * @param wait_time
 * @param precision
 * @param motorID One-based motor identifier.
 */
void BrushedMotorDriver::motorStop(signed int wait_time, unsigned short precision, uint8_t motorID)
{
    // wait_time, cycles to wait
    // precision, duration of wait cycle in milliseconds
    if (motorID > motorsControlled)
        return;
    motorID--;

    if (endstop_a || endstop_b)
        return;

    motorDwell = true;
    startTime[0] = esp_timer_get_time();
    commandDeltaTime[motorID] = startTime[0] + (abs(wait_time) * precision);
    commandDone[motorID] = false;

    setSpeed(0, 0);

    log_i("command %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
}

/**
 * @brief Immediately disable the bridge outputs and mark the command done.
 * @param motorID One-based motor identifier.
 */
void BrushedMotorDriver::abortCommand(uint8_t motorID)
{
    log_i("abort command");
    if (motorID > motorsControlled)
        return;
    motorID--;
    setSpeed(0, 0);
    commandDone[motorID] = true;
}

/**
 * @brief Create a FreeRTOS task pinned to CORE_1 to handle the driver loop.
 */
void BrushedMotorDriver::isrStartIoDriver()
{
    vTaskDelay(0);

    xTaskCreatePinnedToCore(
        isrIoBDC,
        "motorloopbdc",
        2000,
        (void *)1,
        0,
        &motorTaskDriver,
        CORE_1);
}

/**
 * @brief ISR that updates the cached state for endstop A.
 */
void IRAM_ATTR BrushedMotorDriver::endstop_a_interrupt()
{
    if (digitalRead(GPIO_IO_A) == 0)
    {
        BrushedMotorDriver::getInstance()->endstop_a = 1;
    }
    else
    {
        BrushedMotorDriver::getInstance()->endstop_a = 0;
        setSpeed(0, 0);
    }
}

/**
 * @brief ISR that updates the cached state for endstop B.
 */
void IRAM_ATTR BrushedMotorDriver::endstop_b_interrupt()
{
    if (digitalRead(GPIO_IO_B) == 0)
    {
        BrushedMotorDriver::getInstance()->endstop_b = 1;
    }
    else
    {
        BrushedMotorDriver::getInstance()->endstop_b = 0;
        setSpeed(0, 0);
    }
}

/**
 * @brief Tear down the brushed driver outputs and enable motor sleep.
 */
void BrushedMotorDriver::isrStopIoDriver()
{
    digitalWrite(GPIO_nSLEEP, LOW);
    BrushedMotorDriver::getInstance()->bdcPWM1.detachPin(GPIO_IN1);
    BrushedMotorDriver::getInstance()->bdcPWM2.detachPin(GPIO_IN2);
    digitalWrite(GPIO_IN1, LOW);
    digitalWrite(GPIO_IN2, LOW);
}

/**
 * @brief Entry point for the brushed motor driver task; initializes PWM and invokes driver().
 * @param pvParameters Task parameter (unused).
 */
void BrushedMotorDriver::isrIoBDC(void *pvParameters)
{
    pinMode(GPIO_IN1, OUTPUT);
    digitalWrite(GPIO_IN1, LOW);
    pinMode(GPIO_IN2, OUTPUT);
    digitalWrite(GPIO_IN2, LOW);
    pinMode(GPIO_nSLEEP, OUTPUT);
    digitalWrite(GPIO_nSLEEP, HIGH);
    BrushedMotorDriver::getInstance()->bdcPWM1.attachPin(GPIO_IN1, 50000, 10);
    BrushedMotorDriver::getInstance()->bdcPWM1.writeScaled(0.0);
    BrushedMotorDriver::getInstance()->bdcPWM2.attachPin(GPIO_IN2, 50000, 10);
    BrushedMotorDriver::getInstance()->bdcPWM2.writeScaled(0.0);
    BrushedMotorDriver::getInstance()->driver();
}

/**
 * @brief Report whether a motor still has a pending command.
 * @param motor_id Zero-based motor index.
 * @return True if the motor is still executing a command.
 */
bool BrushedMotorDriver::isMotorRunning(uint8_t motor_id)
{
    return !commandDone[motor_id];
}

/**
 * @brief Main loop that checks for command timeouts and pulls operations from the command layer.
 */
void IRAM_ATTR BrushedMotorDriver::driver()
{
    volatile uint_fast64_t delta;
    log_i("driver start");
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
                if (motorDwell)
                {
                    delta = esp_timer_get_time();
                    if (delta >= commandDeltaTime[i])
                    {
                        commandDone[i] = true;
                    }
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
 * @brief Request the next operation from the command layer for polling.
 * @param id Zero-based motor index.
 */
void BrushedMotorDriver::getNextOpForDriver(uint8_t id)
{
    CommandLayer::getInstance()->getNextOp(id + 1);
}

/**
 * @brief Peek at the next operation without dequeuing it.
 * @param id Zero-based motor index.
 */
void BrushedMotorDriver::peekOpForDriver(uint8_t id)
{
    CommandLayer::getInstance()->peekNextOp(id + 1);
}

/**
 * @brief Adjust driver configuration; currently unimplemented.
 * @param setting Configuration selector.
 * @param data1 Primary argument.
 * @param data2 Secondary argument.
 * @param motorID One-based motor identifier.
 */
void BrushedMotorDriver::changeMotorSettings(config_setting setting, uint32_t data1, uint32_t data2, uint8_t motorID)
{
    if (motorID > motorsControlled)
        return;
    motorID--;
}

/**
 * @brief Set the direction and PWM duty cycle for the brushed motor bridge.
 * @param power Value 0-255 that controls PWM duty cycle.
 * @param mDirection Direction flag (1 forward, 0 reverse).
 */
void BrushedMotorDriver::setSpeed(uint8_t power, int8_t mDirection)
{
    float scaledPower = 0.0;
    if(power != 0){
        scaledPower = float(power)/255.0;
    } 
    log_i("Speed: %d %d %f", power, mDirection, scaledPower);
    if ((power < 1) && (power > -1))
    {
        //coast
        BrushedMotorDriver::getInstance()->bdcPWM1.writeScaled(0.0);
        BrushedMotorDriver::getInstance()->bdcPWM2.writeScaled(0.0);
    }
    else if (mDirection == 1)
    {
        // forward
        BrushedMotorDriver::getInstance()->bdcPWM1.writeScaled(scaledPower);
        BrushedMotorDriver::getInstance()->bdcPWM2.writeScaled(0.0);
    }
    else if (mDirection == 0)
    {
        //reverse
        BrushedMotorDriver::getInstance()->bdcPWM1.writeScaled(0.0);
        BrushedMotorDriver::getInstance()->bdcPWM2.writeScaled(scaledPower);
    }
    else
    {
        // NAN
    }
}
