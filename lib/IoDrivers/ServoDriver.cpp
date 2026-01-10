/**
 * @file ServoDriver.cpp
 * @brief Implements the servo motor driver singleton logic.
 */
#include "ServoDriver.h"
#include <reent.h>
#include "OpBuffer.h"

#define CORE_1 1
#define UPDATE_FREQ 60

#define UPDATE_DWELL 1000 / UPDATE_FREQ
hw_timer_t *timerDriver = NULL;
ServoDriver *ServoDriver::instance = NULL;
CommandLayer *commandInstance = NULL;
static uint8_t peekTicks = 5;
static uint8_t peekRate = 5;

uint8_t servoPin[MAX_MOTORS] = {32, 33, 25, 26, 27, 14, 12, 15, 22, 21, 19, 18, 4, 16, 17};

int minUs = 500;
int maxUs = 2400;

/**
 * @brief Initialize the servo driver and reset pending commands.
 */
ServoDriver::ServoDriver() : MotorDriver()
{
    initMotorGpio();
    memset(commandDone, 1, MAX_MOTORS);
    memset(motorDwell, 0, MAX_MOTORS);
    log_v("Servo Driver Up");
}

/**
 * @brief Return the singleton ServoDriver.
 * @return Pointer to the global ServoDriver.
 */
ServoDriver *IRAM_ATTR ServoDriver::getInstance()
{
    if (instance == NULL)
    {
        instance = new ServoDriver();
    }

    return instance;
}

/**
 * @brief Attach all servo objects to their configured pins.
 */
void ServoDriver::initMotorGpio()
{
    for(int i=0; i<MAX_MOTORS; i++){
        servo[i].attach(servoPin[i]);
    }
}

/**
 * @brief Move a servo toward an absolute angle over the given rate.
 * @param targetAngle Desired absolute angle in driver units.
 * @param rate Transition rate (affects travel time calculation).
 * @param motorID One-based servo index.
 */
void ServoDriver::motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    if (motorID > MAX_MOTORS)
        return;
    motorID--; //motors are 1-15, we want 0-14
    
    if(motorSleeping[motorID]){
        motorSleeping[motorID] = false;
        servo[motorID].attach(servoPin[motorID]);
    }

    motorDwell[motorID] = false;
    motorSleeping[motorID] = false;
    startAngle[motorID] = currentAngle[motorID];
    commandDeltaAngle[motorID] = targetAngle - currentAngle[motorID];
    startTime[motorID] = esp_timer_get_time();
    commandDeltaTime[motorID] = 1000000 / rate * abs(commandDeltaAngle[motorID]);

    commandDone[motorID] = false;
    log_v("command %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
}

/**
 * @brief Advance the servo by a delta from its current position.
 * @param targetAngle Delta to add to the current position.
 * @param rate Rate used for timing the transition.
 * @param motorID One-based servo index.
 */
void ServoDriver::motorMove(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    if (motorID > MAX_MOTORS)
        return;
    if (rate == 0)
        return;
    motorID--; //motors are 1-15, we want 0-14
    
    if(motorSleeping[motorID]){
        motorSleeping[motorID] = false;
        servo[motorID].attach(servoPin[motorID]);
    }

    motorDwell[motorID] = false;
    motorSleeping[motorID] = false;
    startAngle[motorID] = currentAngle[motorID];
    commandDeltaAngle[motorID] = targetAngle - currentAngle[motorID];
    startTime[motorID] = esp_timer_get_time();
    commandDeltaTime[motorID] = (uint64_t)commandDeltaAngle[motorID] * rate * 1000000;

    commandDone[motorID] = false;
    log_v("command %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
}

/**
 * @brief Pause the servo for a duration before resuming command processing.
 * @param wait_time Number of wait cycles.
 * @param precision Duration of each cycle in milliseconds.
 * @param motorID One-based servo index.
 */
void ServoDriver::motorStop(signed int wait_time, unsigned short precision, uint8_t motorID)
{
    // wait_time, cycles to wait
    // precision, duration of wait cycle in milliseconds
    if (motorID > MAX_MOTORS)
        return;
    motorID--;
    
    if(motorSleeping[motorID]){
        motorSleeping[motorID] = false;
        servo[motorID].attach(servoPin[motorID]);
    }

    motorDwell[motorID] = true;
    motorSleeping[motorID] = false;
    startTime[0] = esp_timer_get_time();
    commandDeltaTime[motorID] = (abs(wait_time) * precision);
    commandDone[motorID] = false;

    log_v("command %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
}

/**
 * @brief Stop the servo and detach it to allow the motor to relax.
 * @param wait_time Number of wait cycles.
 * @param precision Duration of each cycle in milliseconds.
 * @param motorID One-based servo index.
 */
void ServoDriver::motorSleep(signed int wait_time, unsigned short precision, uint8_t motorID)
{
    // wait_time, cycles to wait
    // precision, duration of wait cycle in milliseconds
    if (motorID > MAX_MOTORS)
        return;
    motorID--;

    motorDwell[motorID] = true;
    motorSleeping[motorID] = true;
    startTime[0] = esp_timer_get_time();
    commandDeltaTime[motorID] = (abs(wait_time) * precision);
    commandDone[motorID] = false;

    servo[motorID].detach();

    log_v("command %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
}

/**
 * @brief Immediately mark the servo command as complete.
 * @param motorID One-based servo index.
 */
void ServoDriver::abortCommand(uint8_t motorID)
{
    if (motorID > MAX_MOTORS)
        return;
    motorID--;
    commandDone[motorID] = true;
}
/**
 * @brief Launch the servo driver loop on CORE_1.
 */
void ServoDriver::isrStartIoDriver()
{

    xTaskCreatePinnedToCore(
        isrIo,
        "motorloop",
        2000,
        (void *)1,
        0,
        &motorTaskDriver,
        CORE_1);
}

/**
 * @brief Stop the timer driving the servo task.
 */
void ServoDriver::isrStopIoDriver()
{
    timerStop(timerDriver);
}

/**
 * @brief ISR entry that dispatches to the main driver loop.
 */
void IRAM_ATTR ServoDriver::isrIo(void *)
{
    //log_i("t");
    ServoDriver::getInstance()->driver();
}

/**
 * @brief Report whether a servo has a pending command.
 * @param motor_id One-based servo index.
 * @return True if a command is active.
 */
bool ServoDriver::isMotorRunning(uint8_t motor_id)
{
    if (motor_id > MAX_MOTORS)
        return false;
    motor_id--;
    return !commandDone[motor_id];
}

/**
 * @brief Main servo driver loop that updates positions and polls commands.
 */
void IRAM_ATTR ServoDriver::driver()
{
    volatile uint_fast64_t delta;
    while (true)
    {
        if (peekTicks == 0)
        {
            peekTicks = peekRate;
        }
        peekTicks--;

        for (int i = 0; (i < MAX_MOTORS); i++)
        {
            if (!commandDone[i])
            {
                delta = esp_timer_get_time() - startTime[i];
                if (motorDwell[i])
                {
                    if (delta >= commandDeltaTime[i])
                    {
                        commandDone[i] = true;
                    }
                }
                else
                {
                    int angle;
                    if (delta < commandDeltaTime[i])
                    {
                        angle = (float)delta / commandDeltaTime[i] * commandDeltaAngle[i] + startAngle[i];
                    }
                    else
                    {
                        angle = commandDeltaAngle[i] + startAngle[i];
                        commandDone[i] = true;
                    }
                    servo[i].write(angle);
                    currentAngle[i] = angle;
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

        vTaskDelay(0);
    }
    vTaskDelete(NULL);
}

/**
 * @brief Request the next queued operation from the command layer.
 * @param id Zero-based motor index.
 */
void ServoDriver::getNextOpForDriver(uint8_t id)
{
    CommandLayer::getInstance()->getNextOp(id + 1);
}

/**
 * @brief Peek at the next pending operation without dequeuing.
 * @param id Zero-based motor index.
 */
void ServoDriver::peekOpForDriver(uint8_t id)
{
    CommandLayer::getInstance()->peekNextOp(id + 1);
}

/**
 * @brief Update servo configuration (bounds) based on external settings.
 * @param setting Configuration selector.
 * @param data1 Primary value.
 * @param data2 Secondary value (servo index).
 * @param motorID One-based servo index.
 */
void ServoDriver::changeMotorSettings(config_setting setting, uint32_t data1, uint32_t data2, uint8_t motorID)
{
    if (motorID > MAX_MOTORS)
        return;
    motorID--; //motors are 1-15, we want 0-14

    commandDone[motorID] = false;
    if (data2 < MAX_MOTORS)
    {
        if (setting == MAX_SERVO_BOUND)
        {
            confs[data2].max = std::min(data1, (uint32_t)200);
        }
        else if (setting == MIN_SERVO_BOUND)
        {
            confs[data2].min = data1;
        }
    }

    commandDone[motorID] = true;
}
