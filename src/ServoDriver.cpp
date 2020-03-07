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

int minUs = 500;
int maxUs = 2400;

ServoDriver::ServoDriver() : MotorDriver()
{
    initMotorGpio();
    memset(commandDone, 1, MAX_MOTORS);
    memset(motorDwell, 0, MAX_MOTORS);
/*     commandDone[0] = 0;
    startTime[0] = esp_timer_get_time();
    commandDeltaTime[0] = 10000000;
    startAngle[0] = 0;
    commandDeltaAngle[0] = 175; */
    log_v("Servo Driver Up");
}

ServoDriver *IRAM_ATTR ServoDriver::getInstance()
{
    if (instance == NULL)
    {
        instance = new ServoDriver();
    }

    return instance;
}

void ServoDriver::initMotorGpio()
{
    servo[0].attach(32);
    servo[1].attach(33);
    servo[2].attach(25);
    servo[3].attach(26);
    servo[4].attach(27);
    servo[5].attach(14);
    servo[6].attach(12);
    servo[7].attach(15);
    servo[8].attach(22);
    servo[9].attach(21);
    servo[10].attach(19);
    servo[11].attach(18);
    servo[12].attach(4);
    servo[13].attach(16);
    servo[14].attach(17);
}

void ServoDriver::motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    if (motorID > MAX_MOTORS)
        return;
    motorID--; //motors are 1-15, we want 0-14

    motorDwell[motorID] = false;
    startAngle[motorID] = currentAngle[motorID];
    commandDeltaAngle[motorID] = targetAngle - currentAngle[motorID];
    startTime[motorID] = esp_timer_get_time();
    commandDeltaTime[motorID] = 1000000 / rate * abs(commandDeltaAngle[motorID]);

    commandDone[motorID] = false;
    log_v("command %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
}

void ServoDriver::motorMove(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    if (motorID > MAX_MOTORS)
        return;
    if (rate == 0)
        return;
    motorID--; //motors are 1-15, we want 0-14

    motorDwell[motorID] = false;
    startAngle[motorID] = currentAngle[motorID];
    commandDeltaAngle[motorID] = targetAngle - currentAngle[motorID];
    startTime[motorID] = esp_timer_get_time();
    commandDeltaTime[motorID] = (uint64_t)commandDeltaAngle[motorID] * rate * 1000000;

    commandDone[motorID] = false;
    log_v("command %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
}

void ServoDriver::motorStop(signed int wait_time, unsigned short precision, uint8_t motorID)
{
    // wait_time, cycles to wait
    // precision, duration of wait cycle in milliseconds
    if (motorID > MAX_MOTORS)
        return;
    motorID--;

    motorDwell[motorID] = true;
    startTime[0] = esp_timer_get_time();
    commandDeltaTime[motorID] = (abs(wait_time) * precision);
    commandDone[motorID] = false;

    log_v("command %d %d %d %d ", startAngle[motorID], commandDeltaAngle[motorID], startTime[motorID], commandDeltaTime[motorID]);
}

void ServoDriver::abortCommand(uint8_t motorID)
{
    if (motorID > MAX_MOTORS)
        return;
    motorID--;
    commandDone[motorID] = true;
}
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

void ServoDriver::isrStopIoDriver()
{
    timerStop(timerDriver);
}

void IRAM_ATTR ServoDriver::isrIo(void *)
{
    //log_i("t");
    ServoDriver::getInstance()->driver();
}

bool ServoDriver::isMotorRunning(uint8_t motor_id)
{
    return !commandDone[motor_id];
}

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

void ServoDriver::getNextOpForDriver(uint8_t id)
{
    CommandLayer::getInstance()->getNextOp(id + 1);
}

void ServoDriver::peekOpForDriver(uint8_t id)
{
    CommandLayer::getInstance()->peekNextOp(id + 1);
}

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
