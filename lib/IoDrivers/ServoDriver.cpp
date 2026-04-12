/**
 * @file ServoDriver.cpp
 * @brief Implements the servo motor driver singleton logic.
 */

#include <algorithm>
#include <cstring>
#include <reent.h>

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ServoDriver.h"
#include "OpBuffer.h"

#define CORE_1 1
#define UPDATE_FREQ 60

#define UPDATE_DWELL 1000 / UPDATE_FREQ
namespace {
constexpr uint32_t kServoPwmFrequencyHz = 50;
constexpr ledc_timer_bit_t kServoPwmResolution = LEDC_TIMER_16_BIT;
constexpr uint32_t kServoPeriodUs = 1000000UL / kServoPwmFrequencyHz;
constexpr uint32_t kServoMinPulseUs = 500;
constexpr uint32_t kServoMaxPulseUs = 2400;
constexpr uint32_t kServoAngleMax = 180;
constexpr uint32_t kServoDutyMax = (1UL << 16) - 1UL;
constexpr gpio_num_t kServoPins[MAX_MOTORS] = {
    GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27,
    GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_15, GPIO_NUM_22, GPIO_NUM_21,
    GPIO_NUM_19, GPIO_NUM_18, GPIO_NUM_4,  GPIO_NUM_16, GPIO_NUM_17,
};

struct ServoPwmChannelConfig {
    ledc_mode_t speedMode;
    ledc_channel_t channel;
    ledc_timer_t timer;
};

const char *TAG = "ServoDriver";
uint8_t sPeekTicks = 5;
uint8_t sPeekRate = 5;
constexpr uint32_t kMotorLoopStackWords = 4096;

bool resolve_motor_index(uint8_t motorID, uint8_t *motorIndex)
{
    if (motorIndex == nullptr) return false;
    if (motorID < 1 || motorID > MAX_MOTORS) return false;
    *motorIndex = static_cast<uint8_t>(motorID - 1);
    return true;
}

ServoPwmChannelConfig resolve_pwm_channel(uint8_t motorIndex)
{
    if (motorIndex < LEDC_CHANNEL_MAX) {
        return {LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(motorIndex), LEDC_TIMER_0};
    }

    return {
        LEDC_HIGH_SPEED_MODE,
        static_cast<ledc_channel_t>(motorIndex - LEDC_CHANNEL_MAX),
        LEDC_TIMER_1,
    };
}

uint32_t angle_to_pulse_width_us(int angle)
{
    const int clampedAngle = std::clamp(angle, 0, static_cast<int>(kServoAngleMax));
    const uint32_t spanUs = kServoMaxPulseUs - kServoMinPulseUs;
    return kServoMinPulseUs +
           (static_cast<uint32_t>(clampedAngle) * spanUs) / kServoAngleMax;
}

uint32_t pulse_width_us_to_duty(uint32_t pulseWidthUs)
{
    return (pulseWidthUs * kServoDutyMax) / kServoPeriodUs;
}
} // namespace

ServoDriver *ServoDriver::instance = NULL;

/**
 * @brief Initialize the servo driver and reset pending commands.
 */
ServoDriver::ServoDriver() : MotorDriver()
{
    initMotorGpio();
    memset(commandDone, 1, MAX_MOTORS);
    memset(motorDwell, 0, MAX_MOTORS);
    ESP_LOGV(TAG, "Servo Driver Up");
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
    ledc_timer_config_t lowSpeedTimer = {};
    lowSpeedTimer.speed_mode = LEDC_LOW_SPEED_MODE;
    lowSpeedTimer.duty_resolution = kServoPwmResolution;
    lowSpeedTimer.timer_num = LEDC_TIMER_0;
    lowSpeedTimer.freq_hz = kServoPwmFrequencyHz;
    lowSpeedTimer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_timer_config(&lowSpeedTimer));

    ledc_timer_config_t highSpeedTimer = {};
    highSpeedTimer.speed_mode = LEDC_HIGH_SPEED_MODE;
    highSpeedTimer.duty_resolution = kServoPwmResolution;
    highSpeedTimer.timer_num = LEDC_TIMER_1;
    highSpeedTimer.freq_hz = kServoPwmFrequencyHz;
    highSpeedTimer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_timer_config(&highSpeedTimer));

    for (uint8_t i = 0; i < MAX_MOTORS; ++i) {
        attachPwmChannel(i);
    }
}

void ServoDriver::attachPwmChannel(uint8_t motorIndex)
{
    if (motorIndex >= MAX_MOTORS) return;

    const ServoPwmChannelConfig pwm = resolve_pwm_channel(motorIndex);
    ledc_channel_config_t channel = {};
    channel.gpio_num = kServoPins[motorIndex];
    channel.speed_mode = pwm.speedMode;
    channel.channel = pwm.channel;
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.timer_sel = pwm.timer;
    channel.duty = 0;
    channel.hpoint = 0;
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_channel_config(&channel));
    pwmAttached[motorIndex] = true;
}

void ServoDriver::detachPwmChannel(uint8_t motorIndex)
{
    if (motorIndex >= MAX_MOTORS) return;
    if (!pwmAttached[motorIndex]) return;

    const ServoPwmChannelConfig pwm = resolve_pwm_channel(motorIndex);
    ESP_ERROR_CHECK_WITHOUT_ABORT(ledc_stop(pwm.speedMode, pwm.channel, 0));
    pwmAttached[motorIndex] = false;
}

void ServoDriver::writeServoAngle(uint8_t motorIndex, int angle)
{
    if (motorIndex >= MAX_MOTORS) return;
    if (!pwmAttached[motorIndex]) attachPwmChannel(motorIndex);

    const ServoPwmChannelConfig pwm = resolve_pwm_channel(motorIndex);
    const uint32_t duty = pulse_width_us_to_duty(angle_to_pulse_width_us(angle));
    if (ledc_set_duty(pwm.speedMode, pwm.channel, duty) != ESP_OK) return;
    if (ledc_update_duty(pwm.speedMode, pwm.channel) != ESP_OK) return;
}

/**
 * @brief Move a servo toward an absolute angle over the given rate.
 * @param targetAngle Desired absolute angle in driver units.
 * @param rate Transition rate (affects travel time calculation).
 * @param motorID One-based servo index.
 */
void ServoDriver::motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    uint8_t motorIndex = 0;
    if (!resolve_motor_index(motorID, &motorIndex)) return;

    if(motorSleeping[motorIndex]){
        motorSleeping[motorIndex] = false;
        attachPwmChannel(motorIndex);
    }

    motorDwell[motorIndex] = false;
    motorSleeping[motorIndex] = false;
    startAngle[motorIndex] = currentAngle[motorIndex];
    commandDeltaAngle[motorIndex] = targetAngle - currentAngle[motorIndex];
    startTime[motorIndex] = esp_timer_get_time();
    commandDeltaTime[motorIndex] = 1000000 / rate * abs(commandDeltaAngle[motorIndex]);

    commandDone[motorIndex] = false;
    ESP_LOGV(TAG, "command %d %d %llu %llu ", startAngle[motorIndex], commandDeltaAngle[motorIndex],
             static_cast<unsigned long long>(startTime[motorIndex]),
             static_cast<unsigned long long>(commandDeltaTime[motorIndex]));
}

/**
 * @brief Advance the servo by a delta from its current position.
 * @param targetAngle Delta to add to the current position.
 * @param rate Rate used for timing the transition.
 * @param motorID One-based servo index.
 */
void ServoDriver::motorMove(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    uint8_t motorIndex = 0;
    if (!resolve_motor_index(motorID, &motorIndex)) return;
    if (rate == 0) return;

    if(motorSleeping[motorIndex]){
        motorSleeping[motorIndex] = false;
        attachPwmChannel(motorIndex);
    }

    motorDwell[motorIndex] = false;
    motorSleeping[motorIndex] = false;
    startAngle[motorIndex] = currentAngle[motorIndex];
    commandDeltaAngle[motorIndex] = targetAngle;
    startTime[motorIndex] = esp_timer_get_time();
    commandDeltaTime[motorIndex] = (1000000ULL * static_cast<uint64_t>(abs(commandDeltaAngle[motorIndex]))) / rate;

    commandDone[motorIndex] = false;
    ESP_LOGV(TAG, "command %d %d %llu %llu ", startAngle[motorIndex], commandDeltaAngle[motorIndex],
             static_cast<unsigned long long>(startTime[motorIndex]),
             static_cast<unsigned long long>(commandDeltaTime[motorIndex]));
}

/**
 * @brief Pause the servo for a duration before resuming command processing.
 * @param wait_time Number of wait cycles.
 * @param precision Duration of each cycle in milliseconds.
 * @param motorID One-based servo index.
 */
void ServoDriver::motorStop(int32_t wait_time, uint16_t precision, uint8_t motorID)
{
    // wait_time, cycles to wait
    // precision, duration of wait cycle in milliseconds
    uint8_t motorIndex = 0;
    if (!resolve_motor_index(motorID, &motorIndex)) return;

    if(motorSleeping[motorIndex]){
        motorSleeping[motorIndex] = false;
        attachPwmChannel(motorIndex);
    }

    motorDwell[motorIndex] = true;
    motorSleeping[motorIndex] = false;
    startTime[motorIndex] = esp_timer_get_time();
    commandDeltaTime[motorIndex] = (abs(wait_time) * precision);
    commandDone[motorIndex] = false;

    ESP_LOGV(TAG, "command %d %d %llu %llu ", startAngle[motorIndex], commandDeltaAngle[motorIndex],
             static_cast<unsigned long long>(startTime[motorIndex]),
             static_cast<unsigned long long>(commandDeltaTime[motorIndex]));
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
    uint8_t motorIndex = 0;
    if (!resolve_motor_index(motorID, &motorIndex)) return;

    motorDwell[motorIndex] = true;
    motorSleeping[motorIndex] = true;
    startTime[motorIndex] = esp_timer_get_time();
    commandDeltaTime[motorIndex] = (abs(wait_time) * precision);
    commandDone[motorIndex] = false;

    detachPwmChannel(motorIndex);

    ESP_LOGV(TAG, "command %d %d %llu %llu ", startAngle[motorIndex], commandDeltaAngle[motorIndex],
             static_cast<unsigned long long>(startTime[motorIndex]),
             static_cast<unsigned long long>(commandDeltaTime[motorIndex]));
}

/**
 * @brief Immediately mark the servo command as complete.
 * @param motorID One-based servo index.
 */
void ServoDriver::abortCommand(uint8_t motorID)
{
    uint8_t motorIndex = 0;
    if (!resolve_motor_index(motorID, &motorIndex)) return;
    commandDone[motorIndex] = true;
}
/**
 * @brief Launch the servo driver loop on CORE_1.
 */
void ServoDriver::isrStartIoDriver()
{
    BaseType_t ok = xTaskCreatePinnedToCore(
        isrIo,
        "motorloop",
        kMotorLoopStackWords,
        (void *)1,
        MotorDriver::kMotorLoopTaskPriority,
        &motorTaskDriver,
        CORE_1);
    if (ok != pdPASS)
    {
        ESP_LOGE(TAG, "motorloop task creation failed");
    }
}

/**
 * @brief Stop the timer driving the servo task.
 */
void ServoDriver::isrStopIoDriver()
{
    for (uint8_t i = 0; i < MAX_MOTORS; ++i) {
        detachPwmChannel(i);
    }
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
    uint8_t motorIndex = 0;
    if (!resolve_motor_index(motor_id, &motorIndex)) return false;
    return !commandDone[motorIndex];
}

/**
 * @brief Main servo driver loop that updates positions and polls commands.
 */
void IRAM_ATTR ServoDriver::driver()
{
    volatile uint_fast64_t delta;
    while (true)
    {
        if (sPeekTicks == 0)
        {
            sPeekTicks = sPeekRate;
        }
        sPeekTicks--;

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
                    writeServoAngle(static_cast<uint8_t>(i), angle);
                    currentAngle[i] = angle;
                }
                if (sPeekTicks == 0)
                {
                    peekOpForDriver(i);
                }
            }
            else
            {
                getNextOpForDriver(i);
            }
        }

        // Always yield at least one tick so IDLE1 can run and service task watchdog.
        vTaskDelay(1);
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
    uint8_t motorIndex = 0;
    if (!resolve_motor_index(motorID, &motorIndex)) return;

    commandDone[motorIndex] = false;
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

    commandDone[motorIndex] = true;
}
