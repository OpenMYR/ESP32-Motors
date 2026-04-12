/**
 * @file BrushedMotorDriver.cpp
 * @brief Stubbed brushed motor driver retained only to satisfy the MotorDriver interface.
 */

#include "BrushedMotorDriver.h"

#include <esp_attr.h>
#include <esp_err.h>

BrushedMotorDriver *BrushedMotorDriver::instance = nullptr;

BrushedMotorDriver::BrushedMotorDriver() : MotorDriver()
{
}

BrushedMotorDriver *IRAM_ATTR BrushedMotorDriver::getInstance()
{
    if (instance == nullptr) instance = new BrushedMotorDriver();
    return instance;
}

void BrushedMotorDriver::initMotorGpio()
{
}

void BrushedMotorDriver::isrStartIoDriver()
{
}

void BrushedMotorDriver::isrStopIoDriver()
{
}

void BrushedMotorDriver::motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    // FIXME: Brushed motor support has not been migrated to ESP-IDF yet.
    (void)targetAngle;
    (void)rate;
    (void)motorID;
}

void BrushedMotorDriver::motorMove(int32_t targetAngle, uint16_t rate, uint8_t motorID)
{
    // FIXME: Brushed motor support has not been migrated to ESP-IDF yet.
    (void)targetAngle;
    (void)rate;
    (void)motorID;
}

void BrushedMotorDriver::motorStop(signed int wait_time, unsigned short precision, uint8_t motorID)
{
    // FIXME: Brushed motor support has not been migrated to ESP-IDF yet.
    (void)wait_time;
    (void)precision;
    (void)motorID;
}

void BrushedMotorDriver::motorSleep(signed int wait_time, unsigned short precision, uint8_t motorID)
{
    // FIXME: Brushed motor support has not been migrated to ESP-IDF yet.
    (void)wait_time;
    (void)precision;
    (void)motorID;
}

void BrushedMotorDriver::setOpcodeContext(uint32_t op_seq, uint8_t motor_id)
{
    // FIXME: Brushed motor support has not been migrated to ESP-IDF yet.
    (void)op_seq;
    (void)motor_id;
}

void BrushedMotorDriver::abortCommand(uint8_t motorID)
{
    // FIXME: Brushed motor support has not been migrated to ESP-IDF yet.
    (void)motorID;
}

bool BrushedMotorDriver::isMotorRunning(uint8_t motor_id)
{
    (void)motor_id;
    return false;
}

void IRAM_ATTR BrushedMotorDriver::driver()
{
}

void BrushedMotorDriver::getNextOpForDriver(uint8_t id)
{
    (void)id;
}

void BrushedMotorDriver::peekOpForDriver(uint8_t id)
{
    (void)id;
}

void BrushedMotorDriver::changeMotorSettings(config_setting setting, uint32_t data1, uint32_t data2, uint8_t motorID)
{
    // FIXME: Brushed motor support has not been migrated to ESP-IDF yet.
    (void)setting;
    (void)data1;
    (void)data2;
    (void)motorID;
}

bool BrushedMotorDriver::isEndstopTripped(uint8_t motor_id)
{
    // FIXME: Brushed motor support has not been migrated to ESP-IDF yet.
    (void)motor_id;
    return false;
}

esp_err_t BrushedMotorDriver::setEndstopTrippedPinSetting(uint8_t setting, uint8_t motor_id)
{
    // FIXME: Brushed motor support has not been migrated to ESP-IDF yet.
    (void)setting;
    (void)motor_id;
    return ESP_ERR_NOT_SUPPORTED;
}

bool BrushedMotorDriver::getEndstopTrippedPinSetting(uint8_t motor_id)
{
    // FIXME: Brushed motor support has not been migrated to ESP-IDF yet.
    (void)motor_id;
    return false;
}
