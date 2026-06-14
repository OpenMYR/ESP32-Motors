#ifndef MYR_BRUSHEDMOTORDRIVER_H
#define MYR_BRUSHEDMOTORDRIVER_H

/**
 * @file BrushedMotorDriver.h
 * @brief Stubbed brushed motor driver retained only to satisfy the MotorDriver interface.
 */

#include <stdint.h>

#include "MotorDriver.h"

class BrushedMotorDriver : public MotorDriver
{
public:
    BrushedMotorDriver();

    bool isMotorRunning(uint8_t motor_id) override;
    void isrStartIoDriver() override;
    void isrStopIoDriver() override;
    void motorGoTo(int32_t targetUnits, uint16_t rate, uint8_t motorID) override;
    void motorMove(int32_t deltaUnits, uint16_t rate, uint8_t motorID) override;
    void motorStop(signed int wait_time, unsigned short precision, uint8_t motorID) override;
    void motorSleep(signed int wait_time, unsigned short precision, uint8_t motorID) override;
    void setOpcodeContext(uint32_t op_seq, uint8_t motor_id) override;
    void abortCommand(uint8_t motorID) override;
    static BrushedMotorDriver *IRAM_ATTR getInstance();
    void changeMotorSettings(MotorDriver::config_setting setting, uint32_t data1, uint32_t data2, uint8_t motorID) override;
    bool isEndstopTripped(uint8_t motor_id) override;
    esp_err_t setEndstopTrippedPinSetting(uint8_t setting, uint8_t motor_id) override;
    bool getEndstopTrippedPinSetting(uint8_t motor_id) override;

private:
    void initMotorGpio() override;
    void IRAM_ATTR driver() override;
    void getNextOpForDriver(uint8_t id) override;
    void peekOpForDriver(uint8_t id) override;

    static BrushedMotorDriver *instance;
};

#endif // MYR_BRUSHEDMOTORDRIVER_H
