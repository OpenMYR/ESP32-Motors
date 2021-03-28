#ifndef _MotorDriver_H_
#define _MotorDriver_H_

#include <stdint.h>
#include <FreeRTOS.h>

class MotorDriver
{
    public:
        MotorDriver()
        {
        };

        enum config_setting
        {
            MIN_SERVO_BOUND,
            MAX_SERVO_BOUND,
            MICROSTEPPING
        };

        virtual void isrStartIoDriver();
        virtual void isrStopIoDriver();
        virtual bool isMotorRunning(uint8_t motor_id);
        virtual void changeMotorSettings(config_setting setting, uint32_t data1, uint32_t data2, uint8_t motor_id);
        virtual void motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID);
        virtual void motorMove(int32_t deltaAngle, uint16_t rate, uint8_t motorID);
        virtual void motorStop(signed int wait_time, unsigned short precision, uint8_t motor_id);
        virtual void motorSleep(signed int wait_time, unsigned short precision, uint8_t motor_id);
        virtual void abortCommand(uint8_t motorID);

    protected:
        virtual void initMotorGpio();
        virtual void IRAM_ATTR driver();
        TaskHandle_t motorTaskDriver;
        virtual void getNextOpForDriver(uint8_t id);
        virtual void peekOpForDriver(uint8_t id);

};

#endif /* _MotorDriver_H_ */