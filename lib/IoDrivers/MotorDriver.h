#ifndef MYR_MOTORDRIVER_H
#define MYR_MOTORDRIVER_H

/**
 * @file MotorDriver.h
 * @brief Shared abstraction for motor driver subclasses.
 */

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**
 * @brief Base interface for motor driver implementations.
 */
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

        /**
         * @brief Start the background IO driver task.
         */
        virtual void isrStartIoDriver() {}

        /**
         * @brief Stop the IO driver and release resources.
         */
        virtual void isrStopIoDriver() {}

        /**
         * @brief Report whether a motor currently has a command.
         * @param motor_id One-based motor identifier.
         * @return True if the motor is busy.
         */
        virtual bool isMotorRunning(uint8_t motor_id) { return false; }

        /**
         * @brief Update implementation-specific configuration.
         * @param setting Selector describing the configuration change.
         * @param data1 Primary configuration value.
         * @param data2 Secondary configuration value.
         * @param motor_id One-based motor identifier.
         */
        virtual void changeMotorSettings(config_setting setting, uint32_t data1, uint32_t data2, uint8_t motor_id) {}

        /**
         * @brief Command an absolute position target.
         * @param targetAngle Absolute goal.
         * @param rate Rate used for timing.
         * @param motorID One-based motor identifier.
         */
        virtual void motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID) {}

        /**
         * @brief Command a delta movement relative to the current position.
         * @param deltaAngle Relative move.
         * @param rate Rate used to time the move.
         * @param motorID One-based motor identifier.
         */
        virtual void motorMove(int32_t deltaAngle, uint16_t rate, uint8_t motorID) {}

        /**
         * @brief Temporarily hold the motor for the provided duration.
         * @param wait_time Number of cycles to wait.
         * @param precision Cycle duration in milliseconds.
         * @param motor_id One-based motor identifier.
         */
        virtual void motorStop(signed int wait_time, unsigned short precision, uint8_t motor_id) {}

        /**
         * @brief Put the motor into a sleep state after the wait duration.
         * @param wait_time Number of cycles to wait before sleeping.
         * @param precision Cycle duration in milliseconds.
         * @param motor_id One-based motor identifier.
         */
        virtual void motorSleep(signed int wait_time, unsigned short precision, uint8_t motor_id) {}

        /**
         * @brief Cancel the current motor command immediately.
         * @param motorID One-based motor identifier.
         */
        virtual void abortCommand(uint8_t motorID) {}

    protected:
        /**
         * @brief Initialize driver-specific GPIOs.
         */
        virtual void initMotorGpio() {}

        /**
         * @brief Core execution loop for the driver task.
         */
        virtual void IRAM_ATTR driver() {}

        /**
         * @brief Request the next operation from the command layer.
         * @param id Zero-based motor index.
         */
        virtual void getNextOpForDriver(uint8_t id) {}

        /**
         * @brief Peek at the next pending operation.
         * @param id Zero-based motor index.
         */
        virtual void peekOpForDriver(uint8_t id) {}
        
        TaskHandle_t motorTaskDriver;

};

#endif // MYR_MOTORDRIVER_H
