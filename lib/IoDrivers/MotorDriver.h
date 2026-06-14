#ifndef MYR_MOTORDRIVER_H
#define MYR_MOTORDRIVER_H

/**
 * @file MotorDriver.h
 * @brief Shared abstraction for motor driver subclasses.
 */

#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>

/**
 * @brief Base interface for motor driver implementations.
 */
class MotorDriver
{
    public:
        static constexpr UBaseType_t kMotorLoopTaskPriority = 5;

        MotorDriver()
        {
        };

        static uint64_t planStopDurationUs(int32_t waitCount, uint16_t intervalUs)
        {
            const uint64_t magnitude =
                waitCount >= 0 ? static_cast<uint64_t>(waitCount) : static_cast<uint64_t>(-(static_cast<int64_t>(waitCount)));
            return magnitude * static_cast<uint64_t>(intervalUs);
        }

        enum config_setting
        {
            MIN_SERVO_BOUND,
            MAX_SERVO_BOUND,
            MICROSTEPPING
        };

        /**
         * @brief Start the background IO driver task.
         */
        virtual void isrStartIoDriver() = 0;

        /**
         * @brief Stop the IO driver and release resources.
         */
        virtual void isrStopIoDriver() = 0;

        /**
         * @brief Report whether a motor currently has a command.
         * @param motor_id One-based motor identifier.
         * @return True if the motor is busy.
         */
        virtual bool isMotorRunning(uint8_t motor_id) = 0;

        /**
         * @brief Update implementation-specific configuration.
         * @param setting Selector describing the configuration change.
         * @param data1 Primary configuration value.
         * @param data2 Secondary configuration value.
         * @param motor_id One-based motor identifier.
         */
        virtual void changeMotorSettings(config_setting setting, uint32_t data1, uint32_t data2, uint8_t motor_id) = 0;

        /**
         * @brief Command an absolute position target.
         * @param targetAngle Absolute goal.
         * @param rate Rate used for timing.
         * @param motorID One-based motor identifier.
         */
        virtual void motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID) = 0;

        /**
         * @brief Command a delta movement relative to the current position.
         * @param deltaAngle Relative move.
         * @param rate Rate used to time the move.
         * @param motorID One-based motor identifier.
         */
        virtual void motorMove(int32_t deltaAngle, uint16_t rate, uint8_t motorID) = 0;

        /**
         * @brief Temporarily hold the motor for the provided duration.
         * @param wait_time Signed legacy wait count.
         * @param interval_us Duration of each wait count in microseconds for Stop.
         * @param motor_id One-based motor identifier.
         */
        virtual void motorStop(signed int wait_time, unsigned short interval_us, uint8_t motor_id) = 0;

        /**
         * @brief Put the motor into a sleep state after the wait duration.
         * @param wait_time Number of cycles to wait before sleeping.
         * @param precision Driver-specific sleep timing field.
         * @param motor_id One-based motor identifier.
         */
        virtual void motorSleep(signed int wait_time, unsigned short precision, uint8_t motor_id) = 0;

        /**
         * @brief Set per-dispatch opcode context for the next driver call.
         * @param op_seq Monotonic opcode sequence number.
         * @param motor_id One-based motor identifier.
         */
        virtual void setOpcodeContext(uint32_t op_seq, uint8_t motor_id) = 0;

        /**
         * @brief Cancel the current motor command immediately.
         * @param motorID One-based motor identifier.
         */
        virtual void abortCommand(uint8_t motorID) = 0;

        /**
         * @brief Query endstop state for a motor.
         * @param motor_id One-based motor identifier.
         * @return True when the selected motor has a tripped endstop.
         */
        virtual bool isEndstopTripped(uint8_t motor_id) = 0;

        /**
         * @brief Configure endstop active polarity for a motor.
         * @param setting 0 for active-low, 1 for active-high.
         * @param motor_id One-based motor identifier.
         * @return ESP_OK when applied, ESP_ERR_NOT_SUPPORTED when unavailable.
         */
        virtual esp_err_t setEndstopTrippedPinSetting(uint8_t setting, uint8_t motor_id) = 0;

        /**
         * @brief Return configured endstop active polarity for a motor.
         * @param motor_id One-based motor identifier.
         * @return True when active-high is configured.
         */
        virtual bool getEndstopTrippedPinSetting(uint8_t motor_id) = 0;

    protected:
        /**
         * @brief Initialize driver-specific GPIOs.
         */
        virtual void initMotorGpio() = 0;

        /**
         * @brief Core execution loop for the driver task.
         */
        virtual void IRAM_ATTR driver() = 0;

        /**
         * @brief Request the next operation from the command layer.
         * @param id Zero-based motor index.
         */
        virtual void getNextOpForDriver(uint8_t id) = 0;

        /**
         * @brief Peek at the next pending operation.
         * @param id Zero-based motor index.
         */
        virtual void peekOpForDriver(uint8_t id) = 0;
        
        /**
         * @brief  Validate a one-based motor ID and convert it to a zero-based index.
         * @param motorID One-based motor identifier from command input.
         * @param motor_count Number of motors supported by the active driver.
         * @param[out] motor_index Zero-based index to use for internal arrays.
         * @return true if @p motorID is in the valid range [1, motor_count]; false otherwise.
         */        
        static inline bool tryResolveMotorIndex(uint8_t motorID, uint8_t motorsControlled, uint8_t &idx) {
          if (motorID == 0 || motorID > motorsControlled) return false;
          idx = static_cast<uint8_t>(motorID - 1);
          return true;
        }

        TaskHandle_t motorTaskDriver;

};

#endif // MYR_MOTORDRIVER_H
