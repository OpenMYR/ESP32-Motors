#include "config/Config.h"

#include "CommandLayer.h"

#include "esp_log.h"
#include "OpBuffer.h"

#if SERVO == 1
#include "ServoDriver.h"
#elif STEPPER == 1
#include "StepperDriver.h"
#elif BDC == 1
#include "BrushedMotorDriver.h"
#endif

namespace {
const char *TAG = "CommandLayer";
}

CommandLayer *CommandLayer::instance = nullptr;
MotorDriver *CommandLayer::driver = nullptr;

CommandLayer::CommandLayer()
{
#if SERVO == 1
    driver = ServoDriver::getInstance();
#elif STEPPER == 1
    driver = StepperDriver::getInstance();
#elif BDC == 1
    driver = BrushedMotorDriver::getInstance();
#endif
    ESP_LOGV(TAG, "CommandLayer ctor");
}

void CommandLayer::init()
{
    CommandLayer::driver->isrStartIoDriver();
    ESP_LOGV(TAG, "CommandLayer init");
}

CommandLayer *CommandLayer::getInstance()
{
    if (instance == nullptr)
    {
        instance = new CommandLayer();
    }
    return instance;
}

void CommandLayer::opcodeMove(signed int delta_units, unsigned short rate, uint8_t motor_id)
{
    CommandLayer::driver->motorMove(delta_units, rate, motor_id);
}

void CommandLayer::opcodeGoto(signed int target_units, unsigned short rate, uint8_t motor_id)
{
    CommandLayer::driver->motorGoTo(target_units, rate, motor_id);
}

void CommandLayer::opcodeStop(signed int wait_time, unsigned short interval_us, uint8_t motor_id)
{
    CommandLayer::driver->motorStop(wait_time, interval_us, motor_id);
}

void CommandLayer::opcodeSleep(signed int step_num, unsigned short step_rate, uint8_t motor_id)
{
    CommandLayer::driver->motorSleep(step_num, step_rate, motor_id);
}

void CommandLayer::opcodeMotorSetting(MotorDriver::config_setting setting, uint32_t data1, uint32_t data2, uint8_t motor_id)
{
    CommandLayer::driver->changeMotorSettings(setting, data1, data2, motor_id);
}

void CommandLayer::opcodeAbortCommand(uint8_t motor_id)
{
    ESP_LOGW(TAG, "Abort active command: motor=%u", static_cast<unsigned>(motor_id));
    CommandLayer::driver->abortCommand(motor_id);
}

void CommandLayer::fetchMotorOpCode(uint8_t id)
{
    //log_i("fetchMotorOpCode %d\n", id);
    parseSubmittOp(id, OpBuffer::getInstance()->getOp(id));
}

void CommandLayer::FillDriverFromQueue()
{
    /* Legacy queue feeder path kept for reference.
    while (true)
    {
        for (int i = 0; (i < MAX_MOTORS); i++)
        {
            if (!driver->isMotorRunning(i))
            {
                fetchMotorOpCode(i);
            }
        }
        delay(UPDATE_DWELL);
    }
    vTaskDelete(NULL); */
}

void CommandLayer::parseSubmittOp(uint8_t id, Op *op)
{
    (void)id;
    if (op == nullptr) return;
    CommandLayer::driver->setOpcodeContext(op->opSeq, op->motorID);
    MotorOpcode opcode;
    if (!try_parse_motor_opcode(op->opcode, &opcode)) return;

    switch (opcode)
    {
    case MotorOpcode::Move:
    {
        opcodeMove(op->stepNum, op->stepRate, op->motorID);
        break;
    }
    case MotorOpcode::Stop:
    {
        opcodeStop(op->stepNum, op->stepRate, op->motorID);
        break;
    }
    case MotorOpcode::Goto:
    {
        opcodeGoto(op->stepNum, op->stepRate, op->motorID);
        break;
    }
    case MotorOpcode::Sleep:
    {
        opcodeSleep(op->stepNum, op->stepRate, op->motorID);
        break;
    }
    case MotorOpcode::Microstep:
    {
        opcodeMotorSetting(MotorDriver::config_setting::MICROSTEPPING, op->stepRate, op->motorID, op->motorID);
        break;
    }
    case MotorOpcode::Abort:
    {
        opcodeAbortCommand(op->motorID);
        break;
    }
    default:
        break;
    }
}

void CommandLayer::getNextOp(uint8_t driverId)
{
    fetchMotorOpCode(driverId);
}

void CommandLayer::peekNextOp(uint8_t driverId)
{
    Op *peekedOp = OpBuffer::getInstance()->peekOp(driverId);
    if (peekedOp == nullptr)
    {
        return;
    }

    MotorOpcode opcode;
    if (!try_parse_motor_opcode(peekedOp->opcode, &opcode)) return;
    if (opcode != MotorOpcode::Abort) return;

    CommandLayer::driver->setOpcodeContext(peekedOp->opSeq, driverId);
    opcodeAbortCommand(driverId);
}
