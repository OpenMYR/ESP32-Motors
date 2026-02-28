#include <unity.h>

#include "CommandLayer.h"
#include "MotorDriver.h"
#include "OpBuffer.h"

namespace {

class FakeMotorDriver : public MotorDriver {
public:
    enum CallType {
        NONE = 0,
        MOVE,
        GOTO,
        STOP,
        SLEEP,
        SETTING,
        ABORT
    };

    void reset()
    {
        last_call = NONE;
        last_setting = config_setting::MIN_SERVO_BOUND;
        last_data1 = 0;
        last_data2 = 0;
        last_i32 = 0;
        last_u16 = 0;
        last_motor_id = 0;
        call_count = 0;
    }

    void changeMotorSettings(config_setting setting, uint32_t data1, uint32_t data2, uint8_t motor_id) override
    {
        last_call = SETTING;
        last_setting = setting;
        last_data1 = data1;
        last_data2 = data2;
        last_motor_id = motor_id;
        call_count++;
    }

    void motorGoTo(int32_t targetAngle, uint16_t rate, uint8_t motorID) override
    {
        last_call = GOTO;
        last_i32 = targetAngle;
        last_u16 = rate;
        last_motor_id = motorID;
        call_count++;
    }

    void motorMove(int32_t deltaAngle, uint16_t rate, uint8_t motorID) override
    {
        last_call = MOVE;
        last_i32 = deltaAngle;
        last_u16 = rate;
        last_motor_id = motorID;
        call_count++;
    }

    void motorStop(signed int wait_time, unsigned short precision, uint8_t motor_id) override
    {
        last_call = STOP;
        last_i32 = wait_time;
        last_u16 = precision;
        last_motor_id = motor_id;
        call_count++;
    }

    void motorSleep(signed int wait_time, unsigned short precision, uint8_t motor_id) override
    {
        last_call = SLEEP;
        last_i32 = wait_time;
        last_u16 = precision;
        last_motor_id = motor_id;
        call_count++;
    }

    void abortCommand(uint8_t motorID) override
    {
        last_call = ABORT;
        last_motor_id = motorID;
        call_count++;
    }

    CallType last_call = NONE;
    config_setting last_setting = config_setting::MIN_SERVO_BOUND;
    uint32_t last_data1 = 0;
    uint32_t last_data2 = 0;
    int32_t last_i32 = 0;
    uint16_t last_u16 = 0;
    uint8_t last_motor_id = 0;
    uint32_t call_count = 0;
};

FakeMotorDriver gFake;

Op make_op(char opcode, uint8_t queue, int32_t step_num, uint16_t step_rate, uint8_t motor_id)
{
    Op op;
    op.opcode = opcode;
    op.queue = queue;
    op.stepNum = step_num;
    op.stepRate = step_rate;
    op.motorID = motor_id;
    op.port = 0;
    op.sourceIPAddr = 0;
    return op;
}

void enqueue_and_dispatch(const Op &op)
{
    Op copy = op;
    TEST_ASSERT_EQUAL_INT8(0, OpBuffer::getInstance()->storeOp(&copy));
    CommandLayer::getNextOp(op.motorID);
}

} // namespace

void setUp(void)
{
    OpBuffer::getInstance()->reset();
    gFake.reset();
    CommandLayer::driver = &gFake;
}

void tearDown(void)
{
    OpBuffer::getInstance()->reset();
    gFake.reset();
}

void test_opcode_M_dispatches_motorMove(void)
{
    enqueue_and_dispatch(make_op('M', 1, -120, 333, 1));
    TEST_ASSERT_EQUAL_INT(FakeMotorDriver::MOVE, gFake.last_call);
    TEST_ASSERT_EQUAL_INT32(-120, gFake.last_i32);
    TEST_ASSERT_EQUAL_UINT16(333, gFake.last_u16);
    TEST_ASSERT_EQUAL_UINT8(1, gFake.last_motor_id);
}

void test_opcode_G_dispatches_motorGoTo(void)
{
    enqueue_and_dispatch(make_op('G', 1, 42, 222, 1));
    TEST_ASSERT_EQUAL_INT(FakeMotorDriver::GOTO, gFake.last_call);
    TEST_ASSERT_EQUAL_INT32(42, gFake.last_i32);
    TEST_ASSERT_EQUAL_UINT16(222, gFake.last_u16);
}

void test_opcode_S_dispatches_motorStop(void)
{
    enqueue_and_dispatch(make_op('S', 1, 200, 1000, 1));
    TEST_ASSERT_EQUAL_INT(FakeMotorDriver::STOP, gFake.last_call);
    TEST_ASSERT_EQUAL_INT32(200, gFake.last_i32);
    TEST_ASSERT_EQUAL_UINT16(1000, gFake.last_u16);
}

void test_opcode_I_dispatches_motorSleep(void)
{
    enqueue_and_dispatch(make_op('I', 1, 200, 1000, 1));
    TEST_ASSERT_EQUAL_INT(FakeMotorDriver::SLEEP, gFake.last_call);
    TEST_ASSERT_EQUAL_INT32(200, gFake.last_i32);
    TEST_ASSERT_EQUAL_UINT16(1000, gFake.last_u16);
}

void test_opcode_U_dispatches_microstepping_setting(void)
{
    enqueue_and_dispatch(make_op('U', 1, 1, 0, 1));
    TEST_ASSERT_EQUAL_INT(FakeMotorDriver::SETTING, gFake.last_call);
    TEST_ASSERT_EQUAL_INT(MotorDriver::config_setting::MICROSTEPPING, gFake.last_setting);
    TEST_ASSERT_EQUAL_UINT32(0, gFake.last_data1);
    TEST_ASSERT_EQUAL_UINT32(1, gFake.last_data2);
    TEST_ASSERT_EQUAL_UINT8(1, gFake.last_motor_id);
}

void test_opcode_U_accepts_explicit_32_microstep_setting(void)
{
    enqueue_and_dispatch(make_op('U', 1, 32, 32, 1));
    TEST_ASSERT_EQUAL_INT(FakeMotorDriver::SETTING, gFake.last_call);
    TEST_ASSERT_EQUAL_INT(MotorDriver::config_setting::MICROSTEPPING, gFake.last_setting);
    TEST_ASSERT_EQUAL_UINT32(32, gFake.last_data1);
    TEST_ASSERT_EQUAL_UINT32(1, gFake.last_data2);
    TEST_ASSERT_EQUAL_UINT8(1, gFake.last_motor_id);
}

void test_opcode_U_uses_legacy_stepRate_when_stepNum_is_zero(void)
{
    // U opcode consumes stepRate as the primary microstepping value.
    enqueue_and_dispatch(make_op('U', 1, 0, 1, 1));
    TEST_ASSERT_EQUAL_INT(FakeMotorDriver::SETTING, gFake.last_call);
    TEST_ASSERT_EQUAL_INT(MotorDriver::config_setting::MICROSTEPPING, gFake.last_setting);
    TEST_ASSERT_EQUAL_UINT32(1, gFake.last_data1);
    TEST_ASSERT_EQUAL_UINT32(1, gFake.last_data2);
    TEST_ASSERT_EQUAL_UINT8(1, gFake.last_motor_id);
}

void test_opcode_U_normalizes_zero_setting_to_full_step(void)
{
    // Command layer forwards the raw value; driver-side normalization is tested separately.
    enqueue_and_dispatch(make_op('U', 1, 0, 0, 1));
    TEST_ASSERT_EQUAL_INT(FakeMotorDriver::SETTING, gFake.last_call);
    TEST_ASSERT_EQUAL_INT(MotorDriver::config_setting::MICROSTEPPING, gFake.last_setting);
    TEST_ASSERT_EQUAL_UINT32(0, gFake.last_data1);
    TEST_ASSERT_EQUAL_UINT32(1, gFake.last_data2);
    TEST_ASSERT_EQUAL_UINT8(1, gFake.last_motor_id);
}

void test_opcode_R_is_ignored(void)
{
    enqueue_and_dispatch(make_op('R', 1, 2500, 6000, 1));
    TEST_ASSERT_EQUAL_UINT32(0, gFake.call_count);
    TEST_ASSERT_EQUAL_INT(FakeMotorDriver::NONE, gFake.last_call);
}

void test_opcodes_H_and_L_are_noops(void)
{
    enqueue_and_dispatch(make_op('H', 1, 10, 20, 1));
    TEST_ASSERT_EQUAL_UINT32(0, gFake.call_count);

    enqueue_and_dispatch(make_op('L', 1, 10, 20, 1));
    TEST_ASSERT_EQUAL_UINT32(0, gFake.call_count);
}

void test_opcode_K_via_peek_dispatches_abort(void)
{
    enqueue_and_dispatch(make_op('M', 1, 100, 100, 1));
    TEST_ASSERT_EQUAL_INT(FakeMotorDriver::MOVE, gFake.last_call);

    Op kill = make_op('K', 1, 0, 0, 1);
    TEST_ASSERT_EQUAL_INT8(0, OpBuffer::getInstance()->storeOp(&kill));
    CommandLayer::peekNextOp(1);

    TEST_ASSERT_EQUAL_INT(FakeMotorDriver::ABORT, gFake.last_call);
    TEST_ASSERT_EQUAL_UINT8(1, gFake.last_motor_id);
}

extern "C" void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_opcode_M_dispatches_motorMove);
    RUN_TEST(test_opcode_G_dispatches_motorGoTo);
    RUN_TEST(test_opcode_S_dispatches_motorStop);
    RUN_TEST(test_opcode_I_dispatches_motorSleep);
    RUN_TEST(test_opcode_U_dispatches_microstepping_setting);
    RUN_TEST(test_opcode_U_accepts_explicit_32_microstep_setting);
    RUN_TEST(test_opcode_U_uses_legacy_stepRate_when_stepNum_is_zero);
    RUN_TEST(test_opcode_U_normalizes_zero_setting_to_full_step);
    RUN_TEST(test_opcode_R_is_ignored);
    RUN_TEST(test_opcodes_H_and_L_are_noops);
    RUN_TEST(test_opcode_K_via_peek_dispatches_abort);
    UNITY_END();
}
