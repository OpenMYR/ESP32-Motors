#include <unity.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "CommandLayer.h"
#include "OpBuffer.h"
#include "StepperDriver.h"
#include "WebServer.h"

namespace {
void assert_uint64_equal(uint64_t expected, uint64_t actual, const char *message)
{
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(static_cast<uint32_t>(expected >> 32), static_cast<uint32_t>(actual >> 32), message);
    TEST_ASSERT_EQUAL_UINT32_MESSAGE(static_cast<uint32_t>(expected), static_cast<uint32_t>(actual), message);
}

class FakeMotorDriver final : public MotorDriver {
public:
    void reset()
    {
        moveCalled = false;
        gotoCalled = false;
        stopCalled = false;
        sleepCalled = false;
        configCalled = false;
        lastStepNum = 0;
        lastStepRate = 0;
        lastMotorId = 0;
        callCount = 0;
    }

    void pushCall(char code)
    {
        if (callCount < static_cast<int>(sizeof(callOrder))) {
            callOrder[callCount++] = code;
        }
    }

    void motorMove(int32_t step_num, uint16_t step_rate, uint8_t motor_id) override
    {
        moveCalled = true;
        lastStepNum = step_num;
        lastStepRate = step_rate;
        lastMotorId = motor_id;
        pushCall('M');
    }

    void motorGoTo(int32_t step_num, uint16_t step_rate, uint8_t motor_id) override
    {
        gotoCalled = true;
        lastStepNum = step_num;
        lastStepRate = step_rate;
        lastMotorId = motor_id;
        pushCall('G');
    }

    void motorStop(signed int wait_time, unsigned short precision, uint8_t motor_id) override
    {
        stopCalled = true;
        lastStepNum = wait_time;
        lastStepRate = precision;
        lastMotorId = motor_id;
        pushCall('S');
    }

    void motorSleep(signed int wait_time, unsigned short precision, uint8_t motor_id) override
    {
        sleepCalled = true;
        lastStepNum = wait_time;
        lastStepRate = precision;
        lastMotorId = motor_id;
        pushCall('I');
    }

    void changeMotorSettings(config_setting setting, uint32_t data1, uint32_t data2, uint8_t motor_id) override
    {
        (void)setting;
        (void)data1;
        (void)data2;
        configCalled = true;
        lastMotorId = motor_id;
        pushCall('U');
    }

    bool moveCalled = false;
    bool gotoCalled = false;
    bool stopCalled = false;
    bool sleepCalled = false;
    bool configCalled = false;
    int32_t lastStepNum = 0;
    uint16_t lastStepRate = 0;
    uint8_t lastMotorId = 0;
    char callOrder[16] = {};
    int callCount = 0;
};

FakeMotorDriver gFakeDriver;

struct ConcurrentDrainContext {
    volatile bool done = false;
};

void drain_queue_task(void *arg)
{
    auto *ctx = static_cast<ConcurrentDrainContext *>(arg);
    for (int i = 0; i < 600; ++i) {
        CommandLayer::getNextOp(1);
        if (gFakeDriver.callCount >= 4) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    ctx->done = true;
    vTaskDelete(nullptr);
}

void wait_for_monitor_attach(void)
{
    for (int i = 0; i < 16; ++i) {
        printf("test_web_command_ingest bootstrap %d/16\n", i + 1);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
} // namespace

void setUp(void)
{
    OpBuffer::getInstance()->reset();
    gFakeDriver.reset();
    CommandLayer::driver = &gFakeDriver;
}

void tearDown(void)
{
}

void test_sleep_command_payload_dispatches_wait_and_precision(void)
{
    const char *payload = "{\"commands\":[{\"code\":\"I\",\"data\":[1,1,1375,777]}]}";
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandIngest::processCommandPayload(payload));

    CommandLayer::getNextOp(1);

    TEST_ASSERT_TRUE(gFakeDriver.sleepCalled);
    TEST_ASSERT_FALSE(gFakeDriver.stopCalled);
    TEST_ASSERT_FALSE(gFakeDriver.moveCalled);
    TEST_ASSERT_FALSE(gFakeDriver.gotoCalled);
    TEST_ASSERT_EQUAL_INT32(1375, gFakeDriver.lastStepNum);
    TEST_ASSERT_EQUAL_UINT16(777, gFakeDriver.lastStepRate);
    TEST_ASSERT_EQUAL_UINT8(1, gFakeDriver.lastMotorId);
}

void test_sleep_command_payload_maps_to_expected_dwell_duration(void)
{
    const char *payload = "{\"commands\":[{\"code\":\"I\",\"data\":[1,1,5000,1000]}]}";
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandIngest::processCommandPayload(payload));

    CommandLayer::getNextOp(1);

    TEST_ASSERT_TRUE(gFakeDriver.sleepCalled);
    TEST_ASSERT_EQUAL_INT32(5000, gFakeDriver.lastStepNum);
    TEST_ASSERT_EQUAL_UINT16(1000, gFakeDriver.lastStepRate);
    assert_uint64_equal(
        5000000ULL,
        StepperDriver::planDwellDurationUs(gFakeDriver.lastStepNum, gFakeDriver.lastStepRate),
        "sleep payload dwell duration");
}

void test_stop_command_queue_zero_inserts_kill_before_dispatch(void)
{
    const char *payload = "{\"commands\":[{\"code\":\"S\",\"data\":[1,0,250,1000]}]}";
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandIngest::processCommandPayload(payload));

    // First fetch consumes injected kill op ('K') and should not dispatch to driver.
    CommandLayer::getNextOp(1);
    TEST_ASSERT_FALSE(gFakeDriver.stopCalled);
    TEST_ASSERT_FALSE(gFakeDriver.sleepCalled);
    TEST_ASSERT_FALSE(gFakeDriver.moveCalled);
    TEST_ASSERT_FALSE(gFakeDriver.gotoCalled);

    // Second fetch should dispatch the queued stop command with intact payload.
    CommandLayer::getNextOp(1);
    TEST_ASSERT_TRUE(gFakeDriver.stopCalled);
    TEST_ASSERT_EQUAL_INT32(250, gFakeDriver.lastStepNum);
    TEST_ASSERT_EQUAL_UINT16(1000, gFakeDriver.lastStepRate);
    TEST_ASSERT_EQUAL_UINT8(1, gFakeDriver.lastMotorId);
}

void test_sequence_u_m_i_m_dispatch_order_is_preserved(void)
{
    const char *payload =
        "{\"commands\":["
        "{\"code\":\"U\",\"data\":[1,0,0,0]},"
        "{\"code\":\"M\",\"data\":[1,1,100,100]},"
        "{\"code\":\"I\",\"data\":[1,1,5000,1000]},"
        "{\"code\":\"M\",\"data\":[1,1,100,100]}"
        "]}";
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandIngest::processCommandPayload(payload));

    for (int i = 0; i < 6; ++i) {
        CommandLayer::getNextOp(1);
    }

    TEST_ASSERT_EQUAL_INT(4, gFakeDriver.callCount);
    TEST_ASSERT_EQUAL_CHAR('U', gFakeDriver.callOrder[0]);
    TEST_ASSERT_EQUAL_CHAR('M', gFakeDriver.callOrder[1]);
    TEST_ASSERT_EQUAL_CHAR('I', gFakeDriver.callOrder[2]);
    TEST_ASSERT_EQUAL_CHAR('M', gFakeDriver.callOrder[3]);
}

void test_sequence_u_m_i_m_dispatch_order_is_preserved_under_concurrent_drain(void)
{
    const char *payload =
        "{\"commands\":["
        "{\"code\":\"U\",\"data\":[1,0,0,0]},"
        "{\"code\":\"M\",\"data\":[1,1,100,100]},"
        "{\"code\":\"I\",\"data\":[1,1,5000,1000]},"
        "{\"code\":\"M\",\"data\":[1,1,100,100]}"
        "]}";

    ConcurrentDrainContext ctx = {};
    TaskHandle_t drainTask = nullptr;
    BaseType_t createOk = xTaskCreate(
        drain_queue_task,
        "testDrainQueue",
        4096,
        &ctx,
        tskIDLE_PRIORITY + 1,
        &drainTask);
    TEST_ASSERT_EQUAL(pdPASS, createOk);

    TEST_ASSERT_EQUAL(ESP_OK, WebCommandIngest::processCommandPayload(payload));

    for (int i = 0; i < 800 && !ctx.done; ++i) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    TEST_ASSERT_TRUE(ctx.done);

    TEST_ASSERT_EQUAL_INT(4, gFakeDriver.callCount);
    TEST_ASSERT_EQUAL_CHAR('U', gFakeDriver.callOrder[0]);
    TEST_ASSERT_EQUAL_CHAR('M', gFakeDriver.callOrder[1]);
    TEST_ASSERT_EQUAL_CHAR('I', gFakeDriver.callOrder[2]);
    TEST_ASSERT_EQUAL_CHAR('M', gFakeDriver.callOrder[3]);
}

extern "C" void app_main(void)
{
    wait_for_monitor_attach();
    UNITY_BEGIN();
    RUN_TEST(test_sleep_command_payload_dispatches_wait_and_precision);
    RUN_TEST(test_sleep_command_payload_maps_to_expected_dwell_duration);
    RUN_TEST(test_stop_command_queue_zero_inserts_kill_before_dispatch);
    RUN_TEST(test_sequence_u_m_i_m_dispatch_order_is_preserved);
    RUN_TEST(test_sequence_u_m_i_m_dispatch_order_is_preserved_under_concurrent_drain);
    UNITY_END();
}
