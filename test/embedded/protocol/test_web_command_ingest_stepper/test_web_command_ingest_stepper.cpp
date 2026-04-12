#include <unity.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "CommandLayer.h"
#include "OpBuffer.h"
#include "StepperDriver.h"
#include "WebCommandDispatcher.h"
#include "esp_timer.h"

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
        for (int i = 0; i < static_cast<int>(sizeof(callOrder)); ++i) {
            callOrder[i] = 0;
            callStepNum[i] = 0;
            callStepRate[i] = 0;
            callMotorId[i] = 0;
        }
    }

    void pushCall(char code, int32_t stepNum, uint16_t stepRate, uint8_t motorId)
    {
        if (callCount < static_cast<int>(sizeof(callOrder))) {
            const int idx = callCount;
            callOrder[idx] = code;
            callStepNum[idx] = stepNum;
            callStepRate[idx] = stepRate;
            callMotorId[idx] = motorId;
            callCount = idx + 1;
        }
    }

    void motorMove(int32_t step_num, uint16_t step_rate, uint8_t motor_id) override
    {
        moveCalled = true;
        lastStepNum = step_num;
        lastStepRate = step_rate;
        lastMotorId = motor_id;
        pushCall('M', step_num, step_rate, motor_id);
    }

    void motorGoTo(int32_t step_num, uint16_t step_rate, uint8_t motor_id) override
    {
        gotoCalled = true;
        lastStepNum = step_num;
        lastStepRate = step_rate;
        lastMotorId = motor_id;
        pushCall('G', step_num, step_rate, motor_id);
    }

    void motorStop(signed int wait_time, unsigned short precision, uint8_t motor_id) override
    {
        stopCalled = true;
        lastStepNum = wait_time;
        lastStepRate = precision;
        lastMotorId = motor_id;
        pushCall('S', wait_time, precision, motor_id);
    }

    void motorSleep(signed int wait_time, unsigned short precision, uint8_t motor_id) override
    {
        sleepCalled = true;
        lastStepNum = wait_time;
        lastStepRate = precision;
        lastMotorId = motor_id;
        pushCall('I', wait_time, precision, motor_id);
    }

    void changeMotorSettings(config_setting setting, uint32_t data1, uint32_t data2, uint8_t motor_id) override
    {
        (void)setting;
        (void)data1;
        (void)data2;
        configCalled = true;
        lastMotorId = motor_id;
        pushCall('U', 0, 0, motor_id);
    }

    void isrStartIoDriver() override {}
    void isrStopIoDriver() override {}
    bool isMotorRunning(uint8_t motor_id) override { (void)motor_id; return false; }
    void setOpcodeContext(uint32_t op_seq, uint8_t motor_id) override { (void)op_seq; (void)motor_id; }
    void abortCommand(uint8_t motorID) override { (void)motorID; }
    bool isEndstopTripped(uint8_t motor_id) override { (void)motor_id; return false; }
    esp_err_t setEndstopTrippedPinSetting(uint8_t setting, uint8_t motor_id) override
    {
        (void)setting;
        (void)motor_id;
        return ESP_ERR_NOT_SUPPORTED;
    }
    bool getEndstopTrippedPinSetting(uint8_t motor_id) override { (void)motor_id; return false; }

    bool moveCalled = false;
    bool gotoCalled = false;
    bool stopCalled = false;
    bool sleepCalled = false;
    bool configCalled = false;
    int32_t lastStepNum = 0;
    uint16_t lastStepRate = 0;
    uint8_t lastMotorId = 0;
    char callOrder[16] = {};
    int32_t callStepNum[16] = {};
    uint16_t callStepRate[16] = {};
    uint8_t callMotorId[16] = {};
    int callCount = 0;

private:
    void initMotorGpio() override {}
    void driver() override {}
    void getNextOpForDriver(uint8_t id) override { (void)id; }
    void peekOpForDriver(uint8_t id) override { (void)id; }
};

FakeMotorDriver gFakeDriver;

struct ConcurrentDrainContext {
    volatile bool done = false;
};

bool gStepperIoStarted = false;

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
        printf("test_web_command_ingest_stepper bootstrap %d/16\n", i + 1);
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
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandDispatcher::processPayload(payload));

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
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandDispatcher::processPayload(payload));

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
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandDispatcher::processPayload(payload));

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

void test_motion_command_rejects_zero_step_rate(void)
{
    const char *payload = "{\"commands\":[{\"code\":\"M\",\"data\":[1,1,250,0]}]}";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, WebCommandDispatcher::processPayload(payload));
    TEST_ASSERT_TRUE(OpBuffer::getInstance()->isEmpty(1));
    TEST_ASSERT_FALSE(gFakeDriver.moveCalled);
    TEST_ASSERT_FALSE(gFakeDriver.gotoCalled);
    TEST_ASSERT_FALSE(gFakeDriver.stopCalled);
    TEST_ASSERT_FALSE(gFakeDriver.sleepCalled);
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
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandDispatcher::processPayload(payload));

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

    TEST_ASSERT_EQUAL(ESP_OK, WebCommandDispatcher::processPayload(payload));

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

void test_sequence_m_sssss_m_queue_duration_totals_2p05_seconds(void)
{
    const char *payload =
        "{\"commands\":["
        "{\"code\":\"M\",\"data\":[1,1,100,100]},"
        "{\"code\":\"S\",\"data\":[1,1,10,1000]},"
        "{\"code\":\"S\",\"data\":[1,1,10,1000]},"
        "{\"code\":\"S\",\"data\":[1,1,10,1000]},"
        "{\"code\":\"S\",\"data\":[1,1,10,1000]},"
        "{\"code\":\"S\",\"data\":[1,1,10,1000]},"
        "{\"code\":\"M\",\"data\":[1,1,100,100]}"
        "]}";
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandDispatcher::processPayload(payload));

    for (int i = 0; i < 7; ++i) {
        CommandLayer::getNextOp(1);
    }

    TEST_ASSERT_EQUAL_INT(7, gFakeDriver.callCount);
    TEST_ASSERT_EQUAL_CHAR('M', gFakeDriver.callOrder[0]);
    TEST_ASSERT_EQUAL_CHAR('S', gFakeDriver.callOrder[1]);
    TEST_ASSERT_EQUAL_CHAR('S', gFakeDriver.callOrder[2]);
    TEST_ASSERT_EQUAL_CHAR('S', gFakeDriver.callOrder[3]);
    TEST_ASSERT_EQUAL_CHAR('S', gFakeDriver.callOrder[4]);
    TEST_ASSERT_EQUAL_CHAR('S', gFakeDriver.callOrder[5]);
    TEST_ASSERT_EQUAL_CHAR('M', gFakeDriver.callOrder[6]);

    const uint64_t moveOneUs =
        StepperDriver::planRelativeMove(0, gFakeDriver.callStepNum[0], gFakeDriver.callStepRate[0]).durationUs;
    const uint64_t stopOneUs =
        StepperDriver::planDwellDurationUs(gFakeDriver.callStepNum[1], gFakeDriver.callStepRate[1]);
    const uint64_t stopTwoUs =
        StepperDriver::planDwellDurationUs(gFakeDriver.callStepNum[2], gFakeDriver.callStepRate[2]);
    const uint64_t stopThreeUs =
        StepperDriver::planDwellDurationUs(gFakeDriver.callStepNum[3], gFakeDriver.callStepRate[3]);
    const uint64_t stopFourUs =
        StepperDriver::planDwellDurationUs(gFakeDriver.callStepNum[4], gFakeDriver.callStepRate[4]);
    const uint64_t stopFiveUs =
        StepperDriver::planDwellDurationUs(gFakeDriver.callStepNum[5], gFakeDriver.callStepRate[5]);
    const uint64_t moveTwoUs =
        StepperDriver::planRelativeMove(gFakeDriver.callStepNum[0], gFakeDriver.callStepNum[6], gFakeDriver.callStepRate[6]).durationUs;
    const uint64_t totalUs = moveOneUs + stopOneUs + stopTwoUs + stopThreeUs + stopFourUs + stopFiveUs + moveTwoUs;

    assert_uint64_equal(1000000ULL, moveOneUs, "first move should be 1s");
    assert_uint64_equal(10000ULL, stopOneUs, "stop #1 should be 0.01s");
    assert_uint64_equal(10000ULL, stopTwoUs, "stop #2 should be 0.01s");
    assert_uint64_equal(10000ULL, stopThreeUs, "stop #3 should be 0.01s");
    assert_uint64_equal(10000ULL, stopFourUs, "stop #4 should be 0.01s");
    assert_uint64_equal(10000ULL, stopFiveUs, "stop #5 should be 0.01s");
    assert_uint64_equal(1000000ULL, moveTwoUs, "second move should be 1s");
    assert_uint64_equal(2050000ULL, totalUs, "queue M,S,S,S,S,S,M total should be 2.05s");
}

void test_sequence_m_sssss_m_runtime_duration_is_2p05_seconds(void)
{
    StepperDriver *stepper = StepperDriver::getInstance();
    CommandLayer::driver = stepper;

    if (!gStepperIoStarted) {
        stepper->isrStartIoDriver();
        gStepperIoStarted = true;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    TaskHandle_t motorTask = xTaskGetHandle("motorloopstep");
    TEST_ASSERT_NOT_NULL_MESSAGE(motorTask, "motorloopstep task handle is null");
    const eTaskState motorTaskState = eTaskGetState(motorTask);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(eDeleted, motorTaskState, "motorloopstep task was deleted");

    OpBuffer::getInstance()->reset();
    stepper->abortCommand(1);
    vTaskDelay(pdMS_TO_TICKS(20));
    TEST_ASSERT_FALSE_MESSAGE(StepperDriver::isEndstopTripped(), "endstop is active in runtime M,S,S,S,S,S,M timing test");

    const char *payload =
        "{\"commands\":["
        "{\"code\":\"M\",\"data\":[1,0,100,100]},"
        "{\"code\":\"S\",\"data\":[1,1,10,1000]},"
        "{\"code\":\"S\",\"data\":[1,1,10,1000]},"
        "{\"code\":\"S\",\"data\":[1,1,10,1000]},"
        "{\"code\":\"S\",\"data\":[1,1,10,1000]},"
        "{\"code\":\"S\",\"data\":[1,1,10,1000]},"
        "{\"code\":\"M\",\"data\":[1,1,100,100]}"
        "]}";
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandDispatcher::processPayload(payload));

    uint64_t startUs = esp_timer_get_time();
    bool started = false;
    for (int i = 0; i < 500; ++i) {
        if (stepper->isMotorRunning(1)) {
            startUs = esp_timer_get_time();
            started = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!started) {
        TEST_ASSERT_FALSE_MESSAGE(OpBuffer::getInstance()->isEmpty(1), "runtime M,S,S,S,S,S,M queue drained without entering running state");
        TEST_FAIL_MESSAGE("runtime M,S,S,S,S,S,M did not start");
    }

    bool completed = false;
    for (int i = 0; i < 1000; ++i) {
        if (!stepper->isMotorRunning(1) && OpBuffer::getInstance()->isEmpty(1)) {
            completed = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (!completed) {
        char status[128];
        snprintf(
            status,
            sizeof(status),
            "runtime M,S,S,S,S,S,M incomplete: running=%d queueEmpty=%d taskState=%d",
            stepper->isMotorRunning(1) ? 1 : 0,
            OpBuffer::getInstance()->isEmpty(1) ? 1 : 0,
            static_cast<int>(eTaskGetState(motorTask)));
        TEST_FAIL_MESSAGE(status);
    }

    const uint64_t elapsedUs = esp_timer_get_time() - startUs;
    // 2.05s target with tolerance for scheduler/task jitter on shared test hardware.
    TEST_ASSERT_TRUE_MESSAGE(elapsedUs >= 1950000ULL, "runtime M,S,S,S,S,S,M duration is shorter than expected");
    TEST_ASSERT_TRUE_MESSAGE(elapsedUs <= 2350000ULL, "runtime M,S,S,S,S,S,M duration is longer than expected");
}

extern "C" void app_main(void)
{
    wait_for_monitor_attach();
    UNITY_BEGIN();
    RUN_TEST(test_sleep_command_payload_dispatches_wait_and_precision);
    RUN_TEST(test_sleep_command_payload_maps_to_expected_dwell_duration);
    RUN_TEST(test_stop_command_queue_zero_inserts_kill_before_dispatch);
    RUN_TEST(test_motion_command_rejects_zero_step_rate);
    RUN_TEST(test_sequence_u_m_i_m_dispatch_order_is_preserved);
    RUN_TEST(test_sequence_u_m_i_m_dispatch_order_is_preserved_under_concurrent_drain);
    RUN_TEST(test_sequence_m_sssss_m_queue_duration_totals_2p05_seconds);
    RUN_TEST(test_sequence_m_sssss_m_runtime_duration_is_2p05_seconds);
    UNITY_END();
}
