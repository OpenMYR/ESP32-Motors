#include <unity.h>

#include "OpBuffer.h"
#include "WebCommandDispatcher.h"
#include "config/Config.h"

namespace {
struct FakeWifiState {
    esp_err_t connect_err = ESP_OK;
    esp_err_t creds_err = ESP_OK;
    esp_err_t mode_err = ESP_OK;
    esp_err_t ota_pass_err = ESP_OK;

    uint32_t connect_calls = 0;
    uint32_t creds_calls = 0;
    uint32_t mode_calls = 0;
    uint32_t disconnect_calls = 0;
    uint32_t ota_pass_calls = 0;

    uint16_t last_mode = 0;
    std::string last_ssid;
    std::string last_pass;
    std::string last_old_ota_pass;
    std::string last_new_ota_pass;
};

FakeWifiState gWifi;

Op *next_op(uint8_t motor_id)
{
    return OpBuffer::getInstance()->getOp(motor_id);
}

esp_err_t fake_try_connect(const std::string *ssid, const std::string *pass)
{
    gWifi.connect_calls++;
    gWifi.last_ssid = (ssid != nullptr) ? *ssid : "";
    gWifi.last_pass = (pass != nullptr) ? *pass : "";
    return gWifi.connect_err;
}

esp_err_t fake_set_sta_credentials(const std::string *ssid, const std::string *pass)
{
    gWifi.creds_calls++;
    gWifi.last_ssid = (ssid != nullptr) ? *ssid : "";
    gWifi.last_pass = (pass != nullptr) ? *pass : "";
    return gWifi.creds_err;
}

esp_err_t fake_set_default_mode(uint16_t mode)
{
    gWifi.mode_calls++;
    gWifi.last_mode = mode;
    return gWifi.mode_err;
}

void fake_fire_disconnect_event()
{
    gWifi.disconnect_calls++;
}

esp_err_t fake_change_ota_pass(const std::string *old_pass, const std::string *new_pass)
{
    gWifi.ota_pass_calls++;
    gWifi.last_old_ota_pass = (old_pass != nullptr) ? *old_pass : "";
    gWifi.last_new_ota_pass = (new_pass != nullptr) ? *new_pass : "";
    return gWifi.ota_pass_err;
}

void configure_fake_wifi()
{
    WebCommandDispatcher::WifiOps ops = {};
    ops.tryConnectToSta = fake_try_connect;
    ops.setDefaultStaCredentials = fake_set_sta_credentials;
    ops.setDefaultMode = fake_set_default_mode;
    ops.fireDisconnectEvent = fake_fire_disconnect_event;
    ops.changeOtaPass = fake_change_ota_pass;
    WebCommandDispatcher::setWifiOpsForTest(&ops);
}

} // namespace

void setUp(void)
{
    OpBuffer::getInstance()->reset();
    gWifi = FakeWifiState();
    configure_fake_wifi();
}

void tearDown(void)
{
    OpBuffer::getInstance()->reset();
    WebCommandDispatcher::resetWifiOpsForTest();
}

void test_processPayload_enqueues_motion_opcode(void)
{
    const char *payload = R"({"commands":[{"code":"M","data":[2,1,-90,300]}]})";
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandDispatcher::processPayload(payload));

    Op *op = next_op(2);
    TEST_ASSERT_NOT_NULL(op);
    TEST_ASSERT_EQUAL_CHAR('M', op->opcode);
    TEST_ASSERT_EQUAL_UINT8(2, op->motorID);
    TEST_ASSERT_EQUAL_UINT8(1, op->queue);
    TEST_ASSERT_EQUAL_INT32(-90, op->stepNum);
    TEST_ASSERT_EQUAL_UINT16(300, op->stepRate);
}

void test_processPayload_queue_zero_kills_then_enqueues_new_command(void)
{
    const char *payload = R"({"commands":[{"code":"G","data":[1,0,123,456]}]})";
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandDispatcher::processPayload(payload));

    Op *kill_op = next_op(1);
    TEST_ASSERT_NOT_NULL(kill_op);
    TEST_ASSERT_EQUAL_CHAR('K', kill_op->opcode);

    Op *cmd_op = next_op(1);
    TEST_ASSERT_NOT_NULL(cmd_op);
    TEST_ASSERT_EQUAL_CHAR('G', cmd_op->opcode);
    TEST_ASSERT_EQUAL_INT32(123, cmd_op->stepNum);
    TEST_ASSERT_EQUAL_UINT16(456, cmd_op->stepRate);
}

void test_processPayload_accepts_motor_config_opcodes(void)
{
    const char *payload = R"({"commands":[{"code":"U","data":[3,1,1,0]},{"code":"R","data":[3,1,5000,1200]}]})";
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandDispatcher::processPayload(payload));

    Op *microstep_op = next_op(3);
    TEST_ASSERT_NOT_NULL(microstep_op);
    TEST_ASSERT_EQUAL_CHAR('U', microstep_op->opcode);
    TEST_ASSERT_EQUAL_INT32(1, microstep_op->stepNum);

    Op *limits_op = next_op(3);
    TEST_ASSERT_NOT_NULL(limits_op);
    TEST_ASSERT_EQUAL_CHAR('R', limits_op->opcode);
    TEST_ASSERT_EQUAL_INT32(5000, limits_op->stepNum);
    TEST_ASSERT_EQUAL_UINT16(1200, limits_op->stepRate);
}

void test_processPayload_accepts_stop_and_sleep_with_zero_precision(void)
{
    const char *payload = R"({"commands":[{"code":"S","data":[4,1,50,0]},{"code":"I","data":[4,1,70,0]}]})";
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandDispatcher::processPayload(payload));

    Op *stop_op = next_op(4);
    TEST_ASSERT_NOT_NULL(stop_op);
    TEST_ASSERT_EQUAL_CHAR('S', stop_op->opcode);
    TEST_ASSERT_EQUAL_INT32(50, stop_op->stepNum);
    TEST_ASSERT_EQUAL_UINT16(0, stop_op->stepRate);

    Op *sleep_op = next_op(4);
    TEST_ASSERT_NOT_NULL(sleep_op);
    TEST_ASSERT_EQUAL_CHAR('I', sleep_op->opcode);
    TEST_ASSERT_EQUAL_INT32(70, sleep_op->stepNum);
    TEST_ASSERT_EQUAL_UINT16(0, sleep_op->stepRate);
}

void test_processPayload_rejects_move_with_zero_rate(void)
{
    const char *payload = R"({"commands":[{"code":"M","data":[2,1,-90,0]}]})";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, WebCommandDispatcher::processPayload(payload));
    TEST_ASSERT_TRUE(OpBuffer::getInstance()->isEmpty(2));
}

void test_processPayload_rejects_invalid_code_shape(void)
{
    const char *payload = R"({"commands":[{"code":"MM","data":[2,1,-90,300]}]})";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, WebCommandDispatcher::processPayload(payload));
    TEST_ASSERT_TRUE(OpBuffer::getInstance()->isEmpty(2));
}

void test_processPayload_rejects_invalid_data_shape(void)
{
    const char *payload = R"({"commands":[{"code":"M","data":[2,1,-90]}]})";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, WebCommandDispatcher::processPayload(payload));
    TEST_ASSERT_TRUE(OpBuffer::getInstance()->isEmpty(2));
}

void test_processPayload_rejects_motion_opcode_with_zero_step_rate(void)
{
    const char *payload = R"({"commands":[{"code":"M","data":[2,1,-90,0]}]})";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, WebCommandDispatcher::processPayload(payload));
    TEST_ASSERT_TRUE(OpBuffer::getInstance()->isEmpty(2));
}

void test_processPayload_ignores_unknown_opcode_and_keeps_processing(void)
{
    const char *payload = R"({"commands":[{"code":"Z","data":[0]},{"code":"S","data":[4,1,50,1]}]})";
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandDispatcher::processPayload(payload));

    Op *op = next_op(4);
    TEST_ASSERT_NOT_NULL(op);
    TEST_ASSERT_EQUAL_CHAR('S', op->opcode);
    TEST_ASSERT_EQUAL_INT32(50, op->stepNum);
}

void test_processPayload_config_C_returns_connect_error_without_followup_calls(void)
{
    gWifi.connect_err = ESP_ERR_INVALID_STATE;
    const char *payload = R"({"commands":[{"code":"C","data":["ssid-x","pass-x"]}]})";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, WebCommandDispatcher::processPayload(payload));
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.connect_calls);
    TEST_ASSERT_EQUAL_UINT32(0, gWifi.creds_calls);
    TEST_ASSERT_EQUAL_UINT32(0, gWifi.mode_calls);
}

void test_processPayload_config_C_returns_credentials_error(void)
{
    gWifi.creds_err = ESP_ERR_NO_MEM;
    const char *payload = R"({"commands":[{"code":"C","data":["ssid-y","pass-y"]}]})";
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, WebCommandDispatcher::processPayload(payload));
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.connect_calls);
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.creds_calls);
    TEST_ASSERT_EQUAL_UINT32(0, gWifi.mode_calls);
    TEST_ASSERT_EQUAL_STRING("ssid-y", gWifi.last_ssid.c_str());
}

void test_processPayload_config_D_fires_disconnect_and_propagates_mode_error(void)
{
    gWifi.mode_err = ESP_FAIL;
    const char *payload = R"({"commands":[{"code":"D","data":["unused","unused"]}]})";
    TEST_ASSERT_EQUAL(ESP_FAIL, WebCommandDispatcher::processPayload(payload));
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.disconnect_calls);
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.mode_calls);
    TEST_ASSERT_EQUAL_UINT16(MYR_WIFI_MODE_AP, gWifi.last_mode);
}

void test_processPayload_config_O_requires_two_strings(void)
{
    const char *payload = R"({"commands":[{"code":"O","data":["only-one"]}]})";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, WebCommandDispatcher::processPayload(payload));
    TEST_ASSERT_EQUAL_UINT32(0, gWifi.ota_pass_calls);
}

void test_processPayload_config_O_invokes_change_password(void)
{
    const char *payload = R"({"commands":[{"code":"O","data":["old-pass","new-pass"]}]})";
    TEST_ASSERT_EQUAL(ESP_OK, WebCommandDispatcher::processPayload(payload));
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.ota_pass_calls);
    TEST_ASSERT_EQUAL_STRING("old-pass", gWifi.last_old_ota_pass.c_str());
    TEST_ASSERT_EQUAL_STRING("new-pass", gWifi.last_new_ota_pass.c_str());
}

void test_processPayload_config_O_propagates_change_password_error(void)
{
    gWifi.ota_pass_err = ESP_ERR_INVALID_STATE;
    const char *payload = R"({"commands":[{"code":"O","data":["old-pass","new-pass"]}]})";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, WebCommandDispatcher::processPayload(payload));
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.ota_pass_calls);
    TEST_ASSERT_EQUAL_STRING("old-pass", gWifi.last_old_ota_pass.c_str());
    TEST_ASSERT_EQUAL_STRING("new-pass", gWifi.last_new_ota_pass.c_str());
}

extern "C" void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_processPayload_enqueues_motion_opcode);
    RUN_TEST(test_processPayload_queue_zero_kills_then_enqueues_new_command);
    RUN_TEST(test_processPayload_accepts_motor_config_opcodes);
    RUN_TEST(test_processPayload_accepts_stop_and_sleep_with_zero_precision);
    RUN_TEST(test_processPayload_rejects_move_with_zero_rate);
    RUN_TEST(test_processPayload_rejects_invalid_code_shape);
    RUN_TEST(test_processPayload_rejects_invalid_data_shape);
    RUN_TEST(test_processPayload_rejects_motion_opcode_with_zero_step_rate);
    RUN_TEST(test_processPayload_ignores_unknown_opcode_and_keeps_processing);
    RUN_TEST(test_processPayload_config_C_returns_connect_error_without_followup_calls);
    RUN_TEST(test_processPayload_config_C_returns_credentials_error);
    RUN_TEST(test_processPayload_config_D_fires_disconnect_and_propagates_mode_error);
    RUN_TEST(test_processPayload_config_O_requires_two_strings);
    RUN_TEST(test_processPayload_config_O_invokes_change_password);
    RUN_TEST(test_processPayload_config_O_propagates_change_password_error);
    UNITY_END();
}
