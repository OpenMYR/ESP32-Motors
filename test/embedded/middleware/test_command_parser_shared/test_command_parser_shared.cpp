#include <string>
#include <cstring>

#include <unity.h>

#include "CommandParser.h"
#include "Op.h"
#include "OpBuffer.h"
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
    CommandParser::WifiOps ops = {};
    ops.tryConnectToSta = fake_try_connect;
    ops.setDefaultStaCredentials = fake_set_sta_credentials;
    ops.setDefaultMode = fake_set_default_mode;
    ops.fireDisconnectEvent = fake_fire_disconnect_event;
    ops.changeOtaPass = fake_change_ota_pass;
    CommandParser::setWifiOpsForTest(&ops);
}
} // namespace

void setUp(void)
{
    OpBuffer::getInstance()->reset();
    gWifi = FakeWifiState();
    CommandParser::ota_active = false;
    configure_fake_wifi();
}

void tearDown(void)
{
    OpBuffer::getInstance()->reset();
    CommandParser::resetWifiOpsForTest();
    CommandParser::ota_active = false;
}

void test_processMotorOp_queue_zero_kills_then_enqueues_new_command(void)
{
    Op op = {};
    op.opcode = 'G';
    op.motorID = 1;
    op.queue = 0;
    op.stepNum = 123;
    op.stepRate = 456;

    TEST_ASSERT_EQUAL(ESP_OK, CommandParser::processMotorOp(op));

    Op *kill_op = next_op(1);
    TEST_ASSERT_NOT_NULL(kill_op);
    TEST_ASSERT_EQUAL_CHAR('K', kill_op->opcode);

    Op *cmd_op = next_op(1);
    TEST_ASSERT_NOT_NULL(cmd_op);
    TEST_ASSERT_EQUAL_CHAR('G', cmd_op->opcode);
    TEST_ASSERT_EQUAL_INT32(123, cmd_op->stepNum);
    TEST_ASSERT_EQUAL_UINT16(456, cmd_op->stepRate);
}

void test_processWifiCommand_connect_runs_in_expected_order(void)
{
    std::string ssid = "ssid-a";
    std::string pass = "pass-a";

    TEST_ASSERT_EQUAL(ESP_OK, CommandParser::processWifiCommand(WifiOpcode::Connect, &ssid, &pass));
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.connect_calls);
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.creds_calls);
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.mode_calls);
    TEST_ASSERT_EQUAL_UINT16(MYR_WIFI_MODE_STATION, gWifi.last_mode);
    TEST_ASSERT_EQUAL_STRING("ssid-a", gWifi.last_ssid.c_str());
    TEST_ASSERT_EQUAL_STRING("pass-a", gWifi.last_pass.c_str());
}

void test_processWifiCommand_disconnect_fires_event_and_sets_ap_mode(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, CommandParser::processWifiCommand(WifiOpcode::Disconnect, nullptr, nullptr));
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.disconnect_calls);
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.mode_calls);
    TEST_ASSERT_EQUAL_UINT16(MYR_WIFI_MODE_AP, gWifi.last_mode);
}

void test_processWifiCommand_change_password_requires_two_strings(void)
{
    std::string old_pass = "old-pass";
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, CommandParser::processWifiCommand(WifiOpcode::ChangeOtaPassword, &old_pass, nullptr));
    TEST_ASSERT_EQUAL_UINT32(0, gWifi.ota_pass_calls);
}

void test_wifi_process_command_accepts_max_length_fields_without_null_terminator(void)
{
    wifi_command_packet packet = {};
    packet.opcode = 'C';
    memset(packet.ssid, 's', sizeof(packet.ssid));
    memset(packet.password, 'p', sizeof(packet.password));
    const std::string expected_ssid(sizeof(packet.ssid), 's');
    const std::string expected_pass(sizeof(packet.password), 'p');
>>>>>>> d68c273 (accept stop and sleep with zero rate)

    ip4_addr_t addr = {};
    CommandParser::wifi_process_command(packet, addr);

    TEST_ASSERT_EQUAL_UINT32(1, gWifi.connect_calls);
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.creds_calls);
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.mode_calls);
    TEST_ASSERT_EQUAL_STRING_LEN(expected_ssid.c_str(), gWifi.last_ssid.c_str(), expected_ssid.size());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(expected_ssid.size()), static_cast<int>(gWifi.last_ssid.size()));
    TEST_ASSERT_EQUAL_STRING_LEN(expected_pass.c_str(), gWifi.last_pass.c_str(), expected_pass.size());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(expected_pass.size()), static_cast<int>(gWifi.last_pass.size()));
}

void test_wifi_process_command_truncates_at_first_null_in_fixed_fields(void)
{
    wifi_command_packet packet = {};
    packet.opcode = 'C';
    memset(packet.ssid, 'x', sizeof(packet.ssid));
    memset(packet.password, 'y', sizeof(packet.password));
    memcpy(packet.ssid, "ssid", 4);
    packet.ssid[4] = '\0';
    memcpy(packet.password, "pass", 4);
    packet.password[4] = '\0';

    ip4_addr_t addr = {};
    CommandParser::wifi_process_command(packet, addr);

    TEST_ASSERT_EQUAL_UINT32(1, gWifi.connect_calls);
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.creds_calls);
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.mode_calls);
    TEST_ASSERT_EQUAL_STRING("ssid", gWifi.last_ssid.c_str());
    TEST_ASSERT_EQUAL_INT(4, static_cast<int>(gWifi.last_ssid.size()));
    TEST_ASSERT_EQUAL_STRING("pass", gWifi.last_pass.c_str());
    TEST_ASSERT_EQUAL_INT(4, static_cast<int>(gWifi.last_pass.size()));
}

void test_wifi_process_command_ssid_without_null_stays_within_ssid_field(void)
{
    wifi_command_packet packet = {};
    packet.opcode = 'C';
    memset(packet.ssid, 's', sizeof(packet.ssid));
    memset(packet.password, 'q', sizeof(packet.password));
    packet.password[3] = '\0';
    const std::string expected_ssid(sizeof(packet.ssid), 's');

    ip4_addr_t addr = {};
    CommandParser::wifi_process_command(packet, addr);

    TEST_ASSERT_EQUAL_UINT32(1, gWifi.connect_calls);
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.creds_calls);
    TEST_ASSERT_EQUAL_UINT32(1, gWifi.mode_calls);
    TEST_ASSERT_EQUAL_STRING_LEN(expected_ssid.c_str(), gWifi.last_ssid.c_str(), expected_ssid.size());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(expected_ssid.size()), static_cast<int>(gWifi.last_ssid.size()));
}

void test_stop_all_motors_clears_and_kills_without_entering_ota_mode(void)
{
    Op queued = {};
    queued.opcode = 'M';
    queued.motorID = 2;
    queued.queue = 1;
    queued.stepNum = 10;
    queued.stepRate = 20;
    TEST_ASSERT_EQUAL(ESP_OK, CommandParser::processMotorOp(queued));

    CommandParser::stop_all_motors();

    Op *kill_op = next_op(2);
    TEST_ASSERT_NOT_NULL(kill_op);
    TEST_ASSERT_EQUAL_CHAR('K', kill_op->opcode);
    TEST_ASSERT_FALSE(CommandParser::ota_active);
}

void test_enter_ota_mode_stops_motors_and_blocks_future_commands(void)
{
    Op queued = {};
    queued.opcode = 'M';
    queued.motorID = 3;
    queued.queue = 1;
    queued.stepNum = 10;
    queued.stepRate = 20;
    TEST_ASSERT_EQUAL(ESP_OK, CommandParser::processMotorOp(queued));

    CommandParser::enter_ota_mode();

    Op *kill_op = next_op(3);
    TEST_ASSERT_NOT_NULL(kill_op);
    TEST_ASSERT_EQUAL_CHAR('K', kill_op->opcode);
    TEST_ASSERT_TRUE(CommandParser::ota_active);

    Op blocked = {};
    blocked.opcode = 'G';
    blocked.motorID = 3;
    blocked.queue = 1;
    blocked.stepNum = 44;
    blocked.stepRate = 55;
    TEST_ASSERT_EQUAL(ESP_OK, CommandParser::processMotorOp(blocked));
    TEST_ASSERT_TRUE(OpBuffer::getInstance()->isEmpty(3));
}

void test_exit_ota_mode_reenables_command_processing(void)
{
    CommandParser::enter_ota_mode();
    TEST_ASSERT_TRUE(CommandParser::ota_active);

    Op *queued_kill = next_op(4);
    TEST_ASSERT_NOT_NULL(queued_kill);
    TEST_ASSERT_EQUAL_CHAR('K', queued_kill->opcode);

    CommandParser::exit_ota_mode();
    TEST_ASSERT_FALSE(CommandParser::ota_active);

    Op op = {};
    op.opcode = 'M';
    op.motorID = 4;
    op.queue = 1;
    op.stepNum = 12;
    op.stepRate = 34;
    TEST_ASSERT_EQUAL(ESP_OK, CommandParser::processMotorOp(op));

    Op *queued_op = next_op(4);
    TEST_ASSERT_NOT_NULL(queued_op);
    TEST_ASSERT_EQUAL_CHAR('M', queued_op->opcode);
}

extern "C" void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_processMotorOp_queue_zero_kills_then_enqueues_new_command);
    RUN_TEST(test_processWifiCommand_connect_runs_in_expected_order);
    RUN_TEST(test_processWifiCommand_disconnect_fires_event_and_sets_ap_mode);
    RUN_TEST(test_processWifiCommand_change_password_requires_two_strings);
    RUN_TEST(test_wifi_process_command_accepts_max_length_fields_without_null_terminator);
    RUN_TEST(test_wifi_process_command_truncates_at_first_null_in_fixed_fields);
    RUN_TEST(test_wifi_process_command_ssid_without_null_stays_within_ssid_field);
    RUN_TEST(test_stop_all_motors_clears_and_kills_without_entering_ota_mode);
    RUN_TEST(test_enter_ota_mode_stops_motors_and_blocks_future_commands);
    RUN_TEST(test_exit_ota_mode_reenables_command_processing);
    UNITY_END();
}
