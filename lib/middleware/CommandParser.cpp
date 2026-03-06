#include "config/Config.h"

#include <esp_log.h>
#include <string>

#include "CommandParser.h"
#include "Op.h"
#include "OpBuffer.h"
#include "WifiController.h"

namespace {
const char *TAG = "CommandParser";

esp_err_t wifi_try_connect(const std::string *ssid, const std::string *pass)
{
    return WifiController::tryConnectToSta(ssid, pass);
}

esp_err_t wifi_set_sta_credentials(const std::string *ssid, const std::string *pass)
{
    return WifiController::setDefaultStaCredentials(ssid, pass);
}

esp_err_t wifi_set_default_mode(uint16_t mode)
{
    return WifiController::setDefaultMode(mode);
}

void wifi_fire_disconnect()
{
    WifiController::fireWifiEvent(WifiController::MYR_WIFI_EVENT_DISCONNECT, nullptr);
}

esp_err_t wifi_change_ota_pass(const std::string *old_pass, const std::string *new_pass)
{
    return WifiController::changeOTAPass(old_pass, new_pass);
}

const CommandParser::WifiOps kDefaultWifiOps = {
    wifi_try_connect,
    wifi_set_sta_credentials,
    wifi_set_default_mode,
    wifi_fire_disconnect,
    wifi_change_ota_pass,
};

OpBuffer *buffer = OpBuffer::getInstance();
} // namespace

std::function<void(command_response_packet &)> CommandParser::ack_func;
CommandParser::WifiOps CommandParser::wifiOps = kDefaultWifiOps;
bool CommandParser::ota_active = false;

void CommandParser::motor_process_command(struct Op packet, ip4_addr_t addr)
{
    (void)addr;
    ESP_ERROR_CHECK_WITHOUT_ABORT(processMotorOp(packet));
}

void CommandParser::wifi_process_command(struct wifi_command_packet packet, ip4_addr_t addr)
{
    (void)addr;

    std::string lhs = packet.ssid;
    std::string rhs = packet.password;
    ESP_ERROR_CHECK_WITHOUT_ABORT(processWifiCommand(packet.opcode, &lhs, &rhs));
}

esp_err_t CommandParser::processMotorOp(const Op &op)
{
    if (ota_active) return ESP_OK;

    if (op.queue == 0)
    {
        buffer->clear(op.motorID);
        buffer->killCurrentOp(op.motorID);
    }

    Op opCopy = op;
    if (buffer->storeOp(&opCopy) >= 0)
    {
        ESP_LOGV(TAG, "queued opcode=%c stepNum=%ld", op.opcode, static_cast<long>(op.stepNum));
        return ESP_OK;
    }

    ESP_LOGE(TAG, "queue store failed opcode=%c stepNum=%ld", op.opcode, static_cast<long>(op.stepNum));
    return ESP_FAIL;
}

esp_err_t CommandParser::processWifiCommand(char opcode, const std::string *lhs, const std::string *rhs)
{
    if (ota_active) return ESP_OK;

    if (opcode == 'C')
    {
        if (lhs == nullptr || rhs == nullptr) return ESP_ERR_INVALID_ARG;

        esp_err_t err = wifiOps.tryConnectToSta(lhs, rhs);
        if (err != ESP_OK) return err;

        err = wifiOps.setDefaultStaCredentials(lhs, rhs);
        if (err != ESP_OK) return err;

        return wifiOps.setDefaultMode(MYR_WIFI_MODE_STATION);
    }

    if (opcode == 'D')
    {
        wifiOps.fireDisconnectEvent();
        return wifiOps.setDefaultMode(MYR_WIFI_MODE_AP);
    }

    if (opcode == 'O')
    {
        if (lhs == nullptr || rhs == nullptr) return ESP_ERR_INVALID_ARG;
        return wifiOps.changeOtaPass(lhs, rhs);
    }

    ESP_LOGW(TAG, "unsupported wifi opcode '%c'", opcode);
    return ESP_OK;
}

void CommandParser::setWifiOpsForTest(const WifiOps *ops)
{
    if (ops != nullptr) wifiOps = *ops;
}

void CommandParser::resetWifiOpsForTest()
{
    wifiOps = kDefaultWifiOps;
}

void CommandParser::register_udp_ack_func(std::function<void(command_response_packet &)> f)
{
    ack_func = f;
}

void CommandParser::stop_all_motors()
{
    uint16_t maxMotorCount = 16;
    for (int id = 0; id < maxMotorCount; id++)
    {
        buffer->clear(id);
        buffer->killCurrentOp(id);
    }
}

void CommandParser::enter_ota_mode()
{
    stop_all_motors();
    ota_active = true;
}

void CommandParser::exit_ota_mode()
{
    ota_active = false;
}
