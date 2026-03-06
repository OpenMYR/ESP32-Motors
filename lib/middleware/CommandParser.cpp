#include "config/Config.h"

#include <esp_log.h>
#include <string>

#include "CommandParser.h"
#include "OpBuffer.h"
#include "Op.h"
#include "WifiController.h"
#include "CommandLayer.h"

const char *TAG = "CommandParser";

OpBuffer *buffer = OpBuffer::getInstance();
std::function<void(command_response_packet &)> CommandParser::ack_func;
bool CommandParser::ota_active = false;

void CommandParser::motor_process_command(struct Op packet, ip4_addr_t addr)
{
    (void)addr;

    if (ota_active)
        return;

    if (packet.queue == 0)
    {
        //kill queue and active command.
        buffer->clear(packet.motorID);
        buffer->killCurrentOp(packet.motorID);
    }

    //CommandLayer::opcodeGoto(dataOne, dataTwo, motor_id);
    if (buffer->storeOp(&packet) >= 0)
    {
        ESP_LOGV(TAG, "queued stepNum=%ld", static_cast<long>(packet.stepNum));
    }
    else
    {
        ESP_LOGE(TAG, "queue store failed stepNum=%ld", static_cast<long>(packet.stepNum));
    }
}

void CommandParser::wifi_process_command(struct wifi_command_packet packet, ip4_addr_t addr)
{
    (void)addr;

    if (ota_active)
        return;

    std::string lhs = packet.ssid;
    std::string rhs = packet.password;
    esp_err_t err = ESP_OK;

    if (packet.opcode == 'C')
    {
        err = WifiController::tryConnectToSta(&lhs, &rhs);
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        if (err != ESP_OK) return;

        err = WifiController::setDefaultStaCredentials(&lhs, &rhs);
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        if (err != ESP_OK) return;

        err = WifiController::setDefaultMode(MYR_WIFI_MODE_STATION);
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    }
    else if (packet.opcode == 'D')
    {
        WifiController::fireWifiEvent(WifiController::MYR_WIFI_EVENT_DISCONNECT, nullptr);
        err = WifiController::setDefaultMode(MYR_WIFI_MODE_AP);
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    }
    else if (packet.opcode == 'O')
    {
        err = WifiController::changeOTAPass(&lhs, &rhs);
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    }
    else
    {
        ESP_LOGW(TAG, "unsupported wifi opcode '%c'", packet.opcode);
    }
}

void CommandParser::register_udp_ack_func(std::function<void(command_response_packet &)> f)
{
    ack_func = f;
}

void CommandParser::stop_motors()
{
    uint16_t maxMotorCount = 16;
    for (int id = 0; id < maxMotorCount; id++)
    {
        buffer->clear(id);
        buffer->killCurrentOp(id);
    }
    ota_active = true;
}
