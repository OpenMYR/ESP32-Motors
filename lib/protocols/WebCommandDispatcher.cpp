#include "WebCommandDispatcher.h"

#include <string.h>

#include <string>

#include "OpBuffer.h"
#include "WifiController.h"
#include "cJSON.h"
#include "config/Config.h"
#include "esp_log.h"

namespace {
const char *TAG = "WebCmdDispatch";

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

void wifi_change_ota_pass(const std::string *old_pass, const std::string *new_pass)
{
    WifiController::changeOTAPass(old_pass, new_pass);
}

const WebCommandDispatcher::WifiOps kDefaultWifiOps = {
    wifi_try_connect,
    wifi_set_sta_credentials,
    wifi_set_default_mode,
    wifi_fire_disconnect,
    wifi_change_ota_pass,
};

WebCommandDispatcher::WifiOps gWifiOps = kDefaultWifiOps;

esp_err_t parse_config_pair(cJSON *data, std::string *lhs, std::string *rhs) {
    if (!cJSON_IsArray(data)) return ESP_ERR_INVALID_ARG;
    if (cJSON_GetArraySize(data) != 2) return ESP_ERR_INVALID_ARG;

    cJSON *lhs_item = cJSON_GetArrayItem(data, 0);
    cJSON *rhs_item = cJSON_GetArrayItem(data, 1);
    if (!cJSON_IsString(lhs_item) || lhs_item->valuestring == nullptr) return ESP_ERR_INVALID_ARG;
    if (!cJSON_IsString(rhs_item) || rhs_item->valuestring == nullptr) return ESP_ERR_INVALID_ARG;

    *lhs = lhs_item->valuestring;
    *rhs = rhs_item->valuestring;
    return ESP_OK;
}

esp_err_t handle_config_command(char code, cJSON *data) {
    if (code == 'C') {
        std::string ssid;
        std::string pass;
        esp_err_t err = parse_config_pair(data, &ssid, &pass);
        if (err != ESP_OK) return err;

        err = gWifiOps.tryConnectToSta(&ssid, &pass);
        if (err != ESP_OK) return err;

        err = gWifiOps.setDefaultStaCredentials(&ssid, &pass);
        if (err != ESP_OK) return err;

        return gWifiOps.setDefaultMode(MYR_WIFI_MODE_STATION);
    }

    if (code == 'D') {
        gWifiOps.fireDisconnectEvent();
        return gWifiOps.setDefaultMode(MYR_WIFI_MODE_AP);
    }

    if (code == 'O') {
        std::string old_pass;
        std::string new_pass;
        esp_err_t err = parse_config_pair(data, &old_pass, &new_pass);
        if (err != ESP_OK) return err;

        gWifiOps.changeOtaPass(&old_pass, &new_pass);
        return ESP_OK;
    }

    ESP_LOGD(TAG, "POST command '%c' ignored in IDF baseline", code);
    return ESP_OK;
}

esp_err_t parse_motor_data(cJSON *data, Op *op) {
    if (!cJSON_IsArray(data)) return ESP_ERR_INVALID_ARG;
    if (cJSON_GetArraySize(data) != 4) return ESP_ERR_INVALID_ARG;

    cJSON *motorIdItem = cJSON_GetArrayItem(data, 0);
    cJSON *queueItem = cJSON_GetArrayItem(data, 1);
    cJSON *stepNumItem = cJSON_GetArrayItem(data, 2);
    cJSON *stepRateItem = cJSON_GetArrayItem(data, 3);
    if (!cJSON_IsNumber(motorIdItem)) return ESP_ERR_INVALID_ARG;
    if (!cJSON_IsNumber(queueItem)) return ESP_ERR_INVALID_ARG;
    if (!cJSON_IsNumber(stepNumItem)) return ESP_ERR_INVALID_ARG;
    if (!cJSON_IsNumber(stepRateItem)) return ESP_ERR_INVALID_ARG;

    op->port = 0;
    op->motorID = static_cast<uint8_t>(motorIdItem->valueint);
    op->queue = static_cast<uint8_t>(queueItem->valueint);
    op->stepNum = static_cast<int32_t>(stepNumItem->valueint);
    op->stepRate = static_cast<uint16_t>(stepRateItem->valueint);
    return ESP_OK;
}

esp_err_t enqueue_motor_op(const Op *op) {
    OpBuffer *buffer = OpBuffer::getInstance();

    if (op->queue == 0) {
        buffer->clear(op->motorID);
        buffer->killCurrentOp(op->motorID);
    }

    Op opCopy = *op;
    if (buffer->storeOp(&opCopy) < 0) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t handle_motor_motion_command(char code, cJSON *data) {
    Op op = {};
    esp_err_t err = parse_motor_data(data, &op);
    if (err != ESP_OK) return err;

    op.opcode = code;
    return enqueue_motor_op(&op);
}

esp_err_t handle_motor_config_command(char code, cJSON *data) {
    Op op = {};
    esp_err_t err = parse_motor_data(data, &op);
    if (err != ESP_OK) return err;

    op.opcode = code;
    return enqueue_motor_op(&op);
}
} // namespace

namespace WebCommandDispatcher {
esp_err_t processPayload(const char *payload) {
    cJSON *root = cJSON_Parse(payload);
    if (root == nullptr) return ESP_ERR_INVALID_ARG;

    cJSON *commands = cJSON_GetObjectItemCaseSensitive(root, "commands");
    if (!cJSON_IsArray(commands)) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    cJSON *command = nullptr;
    cJSON_ArrayForEach(command, commands) {
        cJSON *code = cJSON_GetObjectItemCaseSensitive(command, "code");
        cJSON *data = cJSON_GetObjectItemCaseSensitive(command, "data");

        if (!cJSON_IsString(code) || code->valuestring == nullptr) {
            err = ESP_ERR_INVALID_ARG;
            break;
        }
        if (strlen(code->valuestring) != 1) {
            err = ESP_ERR_INVALID_ARG;
            break;
        }

        const char opcode = code->valuestring[0];
        if (opcode == 'C' || opcode == 'D' || opcode == 'O') {
            err = handle_config_command(opcode, data);
        } else if (opcode == 'M' || opcode == 'S' || opcode == 'G' || opcode == 'I') {
            err = handle_motor_motion_command(opcode, data);
        } else if (opcode == 'U' || opcode == 'R' || opcode == 'H' || opcode == 'L') {
            err = handle_motor_config_command(opcode, data);
        } else {
            ESP_LOGD(TAG, "POST command '%c' ignored", opcode);
            err = ESP_OK;
        }
        if (err != ESP_OK) break;
    }

    cJSON_Delete(root);
    return err;
}

void setWifiOpsForTest(const WifiOps *ops)
{
    if (ops != nullptr) {
        gWifiOps = *ops;
    }
}

void resetWifiOpsForTest()
{
    gWifiOps = kDefaultWifiOps;
}
} // namespace WebCommandDispatcher
