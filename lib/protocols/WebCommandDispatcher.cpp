#include "WebCommandDispatcher.h"

#include <string.h>

#include <string>

#include "CommandParser.h"
#include "Op.h"
#include "cJSON.h"
#include "esp_log.h"

namespace {
const char *TAG = "WebCmdDispatch";

esp_err_t parse_config_pair(cJSON *data, std::string *lhs, std::string *rhs)
{
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

esp_err_t handle_config_command(WifiOpcode opcode, cJSON *data)
{
    if (opcode == WifiOpcode::Disconnect) return CommandParser::processWifiCommand(opcode, nullptr, nullptr);

    switch (opcode)
    {
    case WifiOpcode::Connect:
    case WifiOpcode::ChangeOtaPassword:
    {
        std::string lhs;
        std::string rhs;
        esp_err_t err = parse_config_pair(data, &lhs, &rhs);
        if (err != ESP_OK) return err;
        return CommandParser::processWifiCommand(opcode, &lhs, &rhs);
    }
    default:
        ESP_LOGD(TAG, "POST command '%c' ignored in IDF baseline", to_char(opcode));
        return ESP_OK;
    }
}

esp_err_t parse_motor_data(cJSON *data, Op *op)
{
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

esp_err_t handle_motor_motion_command(MotorOpcode opcode, cJSON *data)
{
    Op op = {};
    esp_err_t err = parse_motor_data(data, &op);
    if (err != ESP_OK) return err;
    if (op.stepRate == 0 && opcode != MotorOpcode::Stop && opcode != MotorOpcode::Sleep) return ESP_ERR_INVALID_ARG;

    op.opcode = to_char(opcode);
    return CommandParser::processMotorOp(op);
}

esp_err_t handle_motor_config_command(MotorOpcode opcode, cJSON *data)
{
    Op op = {};
    esp_err_t err = parse_motor_data(data, &op);
    if (err != ESP_OK) return err;

    op.opcode = to_char(opcode);
    return CommandParser::processMotorOp(op);
}
} // namespace

namespace WebCommandDispatcher {
esp_err_t processPayload(const char *payload)
{
    cJSON *root = cJSON_Parse(payload);
    if (root == nullptr) return ESP_ERR_INVALID_ARG;

    cJSON *commands = cJSON_GetObjectItemCaseSensitive(root, "commands");
    if (!cJSON_IsArray(commands))
    {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    cJSON *command = nullptr;
    cJSON_ArrayForEach(command, commands)
    {
        cJSON *code = cJSON_GetObjectItemCaseSensitive(command, "code");
        cJSON *data = cJSON_GetObjectItemCaseSensitive(command, "data");

        if (!cJSON_IsString(code) || code->valuestring == nullptr)
        {
            err = ESP_ERR_INVALID_ARG;
            break;
        }
        if (strlen(code->valuestring) != 1)
        {
            err = ESP_ERR_INVALID_ARG;
            break;
        }

        const char rawOpcode = code->valuestring[0];
        WifiOpcode wifiOpcode = WifiOpcode::Connect;
        MotorOpcode motorOpcode = MotorOpcode::Move;
        if (try_parse_wifi_opcode(rawOpcode, &wifiOpcode))
        {
            err = handle_config_command(wifiOpcode, data);
        }
        else if (try_parse_motor_opcode(rawOpcode, &motorOpcode) && is_motion_opcode(motorOpcode))
        {
            err = handle_motor_motion_command(motorOpcode, data);
        }
        else if (try_parse_motor_opcode(rawOpcode, &motorOpcode) && is_motor_config_opcode(motorOpcode))
        {
            err = handle_motor_config_command(motorOpcode, data);
        }
        else
        {
            ESP_LOGD(TAG, "POST command '%c' ignored", rawOpcode);
            err = ESP_OK;
        }
        if (err != ESP_OK) break;
    }

    cJSON_Delete(root);
    return err;
}

void setWifiOpsForTest(const WifiOps *ops)
{
    CommandParser::setWifiOpsForTest(ops);
}

void resetWifiOpsForTest()
{
    CommandParser::resetWifiOpsForTest();
}
} // namespace WebCommandDispatcher
