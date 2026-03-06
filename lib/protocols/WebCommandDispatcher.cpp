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

esp_err_t handle_config_command(char code, cJSON *data)
{
    if (code == 'D') return CommandParser::processWifiCommand(code, nullptr, nullptr);

    if (code == 'C' || code == 'O')
    {
        std::string lhs;
        std::string rhs;
        esp_err_t err = parse_config_pair(data, &lhs, &rhs);
        if (err != ESP_OK) return err;
        return CommandParser::processWifiCommand(code, &lhs, &rhs);
    }

    ESP_LOGD(TAG, "POST command '%c' ignored in IDF baseline", code);
    return ESP_OK;
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

esp_err_t handle_motor_motion_command(char code, cJSON *data)
{
    Op op = {};
    esp_err_t err = parse_motor_data(data, &op);
    if (err != ESP_OK) return err;
    if (op.stepRate == 0) return ESP_ERR_INVALID_ARG;

    op.opcode = code;
    return CommandParser::processMotorOp(op);
}

esp_err_t handle_motor_config_command(char code, cJSON *data)
{
    Op op = {};
    esp_err_t err = parse_motor_data(data, &op);
    if (err != ESP_OK) return err;

    op.opcode = code;
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

        const char opcode = code->valuestring[0];
        if (opcode == 'C' || opcode == 'D' || opcode == 'O')
        {
            err = handle_config_command(opcode, data);
        }
        else if (opcode == 'M' || opcode == 'S' || opcode == 'G' || opcode == 'I')
        {
            err = handle_motor_motion_command(opcode, data);
        }
        else if (opcode == 'U' || opcode == 'R' || opcode == 'H' || opcode == 'L')
        {
            err = handle_motor_config_command(opcode, data);
        }
        else
        {
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
    CommandParser::setWifiOpsForTest(ops);
}

void resetWifiOpsForTest()
{
    CommandParser::resetWifiOpsForTest();
}
} // namespace WebCommandDispatcher
