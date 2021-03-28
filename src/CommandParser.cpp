#if __has_include("config/LocalConfig.h")
#include "config/LocalConfig.h"
#else
#include "config/DefaultConfig.h"
#endif

#include "CommandParser.h"
#include "OpBuffer.h"
#include "Op.h"
#include "WifiController.h"
#include "CommandLayer.h"
#include "esp_log.h"

#include "esp_log.h"
String const TAG = "CommandParser";

StaticJsonDocument<2000> doc;
OpBuffer *buffer = OpBuffer::getInstance();
std::function<void(command_response_packet &)> CommandParser::ack_func;
bool CommandParser::ota_active = false;

void CommandParser::motor_process_command(struct Op packet, IPAddress addr)
{
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
        log_v("cool %d", packet.stepNum);
    }
    else
    {
        log_e("error %d", packet.stepNum);
    }
}

void CommandParser::wifi_process_command(struct wifi_command_packet packet, IPAddress addr)
{
    if (ota_active)
        return;

    if (packet.opcode == 'C')
    {
        //change_opmode(true, packet.ssid, packet.password);
    }
    else if (packet.opcode == 'D')
    {
        //change_opmode(false, "", "");
    }
    if (packet.opcode == 'O')
    {
        //change_ota_pass(packet.ssid, packet.password);
    }
}

bool CommandParser::json_parseCommands(String jsonString, IPAddress ip)
{
    bool return202Required = true;
    if (ota_active)
        return return202Required;

    log_v("Parsing json string of length %d", jsonString.length());
    log_v("%s", jsonString.c_str());

    DeserializationError error = deserializeJson(doc, jsonString);
    if (error)
    {
        log_e("deserializeJson() failed: %s", error.c_str());
        return return202Required;
    }

    JsonArray commandArray = doc["commands"];
    if (commandArray.isNull())
    {
        log_i("No commands to parse");
        return return202Required;
    }

    log_v("%d Commands found", commandArray.size());
    for (unsigned int i = 0; i < commandArray.size(); i++)
    {
        JsonObject cmd = commandArray[i];
        if (isValidCommand(&cmd))
        {
            return202Required = return202Required && decodeCommand(&cmd);
        }
    }
    return return202Required;
}

bool CommandParser::isValidCommand(JsonObject *cmd)
{
    return (!cmd->isNull() && cmd->containsKey("code") && cmd->containsKey("data"));
}

bool CommandParser::decodeCommand(JsonObject *cmd)
{
    const char *code = (*cmd)["code"];
    log_v("Code: %d", (int)(*code));
    switch (*code)
    {
    case 'C':
    case 'D':
    case 'O':
    {
        parseConfig(cmd, *code);
        return false; //If we change networks, we dont want to send a 200 code
        break;
    }
    case 'M':
    case 'S':
    case 'G':
    case 'I':
    {
        parseMotorCommand(cmd, *code);
        break;
    }
    case 'U':
    case 'H':
    case 'L':
    {
        parseMotorConfig(cmd, *code);
        break;
    }
    default:
        log_i("Unknown packet");
        break;
    }
    return true;
}

void CommandParser::parseConfig(JsonObject *cmd, char code)
{
    JsonArray dataArray = (*cmd)["data"];

    if (dataArray.size() != 2)
    {
        log_i("Malformed data array!");
        return;
    }

    String ssid = dataArray[0];
    String pass = dataArray[1];

    if(code == 'C') {
        WifiController::setStaCredentials(&ssid, &pass, true);
        WifiController::changeMode(MYR_WIFI_MODE_STATION, true);
    }
    else if(code == 'D') WifiController::changeMode(MYR_WIFI_MODE_AP, true);
    else if(code == 'O') WifiController::setStaCredentials(&ssid, &pass, true);
    else log_i("Unknown command %c", code);
}

void CommandParser::parseMotorCommand(JsonObject *cmd, char code)
{
    JsonArray dataArray = (*cmd)["data"];
    if (dataArray.size() != 4)
    {
        log_i("Malformed data array!");
        return;
    }

    uint32_t motor_id = dataArray[0];
    uint32_t queue = dataArray[1];
    int32_t dataOne = dataArray[2];
    uint32_t dataTwo = dataArray[3];

    Op parsed_motor_command;
    parsed_motor_command.port = 0;
    parsed_motor_command.motorID = motor_id;
    parsed_motor_command.opcode = code;
    parsed_motor_command.queue = queue;
    parsed_motor_command.stepNum = dataOne;
    parsed_motor_command.stepRate = dataTwo;

    if (parsed_motor_command.queue == 0)
    {
        //kill queue and active command.
        buffer->clear(parsed_motor_command.motorID);
        buffer->killCurrentOp(parsed_motor_command.motorID);
    }

    //CommandLayer::opcodeGoto(dataOne, dataTwo, motor_id);
    if (buffer->storeOp(&parsed_motor_command) >= 0)
    {
        log_v("cool %d", dataOne);
    }
    else
    {
        log_e("error %d", dataOne);
    }
}

void CommandParser::parseMotorConfig(JsonObject *cmd, char code)
{
    JsonArray dataArray = (*cmd)["data"];
    if (dataArray.size() != 4)
    {
        log_i("Malformed data array!");
        return;
    }

    uint32_t motor_id = dataArray[0];
    uint32_t queue = dataArray[1];
    uint32_t dataOne = dataArray[2];
    uint32_t dataTwo = dataArray[3];

    Op parsed_motor_command;
    parsed_motor_command.port = 0;
    parsed_motor_command.motorID = motor_id;
    parsed_motor_command.opcode = code;
    parsed_motor_command.queue = queue;
    parsed_motor_command.stepNum = 0;
    parsed_motor_command.stepRate = dataTwo;
    //uint8_t dummy_ip[4];
    //motor_process_command(parsed_motor_command, dummy_ip);

    if (parsed_motor_command.queue == 0)
    {
        //kill queue and active command.
        buffer->clear(parsed_motor_command.motorID);
        buffer->killCurrentOp(parsed_motor_command.motorID);
    }

    //CommandLayer::opcodeGoto(dataOne, dataTwo, motor_id);
    if (buffer->storeOp(&parsed_motor_command) >= 0)
    {
        log_v("cool %d", dataOne);
    }
    else
    {
        log_e("error %d", dataOne);
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