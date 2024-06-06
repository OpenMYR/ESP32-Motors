#ifndef _CommandParser_H_
#define _CommandParser_H_

#include <functional>
#include <ArduinoJson.h>
#include <IPAddress.h>
#include "Op.h"

class CommandParser
{
public:
    static bool json_parseCommands(String, IPAddress);
    static void wifi_process_command(struct wifi_command_packet, IPAddress);
    static void motor_process_command(struct Op, IPAddress);
    static bool ota_active;
    static void register_udp_ack_func(std::function<void(command_response_packet &)>f);
    static void stop_motors();

private:
    static bool isValidCommand(JsonObject *cmd);
    static bool decodeCommand(JsonObject *cmd);
    static void parseConfig(JsonObject *cmd, char);
    static void parseMotorCommand(JsonObject *cmd, char code);
    static void parseMotorConfig(JsonObject *cmd, char code);
    static std::function<void(command_response_packet&)> ack_func;
};

#endif /* _CommandParser_H_ */