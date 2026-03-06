#ifndef MYR_COMMANDPARSER_H
#define MYR_COMMANDPARSER_H

#include <functional>
#include <stdint.h>

#include "lwip/ip4_addr.h"

#include "Op.h"

class CommandParser
{
public:
    static void wifi_process_command(struct wifi_command_packet, ip4_addr_t);
    static void motor_process_command(struct Op, ip4_addr_t);
    static bool ota_active;
    static void register_udp_ack_func(std::function<void(command_response_packet &)>f);
    static void stop_motors();

private:
    static std::function<void(command_response_packet&)> ack_func;
};

#endif // MYR_COMMANDPARSER_H
