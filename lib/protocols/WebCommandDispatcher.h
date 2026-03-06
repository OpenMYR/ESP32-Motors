#ifndef MYR_WEB_COMMAND_DISPATCHER_H
#define MYR_WEB_COMMAND_DISPATCHER_H

#include <stdint.h>

#include "CommandParser.h"
#include "esp_err.h"

namespace WebCommandDispatcher {
using WifiOps = CommandParser::WifiOps;

esp_err_t processPayload(const char *payload);
void setWifiOpsForTest(const WifiOps *ops);
void resetWifiOpsForTest();
}

#endif // MYR_WEB_COMMAND_DISPATCHER_H
