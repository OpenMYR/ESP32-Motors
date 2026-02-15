#ifndef MYR_WEB_COMMAND_DISPATCHER_H
#define MYR_WEB_COMMAND_DISPATCHER_H

#include <stdint.h>
#include <string>

#include "esp_err.h"

namespace WebCommandDispatcher {
struct WifiOps {
    esp_err_t (*tryConnectToSta)(const std::string *ssid, const std::string *pass);
    esp_err_t (*setDefaultStaCredentials)(const std::string *ssid, const std::string *pass);
    esp_err_t (*setDefaultMode)(uint16_t mode);
    void (*fireDisconnectEvent)();
    void (*changeOtaPass)(const std::string *old_pass, const std::string *new_pass);
};

esp_err_t processPayload(const char *payload);
void setWifiOpsForTest(const WifiOps *ops);
void resetWifiOpsForTest();
}

#endif // MYR_WEB_COMMAND_DISPATCHER_H
