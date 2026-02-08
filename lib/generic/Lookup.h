#ifndef MYR_LOOKUP_H
#define MYR_LOOKUP_H

#include <stdint.h>

#include "esp_wifi_types.h"

namespace Lookup {
const char *wifiDisconnectReasonToString(uint8_t reason);
const char *wifiAuthModeToString(wifi_auth_mode_t authMode);
const char *wifiStateEventToString(uint8_t event);
const char *wifiStateToString(uint8_t state);
}

#endif // MYR_LOOKUP_H
