#ifndef _WifiController_H_
#define _WifiController_H_

#include <WString.h>
#include <String.h>
#include <Wifi.h>
#include <esp_wifi.h>
extern "C" {
#include <tcpip_adapter.h>
}

#define MYR_WIFI_STATE_INIT 0 
#define MYR_WIFI_STATE_AP 1 
#define MYR_WIFI_STATE_STA 2 
#define MYR_WIFI_STATE_STA_FAIL 3 
#define MYR_WIFI_STATE_STA_CONNECTING 4 
#define MYR_WIFI_STATE_ERROR 5 

class WifiController {
    public:
        static esp_err_t init();
        static esp_err_t setApCredentials(const String* ssid, const String* pass, bool save);
        static esp_err_t setStaCredentials(const String* ssid, const String* pass, bool save);
        static esp_err_t changeMode(uint8_t mode, bool save);
        static void changeOTAPass(const String* oldPass, const String* pass);
        static void event_handler(void *arg, system_event_t *event);
    private:
        static void generateSsid();
        static esp_err_t startListener();
        static void WiFiEvent(WiFiEvent_t);
        static esp_err_t startTCP();
        static esp_err_t saveValue(String id, const String* value);
        static esp_err_t saveValue(String id, uint8_t value);
        static esp_err_t changeModeToAp();
        static esp_err_t changeModeToSta();
        static esp_err_t changeModeToApSta();
};

#endif /* _WifiController_H_ */
