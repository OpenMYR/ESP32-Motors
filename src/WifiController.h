#ifndef _WifiController_H_
#define _WifiController_H_

#include <WString.h>
#include <String.h>
#include <Wifi.h>
#include <esp_wifi.h>
extern "C" {
#include <tcpip_adapter.h>
#include "esp_netif.h"
}


typedef void (*VoidFunction) ();

ESP_EVENT_DECLARE_BASE(MYR_WIFI_EVENT_BASE);

/* Definitions state events */
typedef enum {
    MYR_WIFI_EVENT_CONN_NEW_STA             = 0,       /*!< Connection request to a new AP */
    MYR_WIFI_EVENT_DISCONNECT               = 1,       /*!< Disconnect request from current AP */
    MYR_WIFI_EVENT_CONNECTED                = 2,       /*!< Successfully connected to a new AP */
    MYR_WIFI_EVENT_CONNECTION_FAILED        = 3,       /*!< A connection has been lost or failed to establish */
    MYR_WIFI_EVENT_TIMER                    = 4       /*!< A time has passed */
} myr_wifi_event_t;

/* Definitions for states */
typedef enum {
    MYR_WIFI_STATE_AP                       = 0,       /*!< AP has started */
    MYR_WIFI_STATE_STA                      = 1,       /*!< Device has connected to an ap */
    MYR_WIFI_STATE_STA_CONNECTING           = 2,       /*!< Device is trying to connect to an ap */
    MYR_WIFI_STATE_AP_STA_CONNECTING        = 3,       /*!<  */
    MYR_WIFI_STATE_AP_STA_RAMPDOWN          = 4       /*!<  */
} myr_wifi_state_t;

class WifiController {
    public:
        static esp_err_t init();
        static void fireWifiEvent(myr_wifi_event_t event, void *data);
        //static void network_event_handler(void *arg, system_event_t *event);
        static void network_event_handler(void* arg, esp_event_base_t base, int32_t id, void* event_data);
        static void state_event_handler(void *arg, esp_event_base_t base, int32_t id, void* event_data);
        static esp_err_t tryConnectToSta(const String* ssid, const String* pass);

        static esp_err_t setDefaultApCredentials(const String* ssid, const String* pass);
        static esp_err_t setDefaultStaCredentials(const String* ssid, const String* pass);
        static esp_err_t setDefaultMode(uint8_t mode);
        static void changeOTAPass(const String* oldPass, const String* pass);
        
    private:
        static void initMode(uint8_t mode);
        static void fConnectTo();
        static void fConnectToWithAp();
        static void fDisconnect();
        static void fConnectedSta();
        static void fConnectedApSta();
        static void fFailedConnAttempt();
        static void fIncrementAttempts();
        static void fRampConnectionLost();
        static void fCloseAP();
        static void fConnInterval();
        static void fDoNothing();
        
        static VoidFunction transitions[5][5];

        static void spawnStateTimer(int timeInMs);
        static void killStateTimer();
        static void stateTimerCallback(TimerHandle_t pxTimer);
        static void ipMessageTask(void *);

        static void generateSsid();
        static esp_err_t startListener();
        static esp_err_t startTCP();
        static esp_err_t saveValue(String id, const String* value);
        static esp_err_t saveValue(String id, uint8_t value);
        static esp_err_t changeModeToAp();
        static esp_err_t changeModeToSta();
        static esp_err_t changeModeToApSta();
};

#endif /* _WifiController_H_ */
