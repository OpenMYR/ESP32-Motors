#ifndef MYR_WIFICONTROLLER_H
#define MYR_WIFICONTROLLER_H

#include <WString.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Preferences.h>
extern "C" {
#include <esp_netif.h>
}

typedef void (*VoidFunction) ();

ESP_EVENT_DECLARE_BASE(MYR_WIFI_EVENT_BASE);

/* Definitions state events */
typedef enum {
    MYR_WIFI_EVENT_CONN_NEW_STA             = 0,       /*!< Connection request to a new AP */
    MYR_WIFI_EVENT_DISCONNECT               = 1,       /*!< Disconnect request from current AP */
    MYR_WIFI_EVENT_CONNECTED                = 2,       /*!< Successfully connected to a new AP */
    MYR_WIFI_EVENT_CONNECTION_FAILED        = 3,       /*!< A connection has been lost or failed to establish */
    MYR_WIFI_EVENT_TIMER                    = 4        /*!< A time has passed */
} myr_wifi_event_t;

/* Definitions for states */
typedef enum {
    MYR_WIFI_STATE_AP                       = 0,       /*!< AP has started */
    MYR_WIFI_STATE_STA                      = 1,       /*!< Device has connected to an ap */
    MYR_WIFI_STATE_STA_CONNECTING           = 2,       /*!< Device is trying to connect to an ap */
    MYR_WIFI_STATE_AP_STA_CONNECTING        = 3,       /*!<  */
    MYR_WIFI_STATE_AP_STA_RAMPDOWN          = 4        /*!<  */
} myr_wifi_state_t;

class WifiController {
    public:
        static esp_err_t init();
        static void fireWifiEvent(myr_wifi_event_t event, void *data);
        static void network_event_handler(void *arg, esp_event_base_t base, int32_t id, void* event_data);
        static void state_event_handler(void *arg, esp_event_base_t base, int32_t id, void* event_data);
        static esp_err_t tryConnectToSta(const String* ssid, const String* pass);

        static esp_err_t setDefaultApCredentials(const String* ssid, const String* pass);
        static esp_err_t setDefaultStaCredentials(const String* ssid, const String* pass);
        static esp_err_t setDefaultMode(uint8_t mode);
        static void changeOTAPass(const String* oldPass, const String* pass);
        static myr_wifi_state_t getWiFiState();
        
        // Constants
        static constexpr const char* MYR_WIFI_PREF_TAG_INIT = "WiFi Init";
        static constexpr const char* MYR_WIFI_PREF_TAG_MODE = "WiFi Mode";
        static constexpr const char* MYR_WIFI_PREF_TAG_STA_SSID = "WiFi StaSsid";
        static constexpr const char* MYR_WIFI_PREF_TAG_STA_PASS = "WiFi StaPass";
        static constexpr const char* MYR_WIFI_PREF_TAG_AP_SSID = "WiFi ApSsid";
        static constexpr const char* MYR_WIFI_PREF_TAG_AP_PASS = "WiFi ApPass";
        
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
        
        static String myrSsid;
        static int attempts;
        static myr_wifi_state_t state;
        static Preferences preferences;
        static EventGroupHandle_t _network_event_group;
        static EventGroupHandle_t s_wifi_event_group;
        static esp_event_loop_handle_t state_loop_handle;
        static esp_event_loop_args_t loop_args;
        static int stateConnTimeoutLatch;
        static TaskHandle_t ipMessageTaskHandle;
        static TimerHandle_t stateTimerHandle;
        static String apSsid;
        static String apPass;
        static String staSsid;
        static String staPass;
        static String ip;
        static IPAddress localIP;
        static IPAddress gateway;
        static IPAddress subnet;
        static esp_netif_t *ap_netif;
        static esp_netif_t *sta_netif;
};

#endif // MYR_WIFICONTROLLER_H
