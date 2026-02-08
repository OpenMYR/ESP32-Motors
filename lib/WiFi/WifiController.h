#ifndef MYR_WIFICONTROLLER_H
#define MYR_WIFICONTROLLER_H

#include <string>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/timers.h"

ESP_EVENT_DECLARE_BASE(MYR_WIFI_EVENT_BASE);


class WifiController {
    public:

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
        MYR_WIFI_STATE_STA                      = 1,       /*!< Device has connected to an AP */
        MYR_WIFI_STATE_STA_CONNECTING           = 2,       /*!< Device is trying to connect to an AP */
        MYR_WIFI_STATE_AP_STA_CONNECTING        = 3,       /*!< Device is trying to connect while AP is up */
        MYR_WIFI_STATE_AP_STA_RAMPDOWN          = 4        /*!< Grace period before closing AP */
    } myr_wifi_state_t;

    static esp_err_t init();
    static void fireWifiEvent(myr_wifi_event_t event, void *data);
    static void network_event_handler(void *arg, esp_event_base_t base, int32_t id, void *event_data);
    static void state_event_handler(void *arg, esp_event_base_t base, int32_t id, void *event_data);
    static esp_err_t tryConnectToSta(const std::string *ssid, const std::string *pass);

    static esp_err_t setDefaultApCredentials(const std::string *ssid, const std::string *pass);
    static esp_err_t setDefaultStaCredentials(const std::string *ssid, const std::string *pass);
    static esp_err_t setDefaultMode(uint8_t mode);
    static void changeOTAPass(const std::string *oldPass, const std::string *pass);
    static myr_wifi_state_t getWiFiState();

    // Test-helper: pumps the WifiController state event loop to process
    // any queued events. Intended for unit tests to avoid race coditions.
    static void processStateEvents(uint32_t timeout_ms = 10);

    // Constants
    static constexpr const char *MYR_WIFI_PREF_TAG_INIT = "WiFi Init";
    static constexpr const char *MYR_WIFI_PREF_TAG_MODE = "WiFi Mode";
    static constexpr const char *MYR_WIFI_PREF_TAG_STA_SSID = "WiFi StaSsid";
    static constexpr const char *MYR_WIFI_PREF_TAG_STA_PASS = "WiFi StaPass";
    static constexpr const char *MYR_WIFI_PREF_TAG_AP_SSID = "WiFi ApSsid";
    static constexpr const char *MYR_WIFI_PREF_TAG_AP_PASS = "WiFi ApPass";

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

    static void (*transitions[5][5])();

    static void spawnStateTimer(int timeInMs);
    static void killStateTimer();
    static void stateTimerCallback(TimerHandle_t pxTimer);
    static void generateSsid();
    static esp_err_t startListener();
    static esp_err_t startTCP();
    static esp_err_t saveValue(const char *id, const std::string *value);
    static esp_err_t saveValue(const char *id, uint8_t value);
    static std::string getValue(const char *id, const char *defaultValue);
    static uint8_t getValue(const char *id, uint8_t defaultValue);
    static esp_err_t changeModeToAp();
    static esp_err_t changeModeToSta();
    static esp_err_t changeModeToApSta();

    static std::string myrSsid;
    static int attempts;
    static myr_wifi_state_t state;
    static esp_event_loop_handle_t state_loop_handle;
    static esp_event_loop_args_t loop_args;
    static int stateConnTimeoutLatch;
    static TimerHandle_t stateTimerHandle;
    static std::string apSsid;
    static std::string apPass;
    static std::string staSsid;
    static std::string staPass;
    static esp_netif_t *ap_netif;
    static esp_netif_t *sta_netif;
};

#endif // MYR_WIFICONTROLLER_H
