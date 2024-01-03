#if __has_include("config/LocalConfig.h")
#include "config/LocalConfig.h"
#else
#include "config/DefaultConfig.h"
#endif

#include "WifiController.h"
#include <FileIO.h>
#include <esp_log.h>
#include <MD5Builder.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include "lwip/netif.h"

#include <esp_event_loop.h>

#define TAG "WifiController"
#define MYR_WIFI_PREF_TAG_INIT "WiFi Init"
#define MYR_WIFI_PREF_TAG_MODE "WiFi Mode"
#define MYR_WIFI_PREF_TAG_STA_SSID "WiFi StaSsid"
#define MYR_WIFI_PREF_TAG_STA_PASS "WiFi StaPass"
#define MYR_WIFI_PREF_TAG_AP_SSID "WiFi ApSsid"
#define MYR_WIFI_PREF_TAG_AP_PASS "WiFi ApPass"

const char * wifi_err_reason_str[] = { "UNSPECIFIED", "AUTH_EXPIRE", "AUTH_LEAVE", "ASSOC_EXPIRE", "ASSOC_TOOMANY", "NOT_AUTHED", "NOT_ASSOCED", "ASSOC_LEAVE", "ASSOC_NOT_AUTHED", "DISASSOC_PWRCAP_BAD", "DISASSOC_SUPCHAN_BAD", "UNSPECIFIED", "IE_INVALID", "MIC_FAILURE", "4WAY_HANDSHAKE_TIMEOUT", "GROUP_KEY_UPDATE_TIMEOUT", "IE_IN_4WAY_DIFFERS", "GROUP_CIPHER_INVALID", "PAIRWISE_CIPHER_INVALID", "AKMP_INVALID", "UNSUPP_RSN_IE_VERSION", "INVALID_RSN_IE_CAP", "802_1X_AUTH_FAILED", "CIPHER_SUITE_REJECTED", "BEACON_TIMEOUT", "NO_AP_FOUND", "AUTH_FAIL", "ASSOC_FAIL", "HANDSHAKE_TIMEOUT", "CONNECTION_FAIL" };
#define wifi_err_reason_to_str(reason) ((reason>=200)?wifi_err_reason_str[reason-176]:wifi_err_reason_str[reason-1])

static String myrSsid = "";
static int retries;

static uint8_t state = MYR_WIFI_STATE_INIT;
static Preferences preferences;

static xQueueHandle _network_event_queue;
static TaskHandle_t _network_event_task_handle = NULL;
static EventGroupHandle_t _network_event_group = NULL;

static String apSsid;
static String apPass;
static String staSsid;
static String staPass;

IPAddress localIP(192,168,4,1);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);

static void _network_event_task(void * arg){
    system_event_t *event = NULL;
    for (;;) {
        if(xQueueReceive(_network_event_queue, &event, portMAX_DELAY) == pdTRUE){
            WifiController::event_handler(arg, event);
        }
    }
    vTaskDelete(NULL);
    _network_event_task_handle = NULL;
}

esp_err_t WifiController::init() {
    log_i("Initalizing WiFi");
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    esp_err_t err = startTCP();
    if (err) return err;

    err = esp_wifi_init(&cfg);
    if (err) return err;

    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    generateSsid();

    preferences.begin("myr", false);
    uint8_t isFirstInit = preferences.getUChar(MYR_WIFI_PREF_TAG_INIT, 0) == 0;
    preferences.end();
    
    if (isFirstInit) {
        log_i("First launch detected");
        
        saveValue(MYR_WIFI_PREF_TAG_MODE, MYR_WIFI_START_IN_MODE);
        
        if (err) return err;
        err = setApCredentials(&myrSsid, &MYR_WIFI_DEFAULT_AP_PASS, true);
        if (err) return err;
        err = setStaCredentials(&MYR_WIFI_DEFAULT_STATION_SSID, &MYR_WIFI_DEFAULT_STATION_PASS, true);
        if (err) return err;

        saveValue(MYR_WIFI_PREF_TAG_INIT, 1);
    }

    err = esp_wifi_start();
    if (err) return err;

    preferences.begin("myr", true);
    uint8_t targetMode = preferences.getUChar(MYR_WIFI_PREF_TAG_MODE, MYR_WIFI_START_IN_MODE);
    staSsid = preferences.getString(MYR_WIFI_PREF_TAG_STA_SSID, MYR_WIFI_DEFAULT_STATION_SSID);
    staPass = preferences.getString(MYR_WIFI_PREF_TAG_STA_PASS, MYR_WIFI_DEFAULT_STATION_PASS);
    apSsid = preferences.getString(MYR_WIFI_PREF_TAG_AP_SSID, myrSsid);
    apPass = preferences.getString(MYR_WIFI_PREF_TAG_AP_PASS, MYR_WIFI_DEFAULT_AP_PASS);
    preferences.end();

    err = changeMode(targetMode, false);
    
    return err;
}

esp_err_t WifiController::setApCredentials(const String* newSsid, const String* newPass, bool save) { 
    esp_err_t err = ERR_OK;
    apSsid = *newSsid;
    apPass = *newPass; 

    if (save) {
        err = saveValue(MYR_WIFI_PREF_TAG_AP_SSID, newSsid);
        if (err != ERR_OK) return err;
        err = saveValue(MYR_WIFI_PREF_TAG_AP_PASS, newPass);
    }

    return err;
}

esp_err_t WifiController::setStaCredentials(const String* newSsid, const String* newPass, bool save) {
    log_i("ssid: %s", newSsid);
    esp_err_t err = ERR_OK;
    staSsid = *newSsid;
    staPass = *newPass; 

    if (save) {
        err = saveValue(MYR_WIFI_PREF_TAG_STA_SSID, newSsid);
        if (err != ERR_OK) return err;
        err = saveValue(MYR_WIFI_PREF_TAG_STA_PASS, newPass);
    }

    return err;
}

esp_err_t WifiController::changeMode(uint8_t mode, bool save) {
    esp_err_t err;
    switch (mode)
    {
        case MYR_WIFI_MODE_AP:
        {
            log_i("Entering AP mode");

            if (state == MYR_WIFI_STATE_STA) {
                err = esp_wifi_disconnect();
                ESP_ERROR_CHECK(err);
            }
            err = esp_wifi_set_mode(WIFI_MODE_AP);
            ESP_ERROR_CHECK(err);
            
            wifi_config_t configAp;
            err = esp_wifi_get_config(WIFI_IF_AP, &configAp);
            ESP_ERROR_CHECK(err);

            strlcpy(reinterpret_cast<char *>(configAp.ap.ssid), apSsid.c_str(), sizeof(configAp.ap.ssid));
            configAp.ap.ssid_len = strlen(reinterpret_cast<char *>(configAp.ap.ssid));

            if (!apPass || strlen(apPass.c_str()) == 0) {
                    configAp.ap.authmode = WIFI_AUTH_OPEN;
                    *configAp.ap.password = 0;
            } else {
                configAp.ap.authmode = WIFI_AUTH_WPA2_PSK;
                strlcpy(reinterpret_cast<char *>(configAp.ap.password), apPass.c_str(), sizeof(configAp.ap.password));
            }
            err = esp_wifi_set_config(WIFI_IF_AP, &configAp);
            ESP_ERROR_CHECK(err);
            
            state = MYR_WIFI_STATE_AP;
            if (save) saveValue(MYR_WIFI_PREF_TAG_MODE, MYR_WIFI_MODE_AP);

            break;
        }
        case MYR_WIFI_MODE_STATION:
        {
            log_i("Entering Station mode");

            retries = MYR_WIFI_STA_RETRIES;
            if (state == MYR_WIFI_STATE_STA) {
                err = esp_wifi_disconnect();
                esp_wifi_connect();
                ESP_ERROR_CHECK(err);
            } else {  
                err = esp_wifi_set_mode(WIFI_MODE_STA);
                ESP_ERROR_CHECK(err);

                err = esp_wifi_stop();
            }

            wifi_config_t configSta;
            err = esp_wifi_get_config(WIFI_IF_STA, &configSta);
            ESP_ERROR_CHECK(err);

            strcpy(reinterpret_cast<char*>(configSta.sta.ssid), staSsid.c_str());
            strcpy(reinterpret_cast<char*>(configSta.sta.password), staPass.c_str());
            err = esp_wifi_set_config(WIFI_IF_STA, &configSta);

            log_i("ssid is %s", staSsid);

            state = MYR_WIFI_STATE_STA_CONNECTING;
            err = esp_wifi_start();
            if (save) saveValue(MYR_WIFI_PREF_TAG_MODE, MYR_WIFI_MODE_STATION);

            break;   
        } 
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }

    return ERR_OK;
}

void WifiController::changeOTAPass(const String* oldPass, const String* pass) {
    log_i("Changing OTA pass");
}

esp_err_t WifiController::saveValue(String id, const String* value) {
    if (!preferences.begin("myr", false)) return ERR_ALREADY;
    preferences.putString(id.c_str(), *value);
    preferences.end();
    return ERR_OK;
}
esp_err_t WifiController::saveValue(String id, const uint8_t value) {
    if (!preferences.begin("myr", false)) return ERR_ALREADY;
    preferences.begin("myr", false);
    preferences.putUChar(id.c_str(), value);
    preferences.end();
    return ERR_OK;
}

void WifiController::generateSsid() {
    myrSsid = MYR_WIFI_DEFAULT_AP_SSID;
    if (MYR_WIFI_DEFAULT_AP_SSID_UID_CHAR_COUNT == 0) return;
    MD5Builder md5;
    md5.begin();
    md5.add(WiFi.macAddress());
    md5.calculate();

    String md5_out = md5.toString().substring(32 - MYR_WIFI_DEFAULT_AP_SSID_UID_CHAR_COUNT);
    md5_out.toUpperCase();
    myrSsid += md5_out;
}

esp_err_t WifiController::startTCP() {
    esp_err_t err;
    tcpip_adapter_init();

    err = startListener();
    if (err) return err;

    tcpip_adapter_ip_info_t info;
    info.ip.addr = static_cast<uint32_t>(localIP);
    info.gw.addr = static_cast<uint32_t>(gateway);
    info.netmask.addr = static_cast<uint32_t>(subnet);

    err = tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP);
    if (err) return err;
    err = tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info);
    if (err) return err;
    
    dhcps_lease_t lease;
    lease.enable = true;
    lease.start_ip.addr = static_cast<uint32_t>(localIP) + (1 << 24);
    lease.end_ip.addr = static_cast<uint32_t>(localIP) + (11 << 24);

    err = tcpip_adapter_dhcps_option(
        TCPIP_ADAPTER_OP_SET,
        TCPIP_ADAPTER_REQUESTED_IP_ADDRESS,
        (void*)&lease, sizeof(dhcps_lease_t)
    );
    if (err) return err;

    err = tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP);
    if (err) return err;

    tcpip_adapter_ip_info_t ip;
    err = tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ip); 
    if (err) return err;
    
    uint8_t * ipa = (uint8_t *)&(ip.ip.addr);
    log_i("ip addr is : GW: %u.%u.%u.%u", ipa[0], ipa[1], ipa[2], ipa[3]);
    return ERR_OK;
}

static esp_err_t _network_event_cb(void *arg, system_event_t *event){
    if (xQueueSend(_network_event_queue, &event, portMAX_DELAY) != pdPASS) {
        log_w("Network Event Queue Send Failed!");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t WifiController::startListener() {
    if(!_network_event_group){
        _network_event_group = xEventGroupCreate();
        if(!_network_event_group){
            log_e("Network Event Group Create Failed!");
            //todo custom errors
            return ESP_ERR_INVALID_STATE;
        }
        xEventGroupSetBits(_network_event_group, WIFI_DNS_IDLE_BIT);
    }
    if(!_network_event_queue){
        _network_event_queue = xQueueCreate(32, sizeof(system_event_t *));
        if(!_network_event_queue){
            log_e("Network Event Queue Create Failed!");
            return ESP_ERR_INVALID_STATE;
        }
    }
    if(!_network_event_task_handle){
        xTaskCreateUniversal(_network_event_task, "network_event", 4096, NULL, ESP_TASKD_EVENT_PRIO - 1, &_network_event_task_handle, CONFIG_ARDUINO_EVENT_RUNNING_CORE);
        if(!_network_event_task_handle){
            log_e("Network Event Task Start Failed!");
            return ESP_ERR_INVALID_STATE;
        }
    }
    return esp_event_loop_init(&_network_event_cb, NULL);
}

void WifiController::event_handler(void *arg, system_event_t *event)
{
    switch(event->event_id)
    {
        case SYSTEM_EVENT_WIFI_READY: 
            log_i("WiFi interface ready");
            break;
        case SYSTEM_EVENT_SCAN_DONE:
            log_i("Completed scan for access points");
            break;
        case SYSTEM_EVENT_STA_START:
            log_i("WiFi client started");
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_STOP:
            log_i("WiFi clients stopped");
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            log_i("Connected to access point");
            state = MYR_WIFI_STATE_STA;
            retries = MYR_WIFI_STA_RETRIES;
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
        {
            log_i("Disconnected from WiFi access point");
            uint8_t reason = event->event_info.disconnected.reason;
            log_w("Reason: %u - %s", reason, wifi_err_reason_to_str(reason));
            if (reason == WIFI_REASON_ASSOC_LEAVE) {
                if (state == MYR_WIFI_STATE_STA_CONNECTING) {
                    esp_wifi_connect();
                }
            } else if (retries >= 0) {
                log_i("%d tries left", retries);
                retries--;
                esp_wifi_connect();
            } else {
                log_i("Failed to connect, starting fallback AP");
                state = MYR_WIFI_STATE_STA_FAIL;
                changeMode(MYR_WIFI_MODE_AP, false);
            }

            break;
        }
        case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
            log_i("Authentication mode of access point has changed");
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
        {
            uint8_t * spa = (uint8_t *)&(event->event_info.got_ip.ip_info.ip.addr);
            log_i("Obtained IP address: %u.%u.%u.%u",  spa[0], spa[1], spa[2], spa[3]);
            break;
        }
        case SYSTEM_EVENT_STA_LOST_IP:
            log_i("Lost IP address and IP address is reset to 0");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
            log_i("WiFi Protected Setup (WPS): succeeded in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_FAILED:
            log_i("WiFi Protected Setup (WPS): failed in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
            log_i("WiFi Protected Setup (WPS): timeout in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_PIN:
            log_i("WiFi Protected Setup (WPS): pin code in enrollee mode");
            break;
        case SYSTEM_EVENT_AP_START:
            log_i("WiFi access point started");
            break;
        case SYSTEM_EVENT_AP_STOP:
            log_i("WiFi access point stopped");
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            log_i("Client connected");
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            log_i("Client disconnected");
            break;
        case SYSTEM_EVENT_AP_STAIPASSIGNED:
            log_i("IP assigned");
            break;
        case SYSTEM_EVENT_AP_PROBEREQRECVED:
            log_i("Received probe request");
            break;
        case SYSTEM_EVENT_GOT_IP6:
            log_i("IPv6 is preferred");
            break;
        case SYSTEM_EVENT_ETH_START:
            log_i("Ethernet started");
            break;
        case SYSTEM_EVENT_ETH_STOP:
            log_i("Ethernet stopped");
            break;
        case SYSTEM_EVENT_ETH_CONNECTED:
            log_i("Ethernet connected");
            break;
        case SYSTEM_EVENT_ETH_DISCONNECTED:
            log_i("Ethernet disconnected");
            break;
        case SYSTEM_EVENT_ETH_GOT_IP:
            log_i("Obtained IP address");
            break;
        default:
            log_i("Unknown event");
            break;
    }
}