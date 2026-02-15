#include "WifiController.h"

#include <stdio.h>
#include <string.h>

#include "Lookup.h"
#include "config/Config.h"

#include <esp_event.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <nvs.h>
#include <nvs_flash.h>

#include <apps/dhcpserver/dhcpserver.h>

ESP_EVENT_DEFINE_BASE(MYR_WIFI_EVENT_BASE);

static const char *TAG = "WifiController";
static const char *kNvsNamespace = "myr";

static void copy_string_to_array(uint8_t *dst, size_t dst_len, const std::string &src) {
    strlcpy(reinterpret_cast<char *>(dst), src.c_str(), dst_len);
}

void (*WifiController::transitions[5][5])() = {
    //State:   AP            STA                     STA_CONNECTING          AP_STA_CONNECTING       AP_STA_RAMPDOWN
    {fConnectToWithAp       ,fConnectTo             ,fDoNothing             ,fConnectToWithAp       ,fConnectToWithAp       }, //Event: CONN_NEW_STA
    {fDoNothing             ,fDisconnect            ,fDisconnect            ,fDisconnect            ,fDisconnect            }, //Event: DISCONNECT
    {fDoNothing             ,fDoNothing             ,fConnectedSta          ,fConnectedApSta        ,fDoNothing             }, //Event: CONNECTED
    {fDoNothing             ,fConnectTo             ,fFailedConnAttempt     ,fFailedConnAttempt     ,fRampConnectionLost    }, //Event: CONNECTION_FAILED
    {fDoNothing             ,fDoNothing             ,fConnInterval          ,fConnInterval          ,fCloseAP               }, //Event: TIMER
};

std::string WifiController::myrSsid = "";
int WifiController::attempts = 0;
WifiController::myr_wifi_state_t WifiController::state;

esp_event_loop_handle_t WifiController::state_loop_handle = nullptr;
esp_event_loop_args_t WifiController::loop_args = {};

int WifiController::stateConnTimeoutLatch = 1;

TimerHandle_t WifiController::stateTimerHandle = nullptr;

std::string WifiController::apSsid;
std::string WifiController::apPass;
std::string WifiController::staSsid;
std::string WifiController::staPass;

esp_netif_t *WifiController::ap_netif = nullptr;
esp_netif_t *WifiController::sta_netif = nullptr;

esp_err_t WifiController::init() {
    ESP_LOGI(TAG, "Initializing WiFi");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK && err != ESP_ERR_NVS_INVALID_STATE) return err;

    // Keep framework WiFi/netif internals readable in normal operation.
    esp_log_level_set("wifi", ESP_LOG_WARN);
    esp_log_level_set("wifi_init", ESP_LOG_WARN);
    esp_log_level_set("esp_netif_lwip", ESP_LOG_WARN);
    esp_log_level_set("esp_netif_handlers", ESP_LOG_WARN);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    err = startListener();
    if (err) return err;

    err = startTCP();
    if (err) return err;

    err = esp_wifi_init(&cfg);
    if (err != ESP_OK && err != ESP_ERR_WIFI_INIT_STATE) return err;

    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    generateSsid();

    uint8_t isFirstInit = 1;
    if (MYR_REMEMBER_CHANGES) isFirstInit = getValue(MYR_WIFI_PREF_TAG_INIT, static_cast<uint8_t>(0)) == 0;

    if (isFirstInit) {
        ESP_LOGI(TAG, "First launch detected");

        saveValue(MYR_WIFI_PREF_TAG_MODE, MYR_WIFI_START_IN_MODE);

        const std::string defaultApPass(MYR_WIFI_DEFAULT_AP_PASS);
        const std::string defaultStaSsid(MYR_WIFI_DEFAULT_STATION_SSID);
        const std::string defaultStaPass(MYR_WIFI_DEFAULT_STATION_PASS);
        const std::string defaultOtaPass(MYR_OTA_DEFAULT_PASSWORD);

        err = setDefaultApCredentials(&myrSsid, &defaultApPass);
        if (err) return err;
        err = setDefaultStaCredentials(&defaultStaSsid, &defaultStaPass);
        if (err) return err;
        err = saveValue(MYR_WIFI_PREF_TAG_OTA_PASS, &defaultOtaPass);
        if (err) return err;

        saveValue(MYR_WIFI_PREF_TAG_INIT, 1);
    }

    err = esp_wifi_start();
    if (err && err != ESP_ERR_WIFI_CONN) return err;

    uint8_t targetMode = getValue(MYR_WIFI_PREF_TAG_MODE, MYR_WIFI_START_IN_MODE);
    staSsid = getValue(MYR_WIFI_PREF_TAG_STA_SSID, MYR_WIFI_DEFAULT_STATION_SSID);
    staPass = getValue(MYR_WIFI_PREF_TAG_STA_PASS, MYR_WIFI_DEFAULT_STATION_PASS);
    apSsid = getValue(MYR_WIFI_PREF_TAG_AP_SSID, myrSsid.c_str());
    apPass = getValue(MYR_WIFI_PREF_TAG_AP_PASS, MYR_WIFI_DEFAULT_AP_PASS);

    initMode(targetMode);
    return ESP_OK;
}

void WifiController::initMode(uint8_t mode) {
    esp_err_t err;
    switch (mode) {
    case MYR_WIFI_MODE_AP: {
        ESP_LOGI(TAG, "Entering AP mode");

        state = MYR_WIFI_STATE_AP;

        err = changeModeToAp();
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);

        break;
    }
    case MYR_WIFI_MODE_STATION: {
        ESP_LOGI(TAG, "Entering Station mode");

        state = MYR_WIFI_STATE_STA_CONNECTING;
        err = changeModeToSta();
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);

        esp_wifi_connect();

        break;
    }
    }
}

void WifiController::fireWifiEvent(myr_wifi_event_t event, void *data) {
    esp_event_post_to(state_loop_handle, MYR_WIFI_EVENT_BASE, event, data, 0, portMAX_DELAY);
}

void WifiController::processStateEvents(uint32_t timeout_ms) {
    esp_event_loop_run(state_loop_handle, static_cast<int32_t>(pdMS_TO_TICKS(timeout_ms)));
}

void WifiController::fConnectTo() {
    ESP_LOGI(TAG, "fConnectTo");
    killStateTimer();
    attempts = 0;
    state = MYR_WIFI_STATE_STA_CONNECTING;
    esp_wifi_disconnect();
    esp_err_t err = changeModeToSta();
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    stateConnTimeoutLatch = 1;
    esp_wifi_connect();
}

void WifiController::fConnectToWithAp() {
    ESP_LOGI(TAG, "fConnectToWithAp");
    killStateTimer();
    attempts = 0;
    state = MYR_WIFI_STATE_AP_STA_CONNECTING;
    esp_err_t err = changeModeToApSta();
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    stateConnTimeoutLatch = 1;
    esp_wifi_connect();
}

void WifiController::fDisconnect() {
    ESP_LOGI(TAG, "fDisconnect");

    state = MYR_WIFI_STATE_AP;
    esp_err_t err = esp_wifi_disconnect();
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    err = changeModeToAp();
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
}

void WifiController::fConnectedSta() {
    ESP_LOGI(TAG, "fConnectedSta");
    killStateTimer();
    state = MYR_WIFI_STATE_STA;
    esp_err_t err = changeModeToSta();
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
}

void WifiController::fConnectedApSta() {
    ESP_LOGI(TAG, "fConnectedApSta");
    killStateTimer();
    state = MYR_WIFI_STATE_AP_STA_RAMPDOWN;
    spawnStateTimer(5000);
}

void WifiController::fFailedConnAttempt() {
    attempts++;
    ESP_LOGI(TAG, "attempt #%i/%i", attempts, MYR_WIFI_STATION_RETRIES);
    if (stateConnTimeoutLatch == 1) {
        fIncrementAttempts();
    } else {
        stateConnTimeoutLatch++;
    }
}

void WifiController::fIncrementAttempts() {
    ESP_LOGI(TAG, "fIncrementAttempts");
    if (attempts >= MYR_WIFI_STATION_RETRIES) {
        state = MYR_WIFI_STATE_AP_STA_CONNECTING;
        changeModeToApSta();
        spawnStateTimer(MYR_WIFI_STA_RETRY_INTERVAL_LONG);
    } else {
        spawnStateTimer(MYR_WIFI_STA_RETRY_INTERVAL);
    }
    stateConnTimeoutLatch = 0;
    esp_wifi_connect();
}

void WifiController::fRampConnectionLost() {
    ESP_LOGI(TAG, "fRampConnectionLost");
    state = MYR_WIFI_STATE_AP_STA_CONNECTING;
    esp_err_t err = changeModeToApSta();
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    esp_wifi_connect();
}

void WifiController::fCloseAP() {
    ESP_LOGI(TAG, "fCloseAP");
    attempts = 0;
    state = MYR_WIFI_STATE_STA;
    esp_err_t err = changeModeToSta();
    ESP_ERROR_CHECK_WITHOUT_ABORT(err);
    esp_wifi_connect();
}

void WifiController::fConnInterval() {
    ESP_LOGI(TAG, "fConnInterval");
    if (stateConnTimeoutLatch == 1) {
        fIncrementAttempts();
    } else {
        stateConnTimeoutLatch++;
    }
}

void WifiController::fDoNothing() {
    // Resist the urge
}

esp_err_t WifiController::setDefaultApCredentials(const std::string *newSsid, const std::string *newPass) {
    if (newSsid == nullptr || newSsid->empty()) return ESP_ERR_INVALID_ARG;
    static const std::string kEmpty;

    esp_err_t err = saveValue(MYR_WIFI_PREF_TAG_AP_SSID, newSsid);
    if (err != ESP_OK) return err;
    err = saveValue(MYR_WIFI_PREF_TAG_AP_PASS, (newPass == nullptr) ? &kEmpty : newPass);

    return err;
}

esp_err_t WifiController::setDefaultStaCredentials(const std::string *newSsid, const std::string *newPass) {
    if (newSsid == nullptr || newSsid->empty()) return ESP_ERR_INVALID_ARG;
    static const std::string kEmpty;
    ESP_LOGI(TAG, "Persisting station SSID: %s", newSsid->c_str());

    esp_err_t err = saveValue(MYR_WIFI_PREF_TAG_STA_SSID, newSsid);
    if (err != ESP_OK) return err;
    err = saveValue(MYR_WIFI_PREF_TAG_STA_PASS, (newPass == nullptr) ? &kEmpty : newPass);

    return err;
}

esp_err_t WifiController::setDefaultMode(uint8_t mode) {
    if (~static_cast<uint8_t>(MYR_WIFI_SUPPORTED_MODES) & mode) return ESP_ERR_INVALID_ARG;
    return saveValue(MYR_WIFI_PREF_TAG_MODE, mode);
}

esp_err_t WifiController::tryConnectToSta(const std::string *ssid, const std::string *pass) {
    if (ssid == nullptr || ssid->empty()) return ESP_ERR_INVALID_ARG;

    staSsid = *ssid;
    staPass = (pass == nullptr) ? std::string() : *pass;
    fireWifiEvent(MYR_WIFI_EVENT_CONN_NEW_STA, nullptr);
    return ESP_OK;
}

void WifiController::spawnStateTimer(int timeInMs) {
    if (stateTimerHandle == nullptr) {
        stateTimerHandle = xTimerCreate(
            "wifi state timeout",
            pdMS_TO_TICKS(timeInMs),
            pdFALSE,
            reinterpret_cast<void *>(state),
            &WifiController::stateTimerCallback);
    } else {
        xTimerStop(stateTimerHandle, 0);
        xTimerChangePeriod(stateTimerHandle, pdMS_TO_TICKS(timeInMs), 0);
    }
    xTimerStart(stateTimerHandle, 0);
}

void WifiController::killStateTimer() {
    if (stateTimerHandle != nullptr) xTimerStop(stateTimerHandle, 0);
}

void WifiController::stateTimerCallback(TimerHandle_t pxTimer) {
    fireWifiEvent(MYR_WIFI_EVENT_TIMER, pvTimerGetTimerID(pxTimer));
}

esp_err_t WifiController::changeModeToAp() {
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err) return err;

    wifi_config_t configAp = {};
    err = esp_wifi_get_config(WIFI_IF_AP, &configAp);
    if (err) return err;

    strlcpy(reinterpret_cast<char *>(configAp.ap.ssid), apSsid.c_str(), sizeof(configAp.ap.ssid));
    configAp.ap.ssid_len = strlen(reinterpret_cast<char *>(configAp.ap.ssid));

    if (apPass.empty()) {
        configAp.ap.authmode = WIFI_AUTH_OPEN;
        *configAp.ap.password = 0;
    } else {
        configAp.ap.authmode = WIFI_AUTH_WPA2_PSK;
        strlcpy(reinterpret_cast<char *>(configAp.ap.password), apPass.c_str(), sizeof(configAp.ap.password));
    }
    err = esp_wifi_set_config(WIFI_IF_AP, &configAp);
    if (err) return err;

    return ESP_OK;
}

esp_err_t WifiController::changeModeToSta() {
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err) return err;

    wifi_config_t configSta = {};
    err = esp_wifi_get_config(WIFI_IF_STA, &configSta);
    if (err) return err;

    copy_string_to_array(configSta.sta.ssid, sizeof(configSta.sta.ssid), staSsid);
    copy_string_to_array(configSta.sta.password, sizeof(configSta.sta.password), staPass);
    err = esp_wifi_set_config(WIFI_IF_STA, &configSta);
    if (err) return err;

    ESP_LOGI(TAG, "Station credentials configured for SSID: %s", staSsid.c_str());
    return ESP_OK;
}

esp_err_t WifiController::changeModeToApSta() {
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err) return err;

    wifi_config_t configAp = {};
    err = esp_wifi_get_config(WIFI_IF_AP, &configAp);
    if (err) return err;

    strlcpy(reinterpret_cast<char *>(configAp.ap.ssid), apSsid.c_str(), sizeof(configAp.ap.ssid));
    configAp.ap.ssid_len = strlen(reinterpret_cast<char *>(configAp.ap.ssid));

    if (apPass.empty()) {
        configAp.ap.authmode = WIFI_AUTH_OPEN;
        *configAp.ap.password = 0;
    } else {
        configAp.ap.authmode = WIFI_AUTH_WPA2_PSK;
        strlcpy(reinterpret_cast<char *>(configAp.ap.password), apPass.c_str(), sizeof(configAp.ap.password));
    }
    err = esp_wifi_set_config(WIFI_IF_AP, &configAp);
    if (err) return err;

    wifi_config_t configSta = {};
    err = esp_wifi_get_config(WIFI_IF_STA, &configSta);
    if (err) return err;

    copy_string_to_array(configSta.sta.ssid, sizeof(configSta.sta.ssid), staSsid);
    copy_string_to_array(configSta.sta.password, sizeof(configSta.sta.password), staPass);
    err = esp_wifi_set_config(WIFI_IF_STA, &configSta);
    if (err) return err;

    err = esp_wifi_start();
    if (err && err != ESP_ERR_WIFI_CONN) return err;

    return ESP_OK;
}

void WifiController::changeOTAPass(const std::string *oldPass, const std::string *pass) {
    if (oldPass == nullptr || pass == nullptr || pass->empty()) {
        ESP_LOGW(TAG, "Rejected OTA password change: invalid arguments");
        return;
    }

    const std::string current = getOTAPassword();
    if (*oldPass != current) {
        ESP_LOGW(TAG, "Rejected OTA password change: old password mismatch");
        return;
    }

    esp_err_t err = saveValue(MYR_WIFI_PREF_TAG_OTA_PASS, pass);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to persist OTA password: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "OTA password changed");
}

std::string WifiController::getOTAPassword() {
    return getValue(MYR_WIFI_PREF_TAG_OTA_PASS, MYR_OTA_DEFAULT_PASSWORD);
}

esp_err_t WifiController::saveValue(const char *id, const std::string *value) {
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK) return err;

    err = nvs_set_str(nvsHandle, id, value->c_str());
    if (err == ESP_OK) err = nvs_commit(nvsHandle);
    nvs_close(nvsHandle);

    return err;
}

esp_err_t WifiController::saveValue(const char *id, const uint8_t value) {
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK) return err;

    err = nvs_set_u8(nvsHandle, id, value);
    if (err == ESP_OK) err = nvs_commit(nvsHandle);
    nvs_close(nvsHandle);

    return err;
}

std::string WifiController::getValue(const char *id, const char *defaultValue) {
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READONLY, &nvsHandle);
    if (err != ESP_OK) return std::string(defaultValue);

    size_t required = 0;
    err = nvs_get_str(nvsHandle, id, nullptr, &required);
    if (err != ESP_OK || required == 0) {
        nvs_close(nvsHandle);
        return std::string(defaultValue);
    }

    std::string out(required, '\0');
    err = nvs_get_str(nvsHandle, id, out.data(), &required);
    nvs_close(nvsHandle);
    if (err != ESP_OK) return std::string(defaultValue);

    if (!out.empty() && out.back() == '\0') out.pop_back();

    return out;
}

uint8_t WifiController::getValue(const char *id, uint8_t defaultValue) {
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READONLY, &nvsHandle);
    if (err != ESP_OK) return defaultValue;

    uint8_t out = defaultValue;
    err = nvs_get_u8(nvsHandle, id, &out);
    nvs_close(nvsHandle);
    if (err != ESP_OK) return defaultValue;

    return out;
}

void WifiController::generateSsid() {
    myrSsid = MYR_WIFI_DEFAULT_AP_SSID;
    if (MYR_WIFI_DEFAULT_AP_SSID_UID_CHAR_COUNT == 0) return;

    uint8_t mac[6] = {};
    if (esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP) != ESP_OK) return;

    char suffix[13];
    snprintf(suffix, sizeof(suffix), "%02X%02X%02X", mac[3], mac[4], mac[5]);
    std::string suffixStr(suffix);
    if (suffixStr.size() > static_cast<size_t>(MYR_WIFI_DEFAULT_AP_SSID_UID_CHAR_COUNT)) {
        suffixStr = suffixStr.substr(suffixStr.size() - MYR_WIFI_DEFAULT_AP_SSID_UID_CHAR_COUNT);
    }
    myrSsid += suffixStr;
}

WifiController::myr_wifi_state_t WifiController::getWiFiState() {
    return state;
}

esp_err_t WifiController::startTCP() {
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    esp_netif_ip_info_t info = {};
    esp_netif_set_ip4_addr(&info.ip, 192, 168, 4, 1);
    esp_netif_set_ip4_addr(&info.gw, 192, 168, 4, 1);
    esp_netif_set_ip4_addr(&info.netmask, 255, 255, 255, 0);

    ap_netif = esp_netif_create_default_wifi_ap();
    sta_netif = esp_netif_create_default_wifi_sta();
    if (ap_netif == nullptr || sta_netif == nullptr) return ESP_FAIL;

    err = esp_netif_dhcps_stop(ap_netif);
    if (err) return err;
    err = esp_netif_set_ip_info(ap_netif, &info);
    if (err) return err;

    dhcps_lease_t lease = {};
    lease.enable = true;
    IP4_ADDR(&lease.start_ip, 192, 168, 4, 2);
    IP4_ADDR(&lease.end_ip, 192, 168, 4, 12);

    err = esp_netif_dhcps_option(
        ap_netif,
        ESP_NETIF_OP_SET,
        ESP_NETIF_REQUESTED_IP_ADDRESS,
        reinterpret_cast<void *>(&lease), sizeof(dhcps_lease_t));
    if (err) return err;

    err = esp_netif_dhcps_start(ap_netif);
    if (err) return err;

    esp_netif_ip_info_t ipInfo = {};
    err = esp_netif_get_ip_info(ap_netif, &ipInfo);
    if (err) return err;

    uint8_t *ipa = reinterpret_cast<uint8_t *>(&(ipInfo.ip.addr));
    ESP_LOGI(TAG, "ip addr is : GW: %u.%u.%u.%u", ipa[0], ipa[1], ipa[2], ipa[3]);
    return ESP_OK;
}

esp_err_t WifiController::startListener() {
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    loop_args = {
        .queue_size = 32,
        .task_name = "state_events",
        .task_priority = 5,
        // 4096-byte stack is required here; 2048 reproduces an esp_event spinlock assert during STA auth-failure retry bursts on ESP-IDF 5.5.
        .task_stack_size = 4096,
        .task_core_id = 1,
    };
    err = esp_event_loop_create(&loop_args, &state_loop_handle);
    if (err) return err;

    err = esp_event_handler_register_with(
        state_loop_handle,
        MYR_WIFI_EVENT_BASE,
        ESP_EVENT_ANY_ID,
        WifiController::state_event_handler,
        nullptr);
    if (err) return err;

    esp_event_handler_instance_t instance_any_id;
    err = esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &network_event_handler,
        nullptr,
        &instance_any_id);
    if (err) return err;

    err = esp_event_handler_instance_register(
        IP_EVENT,
        ESP_EVENT_ANY_ID,
        &network_event_handler,
        nullptr,
        &instance_any_id);
    if (err) return err;
    return err;
}

void WifiController::state_event_handler(void *arg, esp_event_base_t base, int32_t id, void *event_data) {
    (void)arg;
    (void)base;
    (void)event_data;

    const auto event = static_cast<myr_wifi_event_t>(id);
    const auto currentState = state;
    ESP_LOGI(
        TAG,
        "State event %s(%u) in %s(%u)",
        Lookup::wifiStateEventToString(static_cast<uint8_t>(event)),
        static_cast<unsigned int>(event),
        Lookup::wifiStateToString(static_cast<uint8_t>(currentState)),
        static_cast<unsigned int>(currentState));
    transitions[id][state]();
}

void WifiController::network_event_handler(void *arg, esp_event_base_t base, int32_t id, void *event_data) {
    (void)arg;

    ESP_LOGD(TAG, "--- %s    %ld", base, static_cast<long>(id));

    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_WIFI_READY:
            ESP_LOGI(TAG, "WiFi interface ready");
            break;
        case WIFI_EVENT_SCAN_DONE:
            ESP_LOGI(TAG, "Completed scan for access points");
            break;
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi client started");
            break;
        case WIFI_EVENT_STA_STOP:
            ESP_LOGI(TAG, "WiFi clients stopped");
            break;
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "Connected to access point");
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            const auto *event = static_cast<wifi_event_sta_disconnected_t *>(event_data);
            const uint8_t reason = event->reason;
            ESP_LOGW(TAG, "WiFi disconnect, Reason: %u - %s", reason, Lookup::wifiDisconnectReasonToString(reason));
            fireWifiEvent(MYR_WIFI_EVENT_CONNECTION_FAILED, nullptr);
            break;
        }
        case WIFI_EVENT_AP_START:
            ESP_LOGI(TAG, "WiFi access point started");
            break;
        case WIFI_EVENT_AP_STOP:
            ESP_LOGI(TAG, "WiFi access point stopped");
            break;
        case WIFI_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "Client connected");
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "Client disconnected");
            break;
        case WIFI_EVENT_STA_AUTHMODE_CHANGE: {
            const auto *event = static_cast<wifi_event_sta_authmode_change_t *>(event_data);
            ESP_LOGD(
                TAG,
                "Authentication mode changed: %u(%s) -> %u(%s)",
                static_cast<unsigned int>(event->old_mode),
                Lookup::wifiAuthModeToString(event->old_mode),
                static_cast<unsigned int>(event->new_mode),
                Lookup::wifiAuthModeToString(event->new_mode));
            break;
        }
        case WIFI_EVENT_STA_WPS_ER_SUCCESS:
            ESP_LOGI(TAG, "WiFi Protected Setup (WPS): succeeded in enrollee mode");
            break;
        case WIFI_EVENT_STA_WPS_ER_FAILED:
            ESP_LOGI(TAG, "WiFi Protected Setup (WPS): failed in enrollee mode");
            break;
        case WIFI_EVENT_STA_WPS_ER_TIMEOUT:
            ESP_LOGI(TAG, "WiFi Protected Setup (WPS): timeout in enrollee mode");
            break;
        case WIFI_EVENT_STA_WPS_ER_PIN:
            ESP_LOGI(TAG, "WiFi Protected Setup (WPS): pin code in enrollee mode");
            break;
        case WIFI_EVENT_AP_PROBEREQRECVED:
            ESP_LOGI(TAG, "Received probe request");
            break;
        case WIFI_EVENT_HOME_CHANNEL_CHANGE: {
            const auto *event = static_cast<wifi_event_home_channel_change_t *>(event_data);
            ESP_LOGD(
                TAG,
                "Home channel changed: <%u,%u> -> <%u,%u>",
                static_cast<unsigned int>(event->old_chan),
                static_cast<unsigned int>(event->old_snd),
                static_cast<unsigned int>(event->new_chan),
                static_cast<unsigned int>(event->new_snd));
            break;
        }
        default:
            ESP_LOGD(TAG, "Unhandled WIFI_EVENT id: %ld", static_cast<long>(id));
            break;
        }
    } else if (base == IP_EVENT) {
        switch (id) {
        case IP_EVENT_STA_GOT_IP: {
            auto *event = static_cast<ip_event_got_ip_t *>(event_data);
            uint8_t *spa = reinterpret_cast<uint8_t *>(&(event->ip_info.ip.addr));
            ESP_LOGI(TAG, "Obtained IP address: %u.%u.%u.%u", spa[0], spa[1], spa[2], spa[3]);

            fireWifiEvent(MYR_WIFI_EVENT_CONNECTED, nullptr);

            break;
        }
        case IP_EVENT_STA_LOST_IP:
            ESP_LOGI(TAG, "Lost IP address and IP address is reset to 0");
            break;
        case IP_EVENT_GOT_IP6:
            ESP_LOGI(TAG, "IPv6 is preferred");
            break;
        case IP_EVENT_ETH_GOT_IP:
            ESP_LOGI(TAG, "Obtained IP address");
            break;
        case IP_EVENT_AP_STAIPASSIGNED:
            ESP_LOGI(TAG, "IP assigned");
            break;
        default:
            ESP_LOGD(TAG, "Unhandled IP_EVENT id: %ld", static_cast<long>(id));
            break;
        }
    }
}
