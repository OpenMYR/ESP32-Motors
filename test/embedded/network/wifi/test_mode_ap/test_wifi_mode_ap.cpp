#include <string>

#include <unity.h>

#include "WifiController.h"
#include "config/Config.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace {
void clear_wifi_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        TEST_ASSERT_EQUAL(ESP_OK, nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK && err != ESP_ERR_NVS_INVALID_STATE) {
        TEST_ASSERT_EQUAL(ESP_OK, err);
    }

    nvs_handle_t nvsHandle;
    err = nvs_open("myr", NVS_READWRITE, &nvsHandle);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(ESP_OK, nvs_erase_all(nvsHandle));
    TEST_ASSERT_EQUAL(ESP_OK, nvs_set_u8(nvsHandle, WifiController::MYR_WIFI_PREF_TAG_INIT, 1));
    TEST_ASSERT_EQUAL(ESP_OK, nvs_set_u8(nvsHandle, WifiController::MYR_WIFI_PREF_TAG_MODE, MYR_WIFI_MODE_AP));
    TEST_ASSERT_EQUAL(ESP_OK, nvs_commit(nvsHandle));
    nvs_close(nvsHandle);
}
} // namespace

void setUp(void) {
    clear_wifi_nvs();
}

void tearDown(void) {
}

void test_wifi_ap_mode_sequence(void) {
    TEST_ASSERT_EQUAL(ESP_OK, WifiController::init());
    TEST_ASSERT_EQUAL(WifiController::MYR_WIFI_STATE_AP, WifiController::getWiFiState());

    std::string ssid = "test";
    std::string pass = "test";
    TEST_ASSERT_EQUAL(ESP_OK, WifiController::tryConnectToSta(&ssid, &pass));
    WifiController::processStateEvents();
    TEST_ASSERT_EQUAL(WifiController::MYR_WIFI_STATE_AP_STA_CONNECTING, WifiController::getWiFiState());

    WifiController::fireWifiEvent(WifiController::MYR_WIFI_EVENT_DISCONNECT, nullptr);
    WifiController::processStateEvents();
    TEST_ASSERT_EQUAL(WifiController::MYR_WIFI_STATE_AP, WifiController::getWiFiState());

    std::string apSsid = "TestAP";
    std::string apPass = "password";
    std::string staSsid = "TestSTA";
    std::string staPass = "password";
    TEST_ASSERT_EQUAL(ESP_OK, WifiController::setDefaultApCredentials(&apSsid, &apPass));
    TEST_ASSERT_EQUAL(ESP_OK, WifiController::setDefaultStaCredentials(&staSsid, &staPass));
}

extern "C" void app_main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_wifi_ap_mode_sequence);
    UNITY_END();
}
