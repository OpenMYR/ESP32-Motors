#include <Arduino.h>
#include <unity.h>

#include "WifiController.h"
#include <Preferences.h>
#include "config/DefaultConfig.h"

static Preferences preferences;

void setUp(void) {
    // set stuff up here    
}

void tearDown(void) {
    // clean stuff up here
}

void test_wifi_default_to_ap(void){
    
    preferences.begin("myr", false);
    preferences.putUChar(WifiController::MYR_WIFI_PREF_TAG_INIT, 1);
    preferences.putUChar(WifiController::MYR_WIFI_PREF_TAG_MODE, MYR_WIFI_MODE_AP);
    preferences.end();
    TEST_ASSERT_EQUAL(ESP_OK, WifiController::init());
    TEST_ASSERT_EQUAL(WifiController::MYR_WIFI_STATE_AP, WifiController::getWiFiState());
}

void test_wifi_sta_to_sta(void){

    String ssid = "test";
    String pass = "test";

    TEST_ASSERT_EQUAL(ESP_OK, WifiController::tryConnectToSta(&ssid, &pass));
    TEST_ASSERT_EQUAL(WifiController::MYR_WIFI_STATE_AP_STA_CONNECTING, WifiController::getWiFiState());   // can this be MYR_WIFI_STATE_STA_CONNECTING or MYR_WIFI_STATE_AP_STA_CONNECTING
}

void test_wifi_sta_to_ap(void){

    WifiController::fireWifiEvent(WifiController::MYR_WIFI_EVENT_DISCONNECT, NULL);
    TEST_ASSERT_EQUAL(WifiController::MYR_WIFI_STATE_AP, WifiController::getWiFiState());
}

void test_wifi_sta_credentials(void){

    esp_err_t err;
    String ssid = "Test";
    String pass = "Test";
    
    TEST_ASSERT_EQUAL(ESP_OK, WifiController::setDefaultStaCredentials(&ssid, &pass));
}

void test_wifi_ap_credentials(void){

    String ssid = "Test";
    String pass = "Test";
    
    TEST_ASSERT_EQUAL(ESP_OK, WifiController::setDefaultApCredentials(&ssid, &pass));
}

void test_setDefaultApCredentials(void) {
    String ssid = "TestAP";
    String pass = "password";
    TEST_ASSERT_EQUAL(ESP_OK, WifiController::setDefaultApCredentials(&ssid, &pass));
}

void test_setDefaultStaCredentials(void) {
    String ssid = "TestSTA";
    String pass = "password";
    TEST_ASSERT_EQUAL(ESP_OK, WifiController::setDefaultStaCredentials(&ssid, &pass));
}

void setup()
{

    delay(2000); // service delay
    UNITY_BEGIN();

    RUN_TEST(test_wifi_default_to_ap);
    RUN_TEST(test_wifi_sta_credentials);
    RUN_TEST(test_wifi_sta_to_sta);
    RUN_TEST(test_wifi_sta_to_ap);
    RUN_TEST(test_wifi_ap_credentials);
    RUN_TEST(test_setDefaultApCredentials);
    RUN_TEST(test_setDefaultStaCredentials);

    UNITY_END(); // stop unit testing
}

void loop()
{
}