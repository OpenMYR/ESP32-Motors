#include <Arduino.h>
#include <unity.h>

#include "WifiController.h"
#include "../../src/config/DefaultConfig.h"


void setUp(void) {
    // set stuff up here
    WifiController::init();
}

void tearDown(void) {
    // clean stuff up here
}

void test_wifi_default_to_sta(void){

    esp_err_t err;
    err = WifiController::changeMode(MYR_WIFI_MODE_STATION, false);
    TEST_ASSERT_TRUE(err == false);
}

void test_wifi_sta_to_sta(void){

    esp_err_t err;
    err = WifiController::changeMode(MYR_WIFI_MODE_STATION, false);
    TEST_ASSERT_TRUE(err == false);
}

void test_wifi_sta_to_ap(void){

    esp_err_t err;
    err = WifiController::changeMode(MYR_WIFI_MODE_AP, false);
    TEST_ASSERT_TRUE(err == false);
}

void test_wifi_sta_credentials(void){

    esp_err_t err;
    String ssid = "Test";
    String pass = "Test";
    
    err = WifiController::setStaCredentials(&ssid, &pass, true);
    TEST_ASSERT_TRUE(err == false);

    err = WifiController::changeMode(MYR_WIFI_MODE_STATION, false);
    TEST_ASSERT_TRUE(err == false);
}

void test_wifi_ap_credentials(void){

    esp_err_t err;
    String ssid = "Test";
    String pass = "Test";
    
    err = WifiController::setApCredentials(&ssid, &pass, true);
    TEST_ASSERT_TRUE(err == false);
    
    err = WifiController::changeMode(MYR_WIFI_MODE_AP, true);
    TEST_ASSERT_TRUE(err == false);
}

void setup()
{

    delay(2000); // service delay
    UNITY_BEGIN();

    RUN_TEST(test_wifi_default_to_sta);
    RUN_TEST(test_wifi_sta_to_sta);
    RUN_TEST(test_wifi_sta_to_ap);
    //RUN_TEST(test_wifi_sta_credentials);
    //RUN_TEST(test_wifi_ap_credentials);

    UNITY_END(); // stop unit testing
}

void loop()
{
}