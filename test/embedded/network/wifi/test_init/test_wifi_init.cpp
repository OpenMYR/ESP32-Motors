#include <unity.h>

#include "WifiController.h"
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
    if (err == ESP_OK) {
        TEST_ASSERT_EQUAL(ESP_OK, nvs_erase_all(nvsHandle));
        TEST_ASSERT_EQUAL(ESP_OK, nvs_commit(nvsHandle));
        nvs_close(nvsHandle);
    }
}
} // namespace

void setUp(void) {
    clear_wifi_nvs();
}

void tearDown(void) {
}

void test_wifi_init_returns_ok(void) {
    TEST_ASSERT_EQUAL(ESP_OK, WifiController::init());
}

extern "C" void app_main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_wifi_init_returns_ok);
    UNITY_END();
}
