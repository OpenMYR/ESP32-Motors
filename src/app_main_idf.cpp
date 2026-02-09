#include "FileIO.h"
#include "WebServer.h"
#include "WifiController.h"
#include "OpBuffer.h"
#include "CommandLayer.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

namespace {
const char *TAG = "Main";
}

extern "C" void __attribute__((weak)) app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = WifiController::init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WiFi init failed: %s", esp_err_to_name(err));
        return;
    }

    if (!FileIO::init()) {
        ESP_LOGE(TAG, "FileIO init failed");
        return;
    }

    OpBuffer::getInstance();
    CommandLayer::getInstance()->init();

    if (!WebServer::init()) {
        ESP_LOGE(TAG, "WebServer init failed");
        return;
    }

    ESP_LOGI(TAG, "IDF baseline started");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
