
#include "FileIO.h"
#include "WebServer.h"
#include "WifiController.h"
#include "OpBuffer.h"
#include "CommandLayer.h"
#include "Version.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

namespace {
const char *TAG = "Main";
bool s_ota_pending_verify = false;
int64_t s_ota_deadline_us = 0;
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

    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK &&
        ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
        s_ota_pending_verify = true;
        s_ota_deadline_us = esp_timer_get_time() + 10 * 1000 * 1000; // 10s grace
        ESP_LOGI(TAG, "OTA image pending verification; deferring validation for 10s");
    }

    const auto version = get_app_version();
    ESP_LOGI(TAG, "App version %s (major=%u minor=%u patch=%u)", version.string_repr,
             version.major, version.minor, version.patch);
    ESP_LOGI(TAG, "IDF baseline started");
    while (true) {
        if (s_ota_pending_verify && esp_timer_get_time() >= s_ota_deadline_us) {
            esp_err_t mark_err = esp_ota_mark_app_valid_cancel_rollback();
            if (mark_err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to mark app valid: %s", esp_err_to_name(mark_err));
            } else {
                ESP_LOGI(TAG, "OTA image marked valid; rollback canceled");
                s_ota_pending_verify = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
