#include "FileIO.h"

#include "esp_log.h"
#include "esp_littlefs.h"

namespace {
const char *TAG = "FileIO";
const char *kFsBasePath = "/littlefs";
const char *kFsPartitionLabel = "littlefs";
} // namespace

bool FileIO::init() {
    esp_vfs_littlefs_conf_t conf = {};
    conf.base_path = kFsBasePath;
    conf.partition_label = kFsPartitionLabel;
    conf.format_if_mount_failed = false;

    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "LittleFS mount failed: %s", esp_err_to_name(err));
        return false;
    }

    size_t total = 0;
    size_t used = 0;
    err = esp_littlefs_info(conf.partition_label, &total, &used);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LittleFS info failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "LittleFS mounted: %u/%u bytes used",
             static_cast<unsigned>(used),
             static_cast<unsigned>(total));
    return true;
}
