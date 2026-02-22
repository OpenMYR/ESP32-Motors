#include "WebServer.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <string>

#include "WifiController.h"
#include "OpBuffer.h"
#include "WebCommandDispatcher.h"
#include "WebServerUtils.h"
#include "cJSON.h"
#include "config/Config.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
const char *TAG = "WebServer";
httpd_handle_t g_server = nullptr;
constexpr size_t kMaxUriPathLen = 256;
constexpr size_t kMaxCommandBodyLen = 2048;
constexpr size_t kMaxAuthHeaderLen = 192;
constexpr size_t kMaxOtaChunkLen = 1024;

#if SERVO == 1
const char *kWebRoot = "/littlefs/web_srv";
#elif STEPPER == 1
const char *kWebRoot = "/littlefs/web_step";
#else
const char *kWebRoot = "/littlefs/web_srv";
#endif

enum class StaticPathStatus {
    Ok = 0,
    InvalidUri = 1,
    NotFound = 2,
};

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

const char *content_type_for_path(const char *path) {
    const char *ext = strrchr(path, '.');
    if (ext == nullptr) {
        return "text/plain";
    }
    if (strcmp(ext, ".html") == 0) {
        return "text/html";
    }
    if (strcmp(ext, ".css") == 0) {
        return "text/css";
    }
    if (strcmp(ext, ".js") == 0) {
        return "application/javascript";
    }
    if (strcmp(ext, ".json") == 0) {
        return "application/json";
    }
    if (strcmp(ext, ".svg") == 0) {
        return "image/svg+xml";
    }
    if (strcmp(ext, ".ico") == 0) {
        return "image/x-icon";
    }
    if (strcmp(ext, ".png") == 0) {
        return "image/png";
    }
    return "application/octet-stream";
}

bool file_exists(const char *path) {
    struct stat st = {};
    return stat(path, &st) == 0;
}

StaticPathStatus resolve_static_path(
    const char *uri,
    const char *root,
    char *path,
    size_t path_len) {
    char uri_path[kMaxUriPathLen];
    if (!WebServerUtils::extractUriPath(uri, uri_path, sizeof(uri_path))) return StaticPathStatus::InvalidUri;

    int n = snprintf(path, path_len, "%s%s", root, uri_path);
    if (n < 0 || static_cast<size_t>(n) >= path_len) return StaticPathStatus::InvalidUri;

    if (!file_exists(path)) return StaticPathStatus::NotFound;
    return StaticPathStatus::Ok;
}

esp_err_t send_file_chunks(httpd_req_t *req, const char *path) {
    FILE *file = fopen(path, "rb");
    if (file == nullptr) {
        ESP_LOGW(TAG, "open failed: %s errno=%d", path, errno);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, content_type_for_path(path));

    char buf[1024];
    while (true) {
        size_t read_count = fread(buf, 1, sizeof(buf), file);
        if (read_count > 0) {
            esp_err_t err = httpd_resp_send_chunk(req, buf, read_count);
            if (err != ESP_OK) {
                fclose(file);
                return err;
            }
        }

        if (read_count < sizeof(buf)) {
            if (feof(file)) {
                break;
            }
            fclose(file);
            return ESP_FAIL;
        }
    }

    fclose(file);
    return httpd_resp_send_chunk(req, nullptr, 0);
}

esp_err_t read_post_body(httpd_req_t *req, char *payload, size_t payload_len) {
    if (req->content_len <= 0) return ESP_ERR_INVALID_ARG;
    if (req->content_len >= static_cast<int>(payload_len)) return ESP_ERR_NO_MEM;

    size_t offset = 0;
    while (offset < static_cast<size_t>(req->content_len)) {
        int ret = httpd_req_recv(req, payload + offset, req->content_len - offset);
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (ret <= 0) return ESP_FAIL;
        offset += static_cast<size_t>(ret);
    }
    payload[offset] = '\0';
    return ESP_OK;
}


void request_basic_auth(httpd_req_t *req) {
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"" MYR_OTA_AUTH_REALM "\"");
    httpd_resp_sendstr(req, "Authentication required");
}

bool ensure_authenticated(httpd_req_t *req) {
    char auth_header[kMaxAuthHeaderLen];
    if (httpd_req_get_hdr_value_str(req, "Authorization", auth_header, sizeof(auth_header)) != ESP_OK) {
        request_basic_auth(req);
        return false;
    }

    const std::string otaPass = WifiController::getOTAPassword();
    if (!WebServerUtils::basicAuthMatches(auth_header, MYR_OTA_AUTH_USERNAME, otaPass.c_str())) {
        request_basic_auth(req);
        return false;
    }

    return true;
}

esp_err_t post_ota_handler(httpd_req_t *req) {
    char ota_chunk[kMaxOtaChunkLen];
    if (!ensure_authenticated(req)) {
        return ESP_FAIL;
    }

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(nullptr);
    if (update_partition == nullptr) {
        ESP_LOGE(TAG, "Failed to find OTA update partition");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return err;
    }

    int remaining = req->content_len;
    while (remaining > 0) {
        int to_read = MIN(remaining, static_cast<int>(sizeof(ota_chunk)));
        int read = httpd_req_recv(req, ota_chunk, to_read);
        if (read <= 0) {
            if (read == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            esp_ota_abort(ota_handle);
            ESP_LOGE(TAG, "Failed to receive OTA chunk");
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        err = esp_ota_write(ota_handle, ota_chunk, read);
        if (err != ESP_OK) {
            esp_ota_abort(ota_handle);
            ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
            httpd_resp_send_500(req);
            return err;
        }
        remaining -= read;
    }

    if (esp_ota_end(ota_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return err;
    }

    httpd_resp_sendstr(req, "OTA complete, rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

esp_err_t get_health_handler(httpd_req_t *req) {
    ESP_LOGD(TAG, "GET /health");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

esp_err_t get_static_handler(httpd_req_t *req) {
    char path[320];
    StaticPathStatus status = resolve_static_path(req->uri, kWebRoot, path, sizeof(path));

    if (status == StaticPathStatus::InvalidUri) {
        ESP_LOGW(TAG, "GET %s -> 400", req->uri);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Bad Request");
    }

    if (status == StaticPathStatus::NotFound) {
        ESP_LOGW(TAG, "GET %s -> 404", req->uri);
        httpd_resp_set_status(req, "404 Not Found");
        return httpd_resp_sendstr(req, "Not Found");
    }

    ESP_LOGD(TAG, "GET %s -> %s", req->uri, path);
    return send_file_chunks(req, path);
}

esp_err_t post_command_handler(httpd_req_t *req) {
    char payload[kMaxCommandBodyLen];
    esp_err_t err = read_post_body(req, payload, sizeof(payload));
    if (err == ESP_ERR_NO_MEM) {
        ESP_LOGW(TAG, "POST %s -> 413", req->uri);
        httpd_resp_set_status(req, "413 Payload Too Large");
        return httpd_resp_sendstr(req, "Payload Too Large");
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "POST %s -> 400", req->uri);
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Bad Request");
    }

    err = WebCommandDispatcher::processPayload(payload);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "POST %s -> 400 (%s)", req->uri, esp_err_to_name(err));
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Bad Request");
    }

    httpd_resp_set_status(req, "202 Accepted");
    ESP_LOGD(TAG, "POST %s -> 202", req->uri);
    return httpd_resp_send(req, nullptr, 0);
}
} // namespace

bool WebServer::init() {
    if (g_server != nullptr) {
        ESP_LOGI(TAG, "HTTP server already running");
        return true;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    // POST processing includes request-body buffering and JSON parse work.
    config.stack_size = 8192;

    esp_err_t err = httpd_start(&g_server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        g_server = nullptr;
        return false;
    }

    httpd_uri_t health_uri = {};
    health_uri.uri = "/health";
    health_uri.method = HTTP_GET;
    health_uri.handler = get_health_handler;
    health_uri.user_ctx = nullptr;
    err = httpd_register_uri_handler(g_server, &health_uri);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register /health failed: %s", esp_err_to_name(err));
        reset();
        return false;
    }
    ESP_LOGD(TAG, "Registered route: GET /health");

    httpd_uri_t command_uri = {};
    command_uri.uri = "/";
    command_uri.method = HTTP_POST;
    command_uri.handler = post_command_handler;
    command_uri.user_ctx = nullptr;
    err = httpd_register_uri_handler(g_server, &command_uri);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register POST / failed: %s", esp_err_to_name(err));
        reset();
        return false;
    }
    ESP_LOGD(TAG, "Registered route: POST /");

    httpd_uri_t static_uri = {};
    static_uri.uri = "/*";
    static_uri.method = HTTP_GET;
    static_uri.handler = get_static_handler;
    static_uri.user_ctx = nullptr;
    err = httpd_register_uri_handler(g_server, &static_uri);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register static failed: %s", esp_err_to_name(err));
        reset();
        return false;
    }
    ESP_LOGD(TAG, "Registered route: GET /*");

    httpd_uri_t ota_uri = {};
    ota_uri.uri = "/ota";
    ota_uri.method = HTTP_POST;
    ota_uri.handler = post_ota_handler;
    ota_uri.user_ctx = nullptr;
    err = httpd_register_uri_handler(g_server, &ota_uri);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register POST /ota failed: %s", esp_err_to_name(err));
        reset();
        return false;
    }
    ESP_LOGD(TAG, "Registered route: POST /ota (Basic auth)");

    ESP_LOGI(TAG, "HTTP static root: %s", kWebRoot);
    ESP_LOGI(TAG, "HTTP server started");
    return true;
}

void WebServer::reset() {
    if (g_server != nullptr) {
        ESP_LOGI(TAG, "HTTP server stopping");
        httpd_stop(g_server);
        g_server = nullptr;
        ESP_LOGI(TAG, "HTTP server stopped");
    }
}
