#include "WebServer.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <string>

#include "WifiController.h"
#include "OpBuffer.h"
#include "cJSON.h"
#include "config/Config.h"
#include "esp_http_server.h"
#include "esp_log.h"

namespace {
const char *TAG = "WebServer";
httpd_handle_t g_server = nullptr;
constexpr size_t kMaxUriPathLen = 256;
constexpr size_t kMaxCommandBodyLen = 2048;

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

bool extract_uri_path(const char *uri, char *uri_path, size_t uri_path_len) {
    size_t uri_len = strcspn(uri, "?");
    if (uri_len == 0 || uri_len >= uri_path_len) return false;

    memcpy(uri_path, uri, uri_len);
    uri_path[uri_len] = '\0';

    if (strcmp(uri_path, "/") == 0) {
        strlcpy(uri_path, "/index.html", uri_path_len);
    }

    if (uri_path[0] != '/') return false;
    if (strstr(uri_path, "..") != nullptr) return false;
    if (strchr(uri_path, '\\') != nullptr) return false;

    return true;
}

StaticPathStatus resolve_static_path(
    const char *uri,
    const char *root,
    char *path,
    size_t path_len) {
    char uri_path[kMaxUriPathLen];
    if (!extract_uri_path(uri, uri_path, sizeof(uri_path))) return StaticPathStatus::InvalidUri;

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

esp_err_t parse_config_pair(cJSON *data, std::string *lhs, std::string *rhs) {
    if (!cJSON_IsArray(data)) return ESP_ERR_INVALID_ARG;
    if (cJSON_GetArraySize(data) != 2) return ESP_ERR_INVALID_ARG;

    cJSON *lhs_item = cJSON_GetArrayItem(data, 0);
    cJSON *rhs_item = cJSON_GetArrayItem(data, 1);
    if (!cJSON_IsString(lhs_item) || lhs_item->valuestring == nullptr) return ESP_ERR_INVALID_ARG;
    if (!cJSON_IsString(rhs_item) || rhs_item->valuestring == nullptr) return ESP_ERR_INVALID_ARG;

    *lhs = lhs_item->valuestring;
    *rhs = rhs_item->valuestring;
    return ESP_OK;
}

esp_err_t handle_config_command(char code, cJSON *data) {
    if (code == 'C') {
        std::string ssid;
        std::string pass;
        esp_err_t err = parse_config_pair(data, &ssid, &pass);
        if (err != ESP_OK) return err;

        err = WifiController::tryConnectToSta(&ssid, &pass);
        if (err != ESP_OK) return err;

        err = WifiController::setDefaultStaCredentials(&ssid, &pass);
        if (err != ESP_OK) return err;

        return WifiController::setDefaultMode(MYR_WIFI_MODE_STATION);
    }

    if (code == 'D') {
        WifiController::fireWifiEvent(WifiController::MYR_WIFI_EVENT_DISCONNECT, nullptr);
        return WifiController::setDefaultMode(MYR_WIFI_MODE_AP);
    }

    if (code == 'O') {
        std::string old_pass;
        std::string new_pass;
        esp_err_t err = parse_config_pair(data, &old_pass, &new_pass);
        if (err != ESP_OK) return err;

        WifiController::changeOTAPass(&old_pass, &new_pass);
        return ESP_OK;
    }

    ESP_LOGD(TAG, "POST command '%c' ignored in IDF baseline", code);
    return ESP_OK;
}

esp_err_t parse_motor_data(cJSON *data, Op *op) {
    if (!cJSON_IsArray(data)) return ESP_ERR_INVALID_ARG;
    if (cJSON_GetArraySize(data) != 4) return ESP_ERR_INVALID_ARG;

    cJSON *motorIdItem = cJSON_GetArrayItem(data, 0);
    cJSON *queueItem = cJSON_GetArrayItem(data, 1);
    cJSON *stepNumItem = cJSON_GetArrayItem(data, 2);
    cJSON *stepRateItem = cJSON_GetArrayItem(data, 3);
    if (!cJSON_IsNumber(motorIdItem)) return ESP_ERR_INVALID_ARG;
    if (!cJSON_IsNumber(queueItem)) return ESP_ERR_INVALID_ARG;
    if (!cJSON_IsNumber(stepNumItem)) return ESP_ERR_INVALID_ARG;
    if (!cJSON_IsNumber(stepRateItem)) return ESP_ERR_INVALID_ARG;

    op->port = 0;
    op->motorID = static_cast<uint8_t>(motorIdItem->valueint);
    op->queue = static_cast<uint8_t>(queueItem->valueint);
    op->stepNum = static_cast<int32_t>(stepNumItem->valueint);
    op->stepRate = static_cast<uint16_t>(stepRateItem->valueint);
    return ESP_OK;
}

esp_err_t enqueue_motor_op(const Op *op) {
    OpBuffer *buffer = OpBuffer::getInstance();

    if (op->queue == 0) {
        buffer->clear(op->motorID);
        buffer->killCurrentOp(op->motorID);
    }

    Op opCopy = *op;
    if (buffer->storeOp(&opCopy) < 0) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t handle_motor_motion_command(char code, cJSON *data) {
    Op op = {};
    esp_err_t err = parse_motor_data(data, &op);
    if (err != ESP_OK) return err;

    op.opcode = code;
    return enqueue_motor_op(&op);
}

esp_err_t handle_motor_config_command(char code, cJSON *data) {
    Op op = {};
    esp_err_t err = parse_motor_data(data, &op);
    if (err != ESP_OK) return err;

    op.opcode = code;
    // Preserve legacy payload mapping for motor config commands.
    op.stepNum = 0;
    return enqueue_motor_op(&op);
}

esp_err_t process_command_payload(const char *payload) {
    cJSON *root = cJSON_Parse(payload);
    if (root == nullptr) return ESP_ERR_INVALID_ARG;

    cJSON *commands = cJSON_GetObjectItemCaseSensitive(root, "commands");
    if (!cJSON_IsArray(commands)) {
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    cJSON *command = nullptr;
    cJSON_ArrayForEach(command, commands) {
        cJSON *code = cJSON_GetObjectItemCaseSensitive(command, "code");
        cJSON *data = cJSON_GetObjectItemCaseSensitive(command, "data");

        if (!cJSON_IsString(code) || code->valuestring == nullptr) {
            err = ESP_ERR_INVALID_ARG;
            break;
        }
        if (strlen(code->valuestring) != 1) {
            err = ESP_ERR_INVALID_ARG;
            break;
        }

        char opcode = code->valuestring[0];
        if (opcode == 'C' || opcode == 'D' || opcode == 'O') {
            err = handle_config_command(opcode, data);
        } else if (opcode == 'M' || opcode == 'S' || opcode == 'G' || opcode == 'I') {
            err = handle_motor_motion_command(opcode, data);
        } else if (opcode == 'U' || opcode == 'H' || opcode == 'L') {
            err = handle_motor_config_command(opcode, data);
        } else {
            ESP_LOGD(TAG, "POST command '%c' ignored", opcode);
            err = ESP_OK;
        }
        if (err != ESP_OK) break;
    }

    cJSON_Delete(root);
    return err;
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

    ESP_LOGI(TAG, "GET %s -> %s", req->uri, path);
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

    err = process_command_payload(payload);
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
