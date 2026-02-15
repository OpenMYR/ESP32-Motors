#include "WebServerUtils.h"

#include <stdio.h>
#include <strings.h>
#include <string.h>

#include <stdint.h>

namespace {
int base64_value(char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

bool decode_base64(const char *encoded, uint8_t *decoded, size_t decoded_capacity, size_t *decoded_len)
{
    if (encoded == nullptr || decoded == nullptr || decoded_len == nullptr) return false;

    const size_t in_len = strlen(encoded);
    if (in_len == 0 || (in_len % 4) != 0) return false;

    size_t out = 0;
    for (size_t i = 0; i < in_len; i += 4) {
        const char c0 = encoded[i];
        const char c1 = encoded[i + 1];
        const char c2 = encoded[i + 2];
        const char c3 = encoded[i + 3];

        const int v0 = base64_value(c0);
        const int v1 = base64_value(c1);
        if (v0 < 0 || v1 < 0) return false;

        const bool pad2 = (c2 == '=');
        const bool pad3 = (c3 == '=');
        const int v2 = pad2 ? 0 : base64_value(c2);
        const int v3 = pad3 ? 0 : base64_value(c3);

        if (!pad2 && v2 < 0) return false;
        if (!pad3 && v3 < 0) return false;
        if (pad2 && !pad3) return false;
        if ((pad2 || pad3) && (i + 4 != in_len)) return false;

        const uint32_t triple = (static_cast<uint32_t>(v0) << 18) |
                                (static_cast<uint32_t>(v1) << 12) |
                                (static_cast<uint32_t>(v2) << 6) |
                                static_cast<uint32_t>(v3);

        if (out >= decoded_capacity) return false;
        decoded[out++] = static_cast<uint8_t>((triple >> 16) & 0xFF);

        if (!pad2) {
            if (out >= decoded_capacity) return false;
            decoded[out++] = static_cast<uint8_t>((triple >> 8) & 0xFF);
        }
        if (!pad3) {
            if (out >= decoded_capacity) return false;
            decoded[out++] = static_cast<uint8_t>(triple & 0xFF);
        }
    }

    *decoded_len = out;
    return true;
}
} // namespace

namespace WebServerUtils {
bool extractUriPath(const char *uri, char *uri_path, size_t uri_path_len)
{
    if (uri == nullptr || uri_path == nullptr || uri_path_len == 0) return false;

    const size_t uri_len = strcspn(uri, "?");
    if (uri_len == 0 || uri_len >= uri_path_len) return false;

    memcpy(uri_path, uri, uri_len);
    uri_path[uri_len] = '\0';

    if (strcmp(uri_path, "/") == 0) {
        if (snprintf(uri_path, uri_path_len, "%s", "/index.html") >= static_cast<int>(uri_path_len)) {
            return false;
        }
    } else if (strcmp(uri_path, "/ota") == 0) {
        if (snprintf(uri_path, uri_path_len, "%s", "/Config.html") >= static_cast<int>(uri_path_len)) {
            return false;
        }
    }

    if (uri_path[0] != '/') return false;
    if (strstr(uri_path, "..") != nullptr) return false;
    if (strchr(uri_path, '\\') != nullptr) return false;

    return true;
}

bool basicAuthMatches(const char *auth_header, const char *username, const char *password)
{
    const char auth_prefix[] = "Basic ";
    const char *encoded = nullptr;
    uint8_t decoded[128];
    size_t decoded_len = 0;
    char expected[128];

    if (!auth_header || !username || !password) return false;

    if (strncasecmp(auth_header, auth_prefix, sizeof(auth_prefix) - 1) != 0) return false;

    encoded = auth_header + sizeof(auth_prefix) - 1;
    if (!decode_base64(encoded, decoded, sizeof(decoded), &decoded_len)) return false;
    if (decoded_len == 0 || decoded_len >= sizeof(decoded)) return false;
    if (memchr(decoded, '\0', decoded_len) != nullptr) return false;

    decoded[decoded_len] = '\0';
    if (snprintf(expected, sizeof(expected), "%s:%s", username, password) >= static_cast<int>(sizeof(expected))) {
        return false;
    }
    return strcmp(reinterpret_cast<const char *>(decoded), expected) == 0;
}
} // namespace WebServerUtils
