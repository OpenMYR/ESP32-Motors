#ifndef MYR_WEB_SERVER_UTILS_H
#define MYR_WEB_SERVER_UTILS_H

#include <stddef.h>

namespace WebServerUtils {
bool extractUriPath(const char *uri, char *uri_path, size_t uri_path_len);
bool basicAuthMatches(const char *auth_header, const char *username, const char *password);
}

#endif // MYR_WEB_SERVER_UTILS_H
