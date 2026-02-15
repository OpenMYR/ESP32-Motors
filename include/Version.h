#pragma once

#include <stdint.h>

// Expose the app semantic version to firmware code.
// Values come from CMake definitions set in the top-level CMakeLists.txt.
#ifndef APP_VER_MAJOR
#define APP_VER_MAJOR 0
#endif
#ifndef APP_VER_MINOR
#define APP_VER_MINOR 0
#endif
#ifndef APP_VER_PATCH
#define APP_VER_PATCH 0
#endif
#ifndef APP_VER_STRING
#define APP_VER_STRING "0.0.0"
#endif

struct AppVersion {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    const char *string_repr;
};

static constexpr AppVersion kAppVersion = {
    static_cast<uint8_t>(APP_VER_MAJOR),
    static_cast<uint8_t>(APP_VER_MINOR),
    static_cast<uint8_t>(APP_VER_PATCH),
    APP_VER_STRING,
};

inline AppVersion get_app_version() { return kAppVersion; }

