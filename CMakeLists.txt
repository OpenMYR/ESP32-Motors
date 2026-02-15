cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Application semantic version (major.minor.patch).
# Keep this in sync with your release tags. The suffix (git describe, "-dirty", etc.)
# will still be appended by IDF if PROJECT_VER is not an exact tag.
set(PROJECT_VER "0.0.0")

# Extract numeric components for firmware use.
string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)" _ver_match "${PROJECT_VER}")
set(PROJECT_VER_MAJOR ${CMAKE_MATCH_1})
set(PROJECT_VER_MINOR ${CMAKE_MATCH_2})
set(PROJECT_VER_PATCH ${CMAKE_MATCH_3})

# Expose components as preprocessor symbols so code can read them directly.
add_compile_definitions(
    APP_VER_MAJOR=${PROJECT_VER_MAJOR}
    APP_VER_MINOR=${PROJECT_VER_MINOR}
    APP_VER_PATCH=${PROJECT_VER_PATCH}
    APP_VER_STRING="${PROJECT_VER}"
)

project(ESP32-Motors)
