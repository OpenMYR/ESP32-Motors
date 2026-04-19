#include <unity.h>
#include <string>

#include "../../../../lib/protocols/WebServerUtils.h"

// Native env builds only test sources, so include module implementation directly.
#include "../../../../lib/protocols/WebServerUtils.cpp"

void test_extractUriPath_maps_root_and_ota_routes(void)
{
    char path[64];
    TEST_ASSERT_TRUE(WebServerUtils::extractUriPath("/", path, sizeof(path)));
    TEST_ASSERT_EQUAL_STRING("/index.html", path);

    TEST_ASSERT_TRUE(WebServerUtils::extractUriPath("/ota", path, sizeof(path)));
    TEST_ASSERT_EQUAL_STRING("/Config.html", path);
}

void test_extractUriPath_strips_query_and_rejects_traversal(void)
{
    char path[64];
    TEST_ASSERT_TRUE(WebServerUtils::extractUriPath("/hello.html?x=1", path, sizeof(path)));
    TEST_ASSERT_EQUAL_STRING("/hello.html", path);

    TEST_ASSERT_FALSE(WebServerUtils::extractUriPath("/../secret", path, sizeof(path)));
    TEST_ASSERT_FALSE(WebServerUtils::extractUriPath("/foo\\bar", path, sizeof(path)));
    TEST_ASSERT_FALSE(WebServerUtils::extractUriPath("", path, sizeof(path)));
}

void test_extractUriPath_fails_when_target_buffer_too_small(void)
{
    char tiny[4];
    TEST_ASSERT_FALSE(WebServerUtils::extractUriPath("/", tiny, sizeof(tiny)));
    TEST_ASSERT_FALSE(WebServerUtils::extractUriPath("/ota", tiny, sizeof(tiny)));
}

void test_basicAuthMatches_accepts_valid_header(void)
{
    TEST_ASSERT_TRUE(WebServerUtils::basicAuthMatches("Basic YWRtaW46c2VjcmV0", "admin", "secret"));
    TEST_ASSERT_TRUE(WebServerUtils::basicAuthMatches("basic YWRtaW46c2VjcmV0", "admin", "secret"));
}

void test_basicAuthMatches_rejects_invalid_or_malformed_headers(void)
{
    TEST_ASSERT_FALSE(WebServerUtils::basicAuthMatches("Bearer token", "admin", "secret"));
    TEST_ASSERT_FALSE(WebServerUtils::basicAuthMatches("Basic YWRtaW46d3Jvbmc=", "admin", "secret"));
    TEST_ASSERT_FALSE(WebServerUtils::basicAuthMatches("Basic not-base64!", "admin", "secret"));
    TEST_ASSERT_FALSE(WebServerUtils::basicAuthMatches("Basic YQBi", "admin", "secret"));
}

void test_basicAuthMatches_exercises_base64_plus_slash_and_invalid_char_paths(void)
{
    // Contains '+' and '/' in base64 alphabet; decode succeeds but credentials mismatch.
    TEST_ASSERT_FALSE(WebServerUtils::basicAuthMatches("Basic KysvLw==", "admin", "secret"));
    // Length is valid but '*' is not base64 alphabet; should fail in decoder character path.
    TEST_ASSERT_FALSE(WebServerUtils::basicAuthMatches("Basic QUJ*RA==", "admin", "secret"));
}

void test_basicAuthMatches_rejects_overlong_expected_credentials_buffer(void)
{
    std::string long_user(140, 'a');
    TEST_ASSERT_FALSE(WebServerUtils::basicAuthMatches("Basic YTpi", long_user.c_str(), "b"));
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(test_extractUriPath_maps_root_and_ota_routes);
    RUN_TEST(test_extractUriPath_strips_query_and_rejects_traversal);
    RUN_TEST(test_extractUriPath_fails_when_target_buffer_too_small);
    RUN_TEST(test_basicAuthMatches_accepts_valid_header);
    RUN_TEST(test_basicAuthMatches_rejects_invalid_or_malformed_headers);
    RUN_TEST(test_basicAuthMatches_exercises_base64_plus_slash_and_invalid_char_paths);
    RUN_TEST(test_basicAuthMatches_rejects_overlong_expected_credentials_buffer);
    return UNITY_END();
}
