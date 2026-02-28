#include <unity.h>
#include <algorithm>
#include <cctype>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
std::string g_string_under_test;
}

void setUp(void) {
    g_string_under_test = "Hello, world!";
}

void tearDown(void) {
    g_string_under_test.clear();
}

void test_string_concat(void) {
    const std::string hello = "Hello, ";
    const std::string world = "world!";
    TEST_ASSERT_EQUAL_STRING(g_string_under_test.c_str(), (hello + world).c_str());
}

void test_string_substring(void) {
    TEST_ASSERT_EQUAL_STRING("Hello", g_string_under_test.substr(0, 5).c_str());
}

void test_string_index_of(void) {
    TEST_ASSERT_EQUAL(7, static_cast<int>(g_string_under_test.find('w')));
}

void test_string_equal_ignore_case(void) {
    std::string upper = g_string_under_test;
    std::transform(
        upper.begin(),
        upper.end(),
        upper.begin(),
        [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
    TEST_ASSERT_EQUAL_STRING("HELLO, WORLD!", upper.c_str());
}

void test_string_to_upper_case(void) {
    std::transform(
        g_string_under_test.begin(),
        g_string_under_test.end(),
        g_string_under_test.begin(),
        [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
    TEST_ASSERT_EQUAL_STRING("HELLO, WORLD!", g_string_under_test.c_str());
}

void test_string_replace(void) {
    std::replace(g_string_under_test.begin(), g_string_under_test.end(), '!', '?');
    TEST_ASSERT_EQUAL_STRING("Hello, world?", g_string_under_test.c_str());
}

extern "C" void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    UNITY_BEGIN();

    RUN_TEST(test_string_concat);
    RUN_TEST(test_string_substring);
    RUN_TEST(test_string_index_of);
    RUN_TEST(test_string_equal_ignore_case);
    RUN_TEST(test_string_to_upper_case);
    RUN_TEST(test_string_replace);

    UNITY_END(); // stop unit testing
}
