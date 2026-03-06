#include "udp_srv.h"

#include <errno.h>
#include <string.h>

#include <arpa/inet.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>

#include "UdpPromptScheduler.h"
#include "WifiController.h"
#include "esp_log.h"
#include "esp_netif_ip_addr.h"
#include "esp_system.h"
#include "esp_timer.h"

namespace {
const char *TAG = "udp_srv";
constexpr uint16_t kListenPort = 4120;
constexpr uint16_t kBroadcastPort = 4140;
constexpr TickType_t kPromptIntervalTicks = pdMS_TO_TICKS(1000);
constexpr char kPromptMessage[] = "HEY EVERYBODY! I'M A MOTOR";

}

udp_srv::udp_srv()
    : rx_socket(-1),
      tx_socket(-1),
      task_handle(nullptr),
      running(false),
      service_started_us(0),
      next_prompt_due_us(0)
{
}

udp_srv::~udp_srv()
{
    end();
}

void udp_srv::begin()
{
    if (running) return;

    std::function<void(command_response_packet &)> f =
        std::bind(&udp_srv::broadcast_ack, this, std::placeholders::_1);
    CommandParser::register_udp_ack_func(f);

    rx_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (rx_socket < 0) {
        ESP_LOGE(TAG, "rx socket create failed: errno=%d", errno);
        return;
    }

    tx_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (tx_socket < 0) {
        ESP_LOGE(TAG, "tx socket create failed: errno=%d", errno);
        end();
        return;
    }

    const int enable_broadcast = 1;
    if (setsockopt(tx_socket, SOL_SOCKET, SO_BROADCAST, &enable_broadcast, sizeof(enable_broadcast)) < 0) {
        ESP_LOGE(TAG, "tx socket broadcast enable failed: errno=%d", errno);
        end();
        return;
    }

    struct timeval timeout = {};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    if (setsockopt(rx_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        ESP_LOGE(TAG, "rx socket timeout set failed: errno=%d", errno);
        end();
        return;
    }

    sockaddr_in listen_addr = {};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(kListenPort);
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(rx_socket, reinterpret_cast<sockaddr *>(&listen_addr), sizeof(listen_addr)) < 0) {
        ESP_LOGE(TAG, "rx socket bind failed: errno=%d", errno);
        end();
        return;
    }

    running = true;
    service_started_us = esp_timer_get_time();
    next_prompt_due_us = service_started_us;
    BaseType_t task_ok = xTaskCreate(&udp_srv::udp_task_entry, "udp_srv", 4096, this, 5, &task_handle);
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "udp task create failed");
        end();
        return;
    }

    ESP_LOGI(TAG, "UDP service started");
}

void udp_srv::udp_task_entry(void *arg)
{
    static_cast<udp_srv *>(arg)->udp_task();
}

void udp_srv::udp_task()
{
    uint8_t buffer[kWifiPacketLenBytes];

    while (running) {
        sockaddr_in remote_addr = {};
        socklen_t remote_len = sizeof(remote_addr);
        int len = recvfrom(rx_socket,
                           buffer,
                           sizeof(buffer),
                           0,
                           reinterpret_cast<sockaddr *>(&remote_addr),
                           &remote_len);
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            if (running) {
                ESP_LOGW(TAG, "recvfrom failed: errno=%d", errno);
            }
            continue;
        }

        handle_packet(buffer, static_cast<size_t>(len), ntohl(remote_addr.sin_addr.s_addr));
    }

    task_handle = nullptr;
    vTaskDelete(nullptr);
}

void udp_srv::handle_packet(const uint8_t *data, size_t len, uint32_t remote_addr)
{
    ip4_addr_t remote_ip = {};
    remote_ip.addr = htonl(remote_addr);

    if (len == kCtrlPacketLenBytes) {
        Op motor_packet(const_cast<uint8_t *>(data), remote_addr);
        CommandParser::motor_process_command(motor_packet, remote_ip);
        return;
    }

    if (len == kWifiPacketLenBytes) {
        wifi_command_packet wifi_packet = {};
        memcpy(&wifi_packet, data, sizeof(wifi_packet));
        CommandParser::wifi_process_command(wifi_packet, remote_ip);
    }
}

void udp_srv::prompt_broadcast()
{
    if (tx_socket < 0) return;

    const int64_t now_us = esp_timer_get_time();
    if (now_us < next_prompt_due_us) return;

    esp_ip4_addr_t broadcast_addr = {};
    esp_err_t addr_err = WifiController::getActiveBroadcastAddress(&broadcast_addr);
    if (addr_err != ESP_OK) {
        ESP_LOGW(TAG,
                 "prompt broadcast skipped: no active broadcast addr (%s), wifi_state=%d",
                 esp_err_to_name(addr_err),
                 static_cast<int>(WifiController::getWiFiState()));
        schedule_next_prompt(now_us);
        return;
    }
    sockaddr_in dest_addr = {};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(kBroadcastPort);
    dest_addr.sin_addr.s_addr = broadcast_addr.addr;
    ESP_LOGI(TAG, "prompt broadcast target=" IPSTR ":%u", IP2STR(&broadcast_addr), static_cast<unsigned>(kBroadcastPort));

    int sent = sendto(tx_socket,
                      kPromptMessage,
                      sizeof(kPromptMessage) - 1,
                      0,
                      reinterpret_cast<sockaddr *>(&dest_addr),
                      sizeof(dest_addr));
    if (sent < 0) {
        ESP_LOGW(TAG, "prompt broadcast failed: errno=%d", errno);
        schedule_next_prompt(now_us);
        return;
    }
    schedule_next_prompt(now_us);
}

void udp_srv::end()
{
    if (!running && rx_socket < 0 && tx_socket < 0) return;

    running = false;

    if (rx_socket >= 0) {
        shutdown(rx_socket, 0);
        close(rx_socket);
        rx_socket = -1;
    }

    if (tx_socket >= 0) {
        close(tx_socket);
        tx_socket = -1;
    }

    if (task_handle != nullptr) {
        vTaskDelay(kPromptIntervalTicks);
    }
}

void udp_srv::broadcast_ack(command_response_packet &packet)
{
    if (tx_socket < 0) return;

    esp_ip4_addr_t broadcast_addr = {};
    esp_err_t addr_err = WifiController::getActiveBroadcastAddress(&broadcast_addr);
    if (addr_err != ESP_OK) {
        ESP_LOGW(TAG,
                 "ack broadcast skipped: no active broadcast addr (%s), wifi_state=%d",
                 esp_err_to_name(addr_err),
                 static_cast<int>(WifiController::getWiFiState()));
        return;
    }
    sockaddr_in dest_addr = {};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(kBroadcastPort);
    dest_addr.sin_addr.s_addr = broadcast_addr.addr;
    ESP_LOGI(TAG, "ack broadcast target=" IPSTR ":%u", IP2STR(&broadcast_addr), static_cast<unsigned>(kBroadcastPort));

    int sent = sendto(tx_socket,
                      &packet,
                      sizeof(packet),
                      0,
                      reinterpret_cast<sockaddr *>(&dest_addr),
                      sizeof(dest_addr));
    if (sent < 0) {
        ESP_LOGW(TAG, "ack broadcast failed: errno=%d", errno);
    }
}

void udp_srv::schedule_next_prompt(int64_t now_us)
{
    const int64_t elapsed_us = now_us - service_started_us;
    const int64_t jitter_limit_us =
        elapsed_us < UdpPromptScheduler::kPromptWarmupUs ? UdpPromptScheduler::kWarmupJitterUs
                                                         : UdpPromptScheduler::kSteadyJitterUs;
    const uint32_t jitter_span_us = static_cast<uint32_t>((jitter_limit_us * 2) + 1);
    const int64_t jitter_us = static_cast<int64_t>(esp_random() % jitter_span_us) - jitter_limit_us;
    next_prompt_due_us = now_us + UdpPromptScheduler::computeNextIntervalUs(elapsed_us, jitter_us);
}
