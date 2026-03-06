#ifndef MYR_UDP_SRV_H
#define MYR_UDP_SRV_H

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "CommandParser.h"
#include "Op.h"

#define CTRL_PACKET_LEN_BYTES 11
#define WIFI_PACKET_LEN_BYTES 96

/**
 * @brief UDP ingress/egress service for command and discovery traffic.
 *
 * Responsibilities:
 * - listen for motor and Wi-Fi command packets on UDP port 4120
 * - dispatch decoded packets into CommandParser
 * - broadcast discovery prompts and command acknowledgements on UDP port 4140
 */
class udp_srv
{
    public:
        /**
         * @brief Construct a stopped UDP service instance.
         */
        udp_srv();

        /**
         * @brief Stop sockets/tasks if still running.
         */
        ~udp_srv();

        /**
         * @brief Start sockets and receive task.
         */
        void begin();

        /**
         * @brief Send a discovery prompt if the internal schedule allows it.
         */
        void prompt_broadcast();

        /**
         * @brief Stop service task and close sockets.
         */
        void end();

    private:
        /**
         * @brief FreeRTOS entry shim for udp_task().
         */
        static void udp_task_entry(void *arg);

        /**
         * @brief Receive loop for command packets.
         */
        void udp_task();

        /**
         * @brief Decode and dispatch a received UDP payload.
         *
         * @param data Raw packet data.
         * @param len Packet length in bytes.
         * @param remote_addr Sender IPv4 address in host-order.
         */
        void handle_packet(const uint8_t *data, size_t len, uint32_t remote_addr);

        /**
         * @brief Broadcast a command response packet.
         */
        void broadcast_ack(command_response_packet&);

        /**
         * @brief Compute next allowed prompt timestamp with jitter/backoff policy.
         *
         * @param now_us Current monotonic time in microseconds.
         */
        void schedule_next_prompt(int64_t now_us);

        int rx_socket;
        int tx_socket;
        TaskHandle_t task_handle;
        bool running;
        int64_t service_started_us;
        int64_t next_prompt_due_us;
};

#endif // MYR_UDP_SRV_H
