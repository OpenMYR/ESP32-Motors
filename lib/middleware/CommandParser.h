#ifndef MYR_COMMANDPARSER_H
#define MYR_COMMANDPARSER_H

#include <functional>
#include <string>
#include <stdint.h>

#include "esp_err.h"
#include "lwip/ip4_addr.h"

#include "Op.h"

/**
 * @brief Shared command handling for UDP and web ingress paths.
 */
class CommandParser
{
public:
    /**
     * @brief Injectable Wi-Fi operations used by command handlers.
     *
     * Production code uses the default `WifiController` bindings. Tests may
     * replace these callbacks to observe sequencing and inject failures.
     */
    struct WifiOps {
        esp_err_t (*tryConnectToSta)(const std::string *ssid, const std::string *pass);
        esp_err_t (*setDefaultStaCredentials)(const std::string *ssid, const std::string *pass);
        esp_err_t (*setDefaultMode)(uint16_t mode);
        void (*fireDisconnectEvent)();
        esp_err_t (*changeOtaPass)(const std::string *old_pass, const std::string *new_pass);
    };

    /**
     * @brief Handle a UDP Wi-Fi configuration packet.
     *
     * @param packet Parsed Wi-Fi command packet received over UDP.
     * @param addr Source IPv4 address for the packet.
     */
    static void wifi_process_command(struct wifi_command_packet, ip4_addr_t);

    /**
     * @brief Handle a UDP motor command packet.
     *
     * @param packet Parsed motor command packet received over UDP.
     * @param addr Source IPv4 address for the packet.
     */
    static void motor_process_command(struct Op, ip4_addr_t);

    /**
     * @brief Process a motor operation using the shared queueing rules.
     *
     * If the operation targets queue `0`, the existing queue and active
     * operation for that motor are cleared before the new operation is queued.
     *
     * @param op Motor operation to enqueue or apply.
     * @return `ESP_OK` on success, or an error code if queueing fails.
     */
    static esp_err_t processMotorOp(const Op &op);

    /**
     * @brief Process a Wi-Fi command using the shared dispatcher rules.
     *
     * Supported opcodes are `C` (connect and persist STA credentials), `D`
     * (disconnect and return to AP mode), and `O` (change OTA password).
     *
     * @param opcode Wi-Fi command opcode.
     * @param lhs First command argument, typically SSID or current OTA password.
     * @param rhs Second command argument, typically passphrase or new OTA password.
     * @return `ESP_OK` on success, or an error describing the failed operation.
     */
    static esp_err_t processWifiCommand(WifiOpcode opcode, const std::string *lhs, const std::string *rhs);

    /**
     * @brief Replace Wi-Fi operation callbacks for tests.
     *
     * @param ops Callback table to install. A null pointer leaves the current
     *            callbacks unchanged.
     */
    static void setWifiOpsForTest(const WifiOps *ops);

    /**
     * @brief Restore the default production Wi-Fi callbacks after tests.
     */
    static void resetWifiOpsForTest();

    /**
     * @brief True while OTA mode is active and new commands are ignored.
     */
    static bool ota_active;

    /**
     * @brief Register the UDP acknowledgement callback.
     *
     * @param f Callback used to emit command response packets.
     */
    static void register_udp_ack_func(std::function<void(command_response_packet &)>f);

    /**
     * @brief Stop all motors by clearing queues and killing active operations.
     */
    static void stop_all_motors();

    /**
     * @brief Enter OTA mode by stopping all motors and blocking new commands.
     */
    static void enter_ota_mode();

    /**
     * @brief Exit OTA mode and resume command processing.
     */
    static void exit_ota_mode();

private:
    static std::function<void(command_response_packet&)> ack_func;
    static WifiOps wifiOps;
};

#endif // MYR_COMMANDPARSER_H
