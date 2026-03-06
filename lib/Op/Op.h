#ifndef MYR_OP_H
#define MYR_OP_H

#include <stddef.h>
#include <stdint.h>

constexpr size_t kCtrlPacketLenBytes = 11;
constexpr size_t kWifiCommandSsidLen = 32;
constexpr size_t kWifiCommandPasswordLen = 63;
constexpr size_t kWifiPacketLenBytes = 1 + kWifiCommandSsidLen + kWifiCommandPasswordLen;

typedef struct Op {
    unsigned short port;
    char opcode;
    uint8_t queue;
    int32_t stepNum;
    uint16_t stepRate;
    uint8_t motorID;
    uint32_t sourceIPAddr;
    uint32_t opSeq;

    Op()
    {
        opSeq = 0;
    }

    Op(uint8_t* data)
    {
        port = (data[0] << 8) | data[1];
        opcode = data[2];
        queue = data[3];
        stepNum = (((data[4] << 8) | data[5]) << 16) | ((data[6] << 8) | data[7]);
        stepRate = (data[8] << 8) | data[9];
        motorID = data[10]; 
        sourceIPAddr = 0;
        opSeq = 0;
    }

    Op(uint8_t* data, uint32_t ip)
    {
        port = (data[0] << 8) | data[1];
        opcode = data[2];
        queue = data[3];
        stepNum = (((data[4] << 8) | data[5]) << 16) | ((data[6] << 8) | data[7]);
        stepRate = (data[8] << 8) | data[9];
        motorID = data[10]; 
        sourceIPAddr = ip;
        opSeq = 0;
    }
} Op;

struct command_response_packet {
	char opcode;
    int motor_id;
	int position;
};

struct wifi_command_packet {
	char opcode;
	char ssid[kWifiCommandSsidLen];
	char password[kWifiCommandPasswordLen];
};

static_assert(sizeof(wifi_command_packet) == kWifiPacketLenBytes,
              "wifi_command_packet size must match UDP wire size");

#endif // MYR_OP_H
