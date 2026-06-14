#ifndef MYR_OP_H
#define MYR_OP_H

#include <stddef.h>
#include <stdint.h>

constexpr size_t kCtrlPacketLenBytes = 11;
constexpr size_t kWifiCommandSsidLen = 32;
constexpr size_t kWifiCommandPasswordLen = 63;
constexpr size_t kWifiPacketLenBytes = 1 + kWifiCommandSsidLen + kWifiCommandPasswordLen;

enum class WifiOpcode : char {
    Connect = 'C',
    Disconnect = 'D',
    ChangeOtaPassword = 'O',
};

enum class MotorOpcode : char {
    Move = 'M',
    Stop = 'S',
    Goto = 'G',
    Sleep = 'I',
    Microstep = 'U',
    Reset = 'R',
    Home = 'H',
    Limit = 'L',
    Abort = 'K',
};

inline constexpr char to_char(WifiOpcode opcode)
{
    return static_cast<char>(opcode);
}

inline constexpr char to_char(MotorOpcode opcode)
{
    return static_cast<char>(opcode);
}

inline constexpr bool try_parse_wifi_opcode(char raw, WifiOpcode *opcode)
{
    if (opcode == nullptr) return false;

    switch (raw)
    {
    case to_char(WifiOpcode::Connect):
        *opcode = WifiOpcode::Connect;
        return true;
    case to_char(WifiOpcode::Disconnect):
        *opcode = WifiOpcode::Disconnect;
        return true;
    case to_char(WifiOpcode::ChangeOtaPassword):
        *opcode = WifiOpcode::ChangeOtaPassword;
        return true;
    default:
        return false;
    }
}

inline constexpr bool try_parse_motor_opcode(char raw, MotorOpcode *opcode)
{
    if (opcode == nullptr) return false;

    switch (raw)
    {
    case to_char(MotorOpcode::Move):
        *opcode = MotorOpcode::Move;
        return true;
    case to_char(MotorOpcode::Stop):
        *opcode = MotorOpcode::Stop;
        return true;
    case to_char(MotorOpcode::Goto):
        *opcode = MotorOpcode::Goto;
        return true;
    case to_char(MotorOpcode::Sleep):
        *opcode = MotorOpcode::Sleep;
        return true;
    case to_char(MotorOpcode::Microstep):
        *opcode = MotorOpcode::Microstep;
        return true;
    case to_char(MotorOpcode::Reset):
        *opcode = MotorOpcode::Reset;
        return true;
    case to_char(MotorOpcode::Home):
        *opcode = MotorOpcode::Home;
        return true;
    case to_char(MotorOpcode::Limit):
        *opcode = MotorOpcode::Limit;
        return true;
    case to_char(MotorOpcode::Abort):
        *opcode = MotorOpcode::Abort;
        return true;
    default:
        return false;
    }
}

inline constexpr bool is_motion_opcode(MotorOpcode opcode)
{
    switch (opcode)
    {
    case MotorOpcode::Move:
    case MotorOpcode::Stop:
    case MotorOpcode::Goto:
    case MotorOpcode::Sleep:
        return true;
    default:
        return false;
    }
}

inline constexpr bool is_motor_config_opcode(MotorOpcode opcode)
{
    switch (opcode)
    {
    case MotorOpcode::Microstep:
    case MotorOpcode::Reset:
    case MotorOpcode::Home:
    case MotorOpcode::Limit:
        return true;
    default:
        return false;
    }
}

typedef struct Op {
    unsigned short port;
    char opcode;
    uint8_t queue;
    // Legacy wire-format field names retained for compatibility across motion opcodes.
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
