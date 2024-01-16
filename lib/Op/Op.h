#ifndef Op_H_
#define Op_H_

#include <IPAddress.h>

typedef struct Op {
    unsigned short port;
    char opcode;
    uint8_t queue;
    int32_t stepNum;
    uint16_t stepRate;
    uint8_t motorID;
	IPAddress sourceIPAddr;

    Op()
    {

    }

    Op(uint8_t* data)
    {
        port = (data[0] << 8) | data[1];
        opcode = data[2];
        queue = data[3];
        stepNum = (((data[4] << 8) | data[5]) << 16) | ((data[6] << 8) | data[7]);
        stepRate = (data[8] << 8) | data[9];
        motorID = data[10]; 
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
    }
 } Op;

struct command_response_packet {
	char opcode;
    int motor_id;
	int position;
};

struct wifi_command_packet {
	char opcode;
	char ssid[32] ;
	char password[63];
};

#endif /* Op_H_ */