#include "udp_srv.h"
#include "esp_log.h"
#include "OpBuffer.h"
#include <Arduino.h>
#include <AsyncUDP.h>

udp_srv::udp_srv()
{
    log_e("Const UDP!");
}

void udp_srv::begin()
{
    std::function<void(command_response_packet &)> f = std::bind(&udp_srv::broadcast_ack, this, std::placeholders::_1);
    CommandParser::register_udp_ack_func(f);

    udp.connect(IPAddress(192, 168, 4, 255), 4140);
    udp.listen(4120);
    udp.onPacket([this](AsyncUDPPacket &packet) { this->udp_packet_callback(packet); });
    log_v("begin UDP!");
}

void udp_srv::udp_packet_callback(AsyncUDPPacket &packet)
{
    packet.printf("sup, I got %u bytes of datum", packet.length());
    if (packet.length() == CTRL_PACKET_LEN_BYTES)
    {
        //log_i("valid packet");
        Op motor_packet(packet.data());
        packet.printf("%u,%c,%u,%u,%d,%u",motor_packet.motorID,motor_packet.opcode, motor_packet.port, motor_packet.queue, motor_packet.stepNum, motor_packet.stepRate);
        //log_i("%u,%c,%u,%u,%d,%u\n",motor_packet.motorID,motor_packet.opcode, motor_packet.port, motor_packet.queue, motor_packet.stepNum, motor_packet.stepRate);
        //log_i("%u %u\n",OpBuffer::getInstance()->isFull(0),OpBuffer::getInstance()->isFull(1));
        CommandParser::motor_process_command(motor_packet, packet.remoteIP());
    }
    else if (packet.length() == WIFI_PACKET_LEN_BYTES)
    {
        wifi_command_packet wifiPacket = *(wifi_command_packet *)(packet.data());
        CommandParser::wifi_process_command(wifiPacket, packet.remoteIP());
    }
    packet.~AsyncUDPPacket();
}

void udp_srv::prompt_broadcast()
{
    udp.broadcastTo("HEY EVERYBODY! I'M A MOTOR", 4140);
}

void udp_srv::end()
{
    udp.close();
}

void udp_srv::broadcast_ack(command_response_packet &packet)
{
    udp.broadcastTo((uint8_t *)(&packet), sizeof(command_response_packet), 4140);
}