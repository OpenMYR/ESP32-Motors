#if __has_include("config/LocalConfig.h")
#include "config/LocalConfig.h"
#else
#include "config/DefaultConfig.h"
#endif

#include <Arduino.h>
#include <ETH.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiSTA.h>
#include <WiFiType.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include "FileIO.h"
#include "WifiController.h"
#include "WebServer.h"
#include "udp_srv.h"
#include "ServoDriver.h"
#include "CommandLayer.h"
#include "OpBuffer.h"
#include <esp_log.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

#if (SERVO + STEPPER + BDC) > 1
#error "Too many device types enabled"
#endif

#if SERVO==1
#define STATUSLEDPIN 13
#elif STEPPER==1
#define STATUSLEDPIN 17
#endif

String const TAG = "Main";
const char *host = "openMYR-esp32";

udp_srv *UDP_server;

void setup()
{
    esp_err_t err;
    pinMode(STATUSLEDPIN, OUTPUT);
    Serial.begin(115200);
    digitalWrite(STATUSLEDPIN, HIGH);
    delay(200);
    err = WifiController::init();
    if (err) log_i("Error: %s, #%u", esp_err_to_name(err), err);
    WebServer::init();
    delay(100);
    FileIO::init();
    delay(100);
    OpBuffer::getInstance();
    CommandLayer::getInstance()->init();
    UDP_server = new udp_srv();
    UDP_server->begin();

    /*use mdns for host name resolution*/
    if (!MDNS.begin(host))
    { 
        log_e("Error setting up MDNS responder!");
        while (1)
        {
            delay(1000);
        }
    }
    log_d("mDNS responder started");

    ArduinoOTA
        .onStart([]() {
            String type;
            if (UDP_server)
                UDP_server->end();
            WebServer::reset();
            CommandParser::stop_motors();

            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else // U_SPIFFS
                type = "filesystem";

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            log_d("Start updating %s", type.c_str());
        })
        .onEnd([]() {
            log_d("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            log_d("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            log_e("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
                log_e("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)
                log_e("Begin Failed");
            else if (error == OTA_CONNECT_ERROR)
                log_e("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR)
                log_e("Receive Failed");
            else if (error == OTA_END_ERROR)
                log_e("End Failed");
        });
    ArduinoOTA.setHostname(host);
    ArduinoOTA.begin();

    digitalWrite(STATUSLEDPIN, LOW);
}

void loop()
{
    delay(1000);
    ArduinoOTA.handle();
    UDP_server->prompt_broadcast();
    //log_i("OpBuffer Commands space in queue 1: %d", OpBuffer::getInstance()->opBufferCapacity(1));
}
