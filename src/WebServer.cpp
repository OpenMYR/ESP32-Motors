#if __has_include("config/LocalConfig.h")
#include "config/LocalConfig.h"
#else
#include "config/DefaultConfig.h"
#endif

#include "WebServer.h"
#include <SPIFFS.h>
#include "CommandParser.h"

#include "esp_log.h"
String const TAG = "WebServer";

AsyncWebServer server(80);


#if SERVO==1
#define MOTOR_TYPE 0
#elif STEPPER==1
#define MOTOR_TYPE 1
#endif


void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

const char* PARAM_MESSAGE = "message";

bool WebServer::init() {
    log_i("Starting Webserver");
    
    server.serveStatic("/", SPIFFS, MOTOR_TYPE == 0 ? "/web_srv/" : "/web_step/");
    server.serveStatic("/", SPIFFS,  MOTOR_TYPE == 0 ? "/web_srv/" : "/web_step/").setDefaultFile("index.html");

    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request){
        bool return202Required = true;
        log_v("POST DETECTED: %d Parameters", request->params());
        for(unsigned int i = 0; i < request->params(); i++)
        {
            return202Required = return202Required && CommandParser::json_parseCommands(request->getParam(i)->value(), request->client()->remoteIP());
        }
        if (return202Required) request->send(202);
    });

    server.onNotFound(notFound);

    server.begin();
    return true;
}

void WebServer::reset() {
    server.reset();
}
