
#include "FileIO.h"
#include <FS.h>
#include <SPIFFS.h>
#include <esp_log.h>
#include <ArduinoOTA.h>

String const TAG = "FileIO";

bool FileIO::init()
{
    log_i("Mounting SPIFFS");
    if (!SPIFFS.begin())
    {
        // Serious problem
        log_e("SPIFFS Mount Failed");
        ArduinoOTA.setPasswordHash("acc3a26f06f0d71c9d06380b14139aaa");
        return false;
    }
    else
    {
        log_i("SPIFFS Mount successful");
        File f = SPIFFS.open("/ota_pass.txt", "r");
        if (f)
            ArduinoOTA.setPasswordHash(f.readString().c_str());
        else
            ArduinoOTA.setPasswordHash("acc3a26f06f0d71c9d06380b14139aaa");
        f.close();
        return true;
    }
}
