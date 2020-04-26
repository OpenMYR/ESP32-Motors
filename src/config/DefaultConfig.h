#ifndef _DefaultConfig_H_ //Comment me out in LocalConfig
#define _DefailtConfig_H_ //Comment me out in LocalConfig

// #ifndef _LocalConfig_H_ //Uncomment me in LocalConfig
// #define _LocalConfig_H_ //Uncomment me in LocalConfig

#include <WString.h>
#include <String.h>

/*  DO NOT EDIT THIS FILE DIRECTLY, make a duplicate of this file and rename it to "LocalConfig.h".
    After duplicating, remove this comment and replace the ifndef on the top of the LocalConfig file.
*/

/*  LocalConfig has been added to .gitignore to avoid accidental commiting of sensitive info such
    as your personal WiFi ssid and pass. DefaultConfig should only be changed if you intend on 
    pushing your changes upstream.
*/
#define SERVO

#define MYR_WIFI_MODE_AP            000001
#define MYR_WIFI_MODE_STATION       0x0002
#define MYR_WIFI_MODE_AP_STATION    0x0004

#define MYR_WIFI_START_IN_MODE MYR_WIFI_MODE_AP
#define MYR_WIFI_SUPPORTED_MODES MYR_WIFI_MODE_AP | MYR_WIFI_MODE_STATION

#define MYR_WIFI_DEFAULT_AP_SSID_UID_CHAR_COUNT 4

const String MYR_WIFI_DEFAULT_AP_SSID = "OpenMYR-Motor-";
const String MYR_WIFI_DEFAULT_AP_PASS = "";

const String MYR_WIFI_DEFAULT_STATION_SSID = "ssid";    //Do not define these in DefaultConfig.h, see warning above
const String MYR_WIFI_DEFAULT_STATION_PASS = "pass";

#define MYR_WIFI_STATION_FAIL_TO_AP 1;  //If the device fails to connect, should it establish an AP

#define MYR_WIFI_STA_RETRIES 5;

#endif