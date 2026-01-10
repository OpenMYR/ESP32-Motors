#ifndef MYR_WEBSERVER_H
#define MYR_WEBSERVER_H

#include <ESPAsyncWebServer.h>

class WebServer {
    public:
        static bool init();
        static void reset();
    private:  
};

#endif // MYR_WEBSERVER_H