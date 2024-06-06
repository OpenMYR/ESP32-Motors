#ifndef _WebServer_H_
#define _WebServer_H_
#include <ESPAsyncWebServer.h>

class WebServer {
    public:
        static bool init();
        static void reset();
    private:  
};

#endif /* _WebServer_H_ */