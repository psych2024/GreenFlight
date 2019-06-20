#ifndef GREENFLIGHT_GREENWIFI_H
#define GREENFLIGHT_GREENWIFI_H

#define WIFI_MODULE_RX_PIN 8
#define WIFI_MODULE_TX_PIN 9
#define COMMAND_BUFFER_SIZE 64
#define UDP_HOST_PORT 333

typedef enum {
    INIT, WAITING, CONNECTED, DISCONNECTED
} WifiStatus;

enum BufferStatus {
    COMPLETED, INCOMPLETE, EMPTY
};

class GreenWifi {
public:
    void init();
    char *fetchCommand();
    void sendResponse(const char* buff);

private:
    void readForStartPoint();
    char *readForEndPoint();
};

extern GreenWifi greenWifi;
#endif
