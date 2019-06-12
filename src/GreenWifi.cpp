#include "GreenWifi.h"
#include "Arduino.h"
#include "CytronWiFiShield.h"

const byte ACK_CONNECT[4] = {
        //PONG
        0x50, 0x4F, 0x4E, 0x47
};

const byte WARN_OVERFLOW[5] = {
        //OVERF
        0x4F, 0x56, 0x45, 0x52, 0x46
};

const byte ACK_ARMED[5] = {
        //ARMED
        0x41, 0x52, 0x4D, 0x45, 0x44
};

const byte ACK_DISARMED[8] = {
        0x44, 0x49, 0x53, 0x41, 0x52, 0x4D, 0x45, 0x44
};

char commandBuffer[COMMAND_BUFFER_SIZE];
uint8_t marker = 0;
WifiStatus wifiStatus = INIT;
BufferStatus buffStatus = EMPTY;

void GreenWifi::init() {
    Serial.println(F("Initializing wifi module..."));

    if (!wifi.begin(WIFI_MODULE_RX_PIN, WIFI_MODULE_TX_PIN)) {
        Serial.println(F("Error connecting to wifi module!"));
        while (true);
    }

    if (!wifi.setMode(WIFI_AP)) {
        Serial.println(F("Error setting wifi mode!"));
        while (true);
    }

    if (!wifi.echo(false)) {
        Serial.println(F("Error disabling echo mode!"));
        while (true);
    }

    if (!wifi.setMux(false)) {
        Serial.println(F("Error disabling multiple connections!"));
        while (true);
    }

    if (wifi.udpConnect("0", 0, UDP_HOST_PORT, 2)) {
        wifiStatus = WAITING;
    } else {
        Serial.println(F("Error establishing udp connection!"));
        while (true);
    }

    Serial.println(F("Successfully initialized wifi module!"));
}

char *GreenWifi::fetchCommand() {
    if (buffStatus == INCOMPLETE) {
        return readForEndPoint();

    } else if (buffStatus == COMPLETED) {

        //clear previous data from buffer
        memset(commandBuffer, 0, COMMAND_BUFFER_SIZE);
        marker = 0;

        readForStartPoint();
        return readForEndPoint();

    } else if (buffStatus == EMPTY) {
        readForStartPoint();
        return readForEndPoint();
    }

    return nullptr;
}

void GreenWifi::readForStartPoint() {
    while (wifi.available() > 0) {
        char c = wifi.read();

        if (c == ':') {
            buffStatus = INCOMPLETE;
            break;
        }
    }
}

char *GreenWifi::readForEndPoint() {
    while (wifi.available() > 0) {
        char c = wifi.read();

        if (c == '#') {
            buffStatus = COMPLETED;
            return commandBuffer;
        }

        commandBuffer[marker] = c;
        marker++;
    }

    return nullptr;
}

bool GreenWifi::sendResponse(char *buff) {

}
