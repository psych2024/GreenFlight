#include "GreenWifi.h"
#include "Arduino.h"
#include "CytronWiFiShield.h"

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
            return;
        }
    }
}

char *GreenWifi::readForEndPoint() {
    while (wifi.available() > 0) {
        char c = wifi.read();

        if (c == '#') {
            buffStatus = COMPLETED;

            //Add \0 to end string
            commandBuffer[marker] = '\0';
            marker++;

            return commandBuffer;
        }

        commandBuffer[marker] = c;
        marker++;
    }

    return nullptr;
}

void GreenWifi::sendResponse(const char *buff) {
    wifi.udpSend((const uint8_t *) buff, strlen(buff));
}
