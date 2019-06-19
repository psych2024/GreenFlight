#include "Arduino.h"
#include "GreenWifi.h"
#include "GreenIMU.h"
#include "PIDCalculator.h"

/*
 * Quadcopter Motor Labeling
 * A(4)   B(5)
 *    \ /
 *     +
 *    / \
 * C(6)   D(7)
 */

unsigned long programTimer, loopTimer;
unsigned long timerA, timerB, timerC, timerD;

int throttleInputChannel = 1000;
int yawInputChannel = 1500;
int pitchInputChannel = 1500;
int rollInputChannel = 1500;

bool armed = false;

void sendESCPulse();

void parseCommand();

void setup() {
    Serial.begin(115200);
    DDRD |= B11110000;
    pinMode(13, OUTPUT);

    //greenImu.init();
    greenWifi.init();
}

void loop() {
    programTimer = micros();

    //greenImu.updateYPR();

    sendESCPulse();

    if (micros() - programTimer > 5000) {
        Serial.println(F("Can't keep up with timer!"));
    }

    Serial.println(micros() - programTimer);

    //rest of the time can be used to parse command
    while (micros() - programTimer < 5000) parseCommand();
}

void parseCommand() {
    char *cmd = greenWifi.fetchCommand();
    if (cmd != nullptr) {
        Serial.print(F("Received Command: "));
        Serial.println(cmd);

        if (strcmp(cmd, PSTR("PING")) == 0) {
            greenWifi.sendResponse(PSTR("PONG"));
        } else if (strcmp(cmd, PSTR("ARM")) == 0) {
            armed = true;
            greenWifi.sendResponse(PSTR("ARMED"));
        } else if (strcmp(cmd, PSTR("DISARM")) == 0) {
            armed = false;
            greenWifi.sendResponse(PSTR("DISARMED"));
        } else if (*cmd == 'T') {
            throttleInputChannel = atoi(strtok(cmd, PSTR("TYPR")));
            yawInputChannel = atoi(strtok(NULL, PSTR("TYPR")));
            pitchInputChannel = atoi(strtok(NULL, PSTR("TYPR")));
            rollInputChannel = atoi(strtok(NULL, PSTR("TYPR")));

            constrain(throttleInputChannel, 1000, 2000);
            constrain(yawInputChannel, 1000, 2000);
            constrain(pitchInputChannel, 1000, 2000);
            constrain(rollInputChannel, 1000, 2000);

            if (throttleInputChannel == 0 || yawInputChannel == 0 ||
                pitchInputChannel == 0 || rollInputChannel == 0) {
                Serial.println(F("Failed to parse input"));
            }
        }
    }
}

void sendESCPulse() {
    loopTimer = micros();
    PORTD |= B11110000;

    //Do pid calculations with compulsory 1000us free time
    pidCalculator.calculate(throttleInputChannel, yawInputChannel, pitchInputChannel, rollInputChannel);

    if (armed) {
        timerA = loopTimer + pidCalculator.getCalculatedPulseA();
        timerB = loopTimer + pidCalculator.getCalculatedPulseB();
        timerC = loopTimer + pidCalculator.getCalculatedPulseC();
        timerD = loopTimer + pidCalculator.getCalculatedPulseD();
    } else {
        timerA = loopTimer + 1000;
        timerB = loopTimer + 1000;
        timerC = loopTimer + 1000;
        timerD = loopTimer + 1000;
    }

    if (micros() - loopTimer > 1000) {
        Serial.println(F("Doing too much in 1000us!"));
        return;
    }

    while (PORTD >= 16) {
        loopTimer = micros();
        if (timerA <= loopTimer) PORTD &= B11101111;
        if (timerB <= loopTimer) PORTD &= B11011111;
        if (timerC <= loopTimer) PORTD &= B10111111;
        if (timerD <= loopTimer) PORTD &= B01111111;
    }
}