#include "Arduino.h"
#include "GreenWifi.h"
#include "GreenIMU.h"

/*
 * Quadcopter Motor Labeling
 * A(5)   B(4)
 *    \ /
 *     +
 *    / \
 * C(6)   D(7)
 */

unsigned long programTimer, loopTimer, timeHolder;
unsigned long timerA, timerB, timerC, timerD;
int pulseA, pulseB, pulseC, pulseD;

void sendESCPulse();

void setup() {
    Serial.begin(115200);
    DDRD |= B11110000;

    greenImu.init();
    greenWifi.init();
}

void loop() {
    programTimer = micros();

    greenImu.updateYPR();

    char *cmd = greenWifi.fetchCommand();
    if (cmd != nullptr) {
        Serial.print(F("Received Command: "));
        Serial.println(cmd);

        if (atoi(cmd) != 0) {
            int pulse = atoi(cmd);

            constrain(pulse, 1000, 2000);
            pulseA = pulse;
            pulseB = pulse;
            pulseC = pulse;
            pulseD = pulse;
        }
    }

    sendESCPulse();

    if (micros() - programTimer > 5000) {
        Serial.println(F("Can't keep up with timer!"));
    }

    while (micros() - programTimer < 5000);
}

void sendESCPulse() {
    loopTimer = micros();
    PORTD |= B11110000;

    timerA = loopTimer + pulseA;
    timerB = loopTimer + pulseB;
    timerC = loopTimer + pulseC;
    timerD = loopTimer + pulseD;

    while (PORTD >= 16) {
        timeHolder = micros();
        if (timerA < timeHolder) PORTD &= B11011111;
        if (timerB < timeHolder) PORTD &= B11101111;
        if (timerC < timeHolder) PORTD &= B10111111;
        if (timerD < timeHolder) PORTD &= B01111111;
    }
}