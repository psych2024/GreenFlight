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

int batteryVoltage = 0;

void sendESCPulse();

void parseCommand();

void setup() {
    Serial.begin(115200);
    DDRD |= B11110000;
    pinMode(13, OUTPUT);

    greenWifi.init();
    greenImu.init();
}

void loop() {
    programTimer = micros();

    sendESCPulse();

    greenImu.updateYPR();

    batteryVoltage = (analogRead(0) + 65) * 1.2317;

    //though 200hz is 5000us 250us is left as tolerance to prevent mpu6050 buffer overflow
    if (micros() - programTimer > 4750) {
        Serial.println(F("Can't keep up with timer!"));
    }

    //rest of the time can be used to parse command
    while (micros() - programTimer < 4750) parseCommand();
}

void parseCommand() {
    char *cmd = greenWifi.fetchCommand();
    if (cmd != nullptr) {
        Serial.print(F("Received Command: "));
        Serial.println(cmd);

        if (strcmp(cmd, "PING") == 0) {
            greenWifi.sendResponse("PONG");
        } else if (strcmp(cmd, "ARM") == 0) {
            armed = true;
            digitalWrite(LED_BUILTIN, HIGH);
            greenWifi.sendResponse("ARMED");
        } else if (strcmp(cmd, "DISARM") == 0) {
            armed = false;
            digitalWrite(LED_BUILTIN, LOW);
            greenWifi.sendResponse("DISARMED");
        } else if (*cmd == 'T') {
            throttleInputChannel = atoi(strtok(cmd, "TYPR"));
            yawInputChannel = atoi(strtok(NULL, "TYPR"));
            pitchInputChannel = atoi(strtok(NULL, "TYPR"));
            rollInputChannel = atoi(strtok(NULL, "TYPR"));

            if (throttleInputChannel == 0 || yawInputChannel == 0 ||
                pitchInputChannel == 0 || rollInputChannel == 0) {
                Serial.println(F("Failed to parse input"));
            }

            throttleInputChannel = constrain(throttleInputChannel, 1000, 2000);
            yawInputChannel = constrain(yawInputChannel, 1000, 2000);
            pitchInputChannel = constrain(pitchInputChannel, 1000, 2000);
            rollInputChannel = constrain(rollInputChannel, 1000, 2000);

        } else if (*cmd == 'V') {
            char buff[5];
            sprintf(buff, "V%d", batteryVoltage);
            greenWifi.sendResponse(buff);
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