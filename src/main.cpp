#include "Arduino.h"
#include "GreenWifi.h"
#include "GreenIMU.h"
#include "PIDCalculator.h"
#include "Debug.h"

/*
 * Quadcopter Motor Labeling
 * A(4)   B(5)
 *    \ /
 *     +
 *    / \
 * C(6)   DEBUGL(7)
 */

//TODO: sending response takes a lot of time
//TODO: Implement PID Library
//TODO: Define DEBUG Preprocessor

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
#ifdef GREENFLIGHT_DEBUG_H
    Serial.begin(115200);
#endif
    DDRD |= B11110000;
    pinMode(13, OUTPUT);

    greenWifi.init();
    greenImu.init();
    PIDCalculator::initPIDValues();
}

void loop() {
    programTimer = micros();

    sendESCPulse();

    greenImu.updateYPR();

    batteryVoltage = (analogRead(0) + 65) * 1.2317;

    //though 200hz is 5000us 250us is left as tolerance to prevent mpu6050 buffer overflow
    //By changing the MPU6050_DMP_FIFO_RATE_DIVISOR from 0x00 to 0x01, there were no changes in the sample rate
    //I suspect it is because the dmp is not affected by the rate divisor and implements its own rate
    if (micros() - programTimer > 4750) {
        DEBUGL(F("Can't keep up with timer!"));
    }

    //rest of the time can be used to parse command
    while (micros() - programTimer <= 4750) parseCommand();
}

void parseCommand() {
    char *cmd = greenWifi.fetchCommand();
    if (cmd != nullptr) {
        DEBUG(F("Received Command: "));
        DEBUGL(cmd);

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
        } else if (*cmd == '?') {
            // ?[YPR]
            char *axis = cmd + 1;

            // P000.000I000.000D000.000
            char *response = new char[24];
            if (*axis == 'P' || *axis == 'R') {
                dtostrf(pitchKp, 7, 3, response + 1);
                dtostrf(pitchKi, 7, 3, response + 9);
                dtostrf(pitchKd, 7, 3, response + 17);
            } else {
                dtostrf(yawKp, 7, 3, response + 1);
                dtostrf(yawKi, 7, 3, response + 9);
                dtostrf(yawKd, 7, 3, response + 17);
            }

            response[0] = 'P';
            response[8] = 'I';
            response[16] = 'D';

            greenWifi.sendResponse(response);
        } else if (*cmd == '!') {
            char *axis = cmd + 1;

            // !YP000.000I000.000D000.000
            *(cmd + 10) = '\0';
            *(cmd + 18) = '\0';

            if (*axis == 'P' || *axis == 'R') {
                PIDCalculator::updatePitchKp(atof(cmd + 3));
                PIDCalculator::updatePitchKi(atof(cmd + 11));
                PIDCalculator::updatePitchKd(atof(cmd + 19));
            } else {
                PIDCalculator::updateYawKp(atof(cmd + 3));
                PIDCalculator::updateYawKi(atof(cmd + 11));
                PIDCalculator::updateYawKd(atof(cmd + 19));
            }

            greenWifi.sendResponse("OK");

            while (true) {
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

                delay(500);
            }
        } else if (*cmd == 'T') {
            throttleInputChannel = atoi(strtok(cmd, "TYPR"));
            yawInputChannel = atoi(strtok(NULL, "TYPR"));
            pitchInputChannel = atoi(strtok(NULL, "TYPR"));
            rollInputChannel = atoi(strtok(NULL, "TYPR"));

            if (throttleInputChannel == 0 || yawInputChannel == 0 ||
                pitchInputChannel == 0 || rollInputChannel == 0) {
                DEBUGL(F("Failed to parse input"));
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
        DEBUGL(F("Doing too much in 1000us!"));
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