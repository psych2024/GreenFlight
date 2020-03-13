#include "Arduino.h"
#include "GreenWifi.h"
#include "GreenIMU.h"
#include "PIDCalculator.h"
#include "DataLogger.h"

//Latest Rate PID
//P 5.1
//I 2.0
//D 0.018

//Latest Angle PID
//P 0.6
//I 0
//D 0.001

/*
 * Quadcopter Motor Labeling
 *
 *         +p(.)
 *
 *         FRONT
 *                  +y (clockwise)
 *      D(7)   C(6)
 *          \ /
 *   +r(.)   +
 *          / \
 *      B(5)   A(4)
 *
 */

//TODO: sending response takes a lot of time

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

void logPIDData();

void setup() {
    Serial.begin(115200);
    DDRD |= B11110000;
    pinMode(13, OUTPUT);

    Wire.begin();
    Wire.setClock(1000000);

    greenWifi.init();
    greenImu.init();
}

void loop() {
    programTimer = micros();

    sendESCPulse();

    logPIDData();

    greenImu.updateYPR();

    batteryVoltage = (analogRead(0) + 65) * 1.2317;

    //though 200hz is 5000us 250us is left as tolerance to prevent mpu6050 buffer overflow
    //By changing the MPU6050_DMP_FIFO_RATE_DIVISOR from 0x00 to 0x01, there were no changes in the sample rate
    //I suspect it is because the dmp is not affected by the rate divisor and implements its own rate
    if (micros() - programTimer > 4900) {
        Serial.println(F("Can't keep up with timer!"));
        armed = false;
        sendESCPulse();
        digitalWrite(LED_BUILTIN, HIGH);

        while (true) {}
    }

    //rest of the time can be used to parse command
    while (micros() - programTimer <= 4900) parseCommand();
}

void parseCommand() {
    char *cmd = greenWifi.fetchCommand();
    if (cmd != nullptr) {
        Serial.print(F("Received Command: "));
        Serial.println(cmd);

        if (strcmp_P(cmd, PSTR("PING")) == 0) {
            greenWifi.sendResponse("PONG");
        } else if (strcmp_P(cmd, PSTR("ARM")) == 0) {
            armed = true;
            pidCalculator.enablePid();
            digitalWrite(LED_BUILTIN, HIGH);
            greenWifi.sendResponse("ARMED");
        } else if (strcmp_P(cmd, PSTR("DISARM")) == 0) {
            armed = false;
            pidCalculator.disablePid();
            digitalWrite(LED_BUILTIN, LOW);
            dataLogger.updateDataLength();
            greenWifi.sendResponse("DISARMED");
        } else if (*cmd == '?') {
            // ?[AR][YPR]
            char *mode = cmd + 1;
            Axis axis = *(cmd + 2) == 'Y' ? YAW : PITCH;

            // P000.000I000.000D000.000
            char *response = new char[24];
            if (*mode == 'A') {
                if (axis == PITCH) {
                    dtostrf(PIDCalculator::pitchAngleKp, 7, 3, response + 1);
                    dtostrf(PIDCalculator::pitchAngleKi, 7, 3, response + 9);
                    dtostrf(PIDCalculator::pitchAngleKd, 7, 3, response + 17);
                } else {
                    dtostrf(PIDCalculator::yawAngleKp, 7, 3, response + 1);
                    dtostrf(PIDCalculator::yawAngleKi, 7, 3, response + 9);
                    dtostrf(PIDCalculator::yawAngleKd, 7, 3, response + 17);
                }
            } else {
                if (axis == PITCH) {
                    dtostrf(PIDCalculator::pitchRateKp, 7, 3, response + 1);
                    dtostrf(PIDCalculator::pitchRateKi, 7, 3, response + 9);
                    dtostrf(PIDCalculator::pitchRateKd, 7, 3, response + 17);
                } else {
                    dtostrf(PIDCalculator::yawRateKp, 7, 3, response + 1);
                    dtostrf(PIDCalculator::yawRateKi, 7, 3, response + 9);
                    dtostrf(PIDCalculator::yawRateKd, 7, 3, response + 17);
                }
            }

            response[0] = 'P';
            response[8] = 'I';
            response[16] = 'D';

            greenWifi.sendResponse(response);
        } else if (*cmd == '!') {
            char *mode = cmd + 1;
            Axis axis = *(cmd + 2) == 'Y' ? YAW : PITCH;

            // !AYP000.000I000.000D000.000
            *(cmd + 11) = '\0';
            *(cmd + 19) = '\0';

            if (*mode == 'A') {
                PIDCalculator::updateAnglePID(axis, atof(cmd + 4), atof(cmd + 12), atof(cmd + 20));
            } else {
                PIDCalculator::updateRatePID(axis, atof(cmd + 4), atof(cmd + 12), atof(cmd + 20));
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
                Serial.println(F("Failed to parse input"));
            }

            throttleInputChannel = constrain(throttleInputChannel, 1000, 2000);
            yawInputChannel = constrain(yawInputChannel, 1000, 2000);
            pitchInputChannel = constrain(pitchInputChannel, 1000, 2000);
            rollInputChannel = constrain(rollInputChannel, 1000, 2000);

        } else if (*cmd == 'V') {
            if (armed)
                return;

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
    pidCalculator.calculate(throttleInputChannel, yawInputChannel, pitchInputChannel,
                            rollInputChannel);

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
        //Serial.println(F("Doing too much in 1000us!"));

        armed = false;
        sendESCPulse();
        digitalWrite(LED_BUILTIN, HIGH);
        //while (true) {}
    }

    while (PORTD >= 16) {
        loopTimer = micros();
        if (timerA <= loopTimer) PORTD &= B11101111;
        if (timerB <= loopTimer) PORTD &= B11011111;
        if (timerC <= loopTimer) PORTD &= B10111111;
        if (timerD <= loopTimer) PORTD &= B01111111;
    }
}

// maximum number of bytes that can be recorded per cycle: 28
float buffer[7];
void logPIDData() {
    if (!armed)
        return;

    buffer[0] = *greenImu.getPitchAngle();
    buffer[2] = pidCalculator.pitchAngleSetpoint;
    buffer[1] = pidCalculator.pitchOutput;
    buffer[3] = pidCalculator.pulseA;
    buffer[4] = pidCalculator.pulseB;
    buffer[5] = pidCalculator.pulseC;
    buffer[6] = pidCalculator.pulseD;

    dataLogger.writePIDData(buffer, 7);
}