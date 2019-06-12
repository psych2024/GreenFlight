#include "Arduino.h"

int pulse = 1000;
int oldPulse = 1000;
char buffer[5];
int counter = 0;

unsigned long loopTimer;
unsigned long programTimer;

void setup() {
    Serial.begin(115200);

    Serial.println("Write any character to begin calibration");
    while (!Serial.available()); //Wait for input
    while (Serial.available() > 0) Serial.read();  //Clear input buffer

    Serial.println("Input motor pulse length for calibration");
}

void loop() {
    programTimer = micros();

    if (Serial.available()) {
        char c;
        while ((c = Serial.read()) != '\n') {
            buffer[counter] = c;
            counter++;

            if (counter == 4)
                counter = 0;
        }
    }

    buffer[4] = '\0';
    if (atoi(buffer) != 0) {
        oldPulse = pulse;
        pulse = atoi(buffer);
    }

    if (pulse != oldPulse) {
        Serial.print("Received new pulse: ");
        Serial.println(pulse);
    }

    if (pulse > 2000)
        pulse = 2000;

    if (pulse < 1000)
        pulse = 1000;

    loopTimer = micros();
    PORTD |= B11110000;

    while (PORTD >= 16) {
        if (loopTimer + pulse < micros()) PORTD &= B00001111;
    }

    while (micros() - programTimer > 5000);
}