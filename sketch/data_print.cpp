#include "Wire.h"

#define FRAM_ADDRESS 0x50
#define DATA_LENGTH 7

uint8_t floatSize = sizeof(float);

void readBlock(uint16_t address, uint8_t *buffer, uint8_t size) {
    Wire.beginTransmission(FRAM_ADDRESS);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    Wire.endTransmission();
    Wire.requestFrom(FRAM_ADDRESS, size);
    for (uint8_t i = 0; i < size; i++) {
        *buffer++ = Wire.read();
    }
}

void writeBlock(uint16_t address, uint8_t *buffer, uint8_t size) {
    Wire.beginTransmission(FRAM_ADDRESS);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    for (uint8_t i = 0; i < size; i++) {
        Wire.write(*buffer++);
    }
    Wire.endTransmission();
}

void setup() {
    Serial.begin(115200);

    union pointer_t {
        uint16_t num;
        uint8_t bytes[2];
    };

    pointer_t pointer;
    uint8_t pbuff[2];
    readBlock(0x00, pbuff, 2);
    pointer.bytes[0] = pbuff[0];
    pointer.bytes[1] = pbuff[1];
    Serial.print("Pointer length: ");
    Serial.println(pointer.num);
    Serial.print("Number of recorded data: ");
    Serial.println((pointer.num - 2) / 4);

    union float_t {
        float num;
        uint8_t bytes[4];
    };

    int counter = 0;
    for (int i = 0x02; i < pointer.num; i += floatSize) {
        uint8_t buff[4];
        readBlock(i, buff, floatSize);
        float_t f;
        f.bytes[0] = buff[0];
        f.bytes[1] = buff[1];
        f.bytes[2] = buff[2];
        f.bytes[3] = buff[3];

        Serial.print(f.num);
        Serial.print(" ");
        counter++;
        if (counter == DATA_LENGTH) {
            Serial.println();
            counter = 0;
        }
    }
}

void loop() {

}