#include "DataLogger.h"

uint16_t pointer = 0x02;
uint8_t floatSize = sizeof(float);

void DataLogger::writePIDData(float *buffer, uint8_t length) {
    for (int i = 0; i < length; i++) {
        writeBlock(pointer, (uint8_t *) &buffer[i], floatSize);
        pointer += floatSize;
    }
}

void DataLogger::updateDataLength() {
    writeBlock(0x00, (uint8_t *) &pointer, 2);
}

void DataLogger::writeBlock(uint16_t address, uint8_t *buffer, uint8_t size) {
    Wire.beginTransmission(FRAM_ADDRESS);
    Wire.write(address >> 8);
    Wire.write(address & 0xFF);
    for (uint8_t i = 0; i < size; i++) {
        Wire.write(*buffer++);
    }
    Wire.endTransmission();
}
