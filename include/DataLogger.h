#ifndef GREENFLIGHT_DATALOGGER_H
#define GREENFLIGHT_DATALOGGER_H

#include "Wire.h"

#define FRAM_ADDRESS 0x50

class DataLogger {
public:
    void updateDataLength();

    void writePIDData(float *buffer, uint8_t length);

private:
    void writeBlock(uint16_t address, uint8_t *buffer, uint8_t size);

    void readBlock(uint16_t address, uint8_t *buffer, uint8_t size);

};

extern DataLogger dataLogger;
#endif //GREENFLIGHT_DATALOGGER_H
