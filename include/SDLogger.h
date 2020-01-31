#ifndef GREENFLIGHT_SDLOGGER_H
#define GREENFLIGHT_SDLOGGER_H

#include "Arduino.h"

struct data_t {
    unsigned long time;
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
};
void acquireData(data_t* data);
void printData(Print* pr, data_t* data);
void printHeader(Print* pr);
#endif //GREENFLIGHT_SDLOGGER_H
