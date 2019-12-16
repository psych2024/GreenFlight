#ifndef GREENFLIGHT_GREENIMU_H
#define GREENFLIGHT_GREENIMU_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

class GreenIMU {
public:
    void init();
    void updateYPR();
    float * getPitchAngle();
    float * getYawAngle();
    float * getRollAngle();
    float * getPitchRate();
    float * getYawRate();
    float * getRollRate();
};

extern GreenIMU greenImu;

#endif //GREENFLIGHT_GREENIMU_H
