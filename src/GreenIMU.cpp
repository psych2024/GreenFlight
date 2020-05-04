#include "GreenIMU.h"

GreenIMU greenImu;

Quaternion quaternion;
VectorFloat gravity;
uint8_t fifoBuffer[64];
uint16_t fifoCount;
uint16_t packetSize;

float ypr[3];
float yawAngle;
float pitchAngle;
float rollAngle;

VectorInt16 gyro;
float yawRate;
float pitchRate;
float rollRate;

MPU6050 mpu;

void GreenIMU::init() {
    Serial.println(F("Initializing imu module..."));

    Serial.println(F("Initializing MPU6050..."));
    mpu.initialize();
    uint8_t status = mpu.dmpInitialize();

    if (status != 0) {
        Serial.println(F("Failed to initialise dmp!"));
        while (true);
    }

    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();

    mpu.setXAccelOffset(-2120);
    mpu.setYAccelOffset(-127);
    mpu.setZAccelOffset(1620);
    mpu.setXGyroOffset(85);
    mpu.setYGyroOffset(206);
    mpu.setZGyroOffset(16);

    Serial.println(F("Successfully initialized imu module!"));
}

void GreenIMU::updateYPR() {
    fifoCount = mpu.getFIFOCount();
    //Serial.println(fifoCount);

    if (fifoCount >= 1024) {
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO buffer overflow!"));
        return;
    }

    if (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &quaternion);
        mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
        mpu.dmpGetGyro(&gyro, fifoBuffer);

        yawRate = -gyro.z;
        pitchRate = -gyro.y;
        rollRate = gyro.x;

//        Serial.print(yawRate);
//        Serial.print("    ");
//        Serial.print(pitchRate);
//        Serial.print("    ");
//        Serial.println(rollRate);

        yawAngle = ypr[0] * RAD_TO_DEG;
        pitchAngle = ypr[1] * RAD_TO_DEG;
        rollAngle = ypr[2] * RAD_TO_DEG;

//        Serial.print(yawAngle);
//        Serial.print("    ");
//        Serial.print(pitchAngle);
//        Serial.print("    ");
//        Serial.println(rollAngle);
    } else {
        Serial.println(F("No fifo"));
    }
}

float* GreenIMU::getPitchAngle() {
    return &pitchAngle;
}

float* GreenIMU::getYawAngle() {
    return &yawAngle;
}

float* GreenIMU::getRollAngle() {
    return &rollAngle;
}

float* GreenIMU::getPitchRate() {
    return &pitchRate;
}

float* GreenIMU::getYawRate() {
    return &yawRate;
}

float* GreenIMU::getRollRate() {
    return &rollRate;
}
