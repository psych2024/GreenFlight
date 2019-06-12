#include "GreenIMU.h"

Quaternion quaternion;
VectorFloat gravity;
uint8_t fifoBuffer[64];
uint16_t fifoCount;
uint16_t packetSize;

float ypr[3];
float yaw;
float pitch;
float roll;

MPU6050 mpu;

void GreenIMU::init() {
    Serial.println(F("Initializing imu module..."));

    Fastwire::setup(400, true);

    Serial.println(F("Initializing MPU6050..."));
    mpu.initialize();
    uint8_t status = mpu.dmpInitialize();

    if (status != 0) {
        Serial.println("Failed to initialise dmp!");
        while (true);
    }

    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();

    Serial.println(F("Successfully initialized imu module!"));
}

void GreenIMU::updateYPR() {
    fifoCount = mpu.getFIFOCount();

    if (fifoCount >= 1024) {
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO buffer overflow!"));
        return;
    }

    if (fifoCount > packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&quaternion, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &quaternion);
        mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);

        yaw = ypr[0] * 180/M_PI;
        pitch = ypr[1] * 180/M_PI;
        roll = ypr[2] * 180/M_PI;
    }
}

float GreenIMU::getPitch() {
    return pitch;
}

float GreenIMU::getYaw() {
    return yaw;
}

float GreenIMU::getRoll() {
    return roll;
}