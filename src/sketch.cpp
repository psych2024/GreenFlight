#include "Arduino.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

unsigned long programTimer;
MPU6050 mpu;
uint8_t devStatus;
uint8_t fifoBuffer[64];
uint16_t packetSize;
uint16_t fifoCount;

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];

void setup() {
    Serial.begin(115200);
    Fastwire::setup(400, true);

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    if (devStatus != 0) {
        Serial.println("Failed to initialise dmp!");
        while (true);
    }

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    packetSize = mpu.dmpGetFIFOPacketSize();
}

void loop() {
    programTimer = micros();
    fifoCount = mpu.getFIFOCount();

    if (fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));
        return;
    }

    if(fifoCount > packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
    }

    while (micros() - programTimer < 5000);
}
