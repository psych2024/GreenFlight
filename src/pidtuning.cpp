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

float yaw;
float pitch;
float roll;

const float setpoint = 0;
float error;
float kp;
float ki;
float kd;
float integral;
float previousError;
float calculatedRoll;

int leftMotorPulse;
int rightMotorPulse;

unsigned long loopTimer;
unsigned long leftChannelTimer;
unsigned long rightChannelTimer;

//Left motor DIGITAL 4
//Right motor DIGITAL 5

void setup() {
    Serial.begin(115200);
    DDRD |= B00110000;

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

    pinMode(8, OUTPUT);
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
        yaw = ypr[0] * 180/M_PI;
        pitch = ypr[1] * 180/M_PI;
        roll = ypr[2] * 180/M_PI;
    }

    error = setpoint - roll;
    integral += error;
    calculatedRoll = error * kp + integral * ki + kd * (error - previousError);
    previousError = error;

    if(calculatedRoll > 200)
        calculatedRoll = 200;

    if(calculatedRoll < -200)
        calculatedRoll = -200;

    leftMotorPulse = 1500 + calculatedRoll;
    rightMotorPulse = 1500 - calculatedRoll;

    loopTimer = micros();
    PORTD |= B00110000;

    leftChannelTimer = loopTimer + leftMotorPulse;
    rightChannelTimer = loopTimer + rightMotorPulse;

    while(PORTD >= 16) {
        loopTimer = micros();
        if(leftChannelTimer <= loopTimer) PORTD &= B11101111;
        if(rightChannelTimer <= loopTimer) PORTD &= B11011111;
    }

    if(micros() - programTimer > 5000)
        digitalWrite(8, HIGH);

    while (micros() - programTimer < 5000);
}
