#ifndef GREENFLIGHT_PIDCALCULATOR_H
#define GREENFLIGHT_PIDCALCULATOR_H

#include "GreenIMU.h"
#include "PID_v1.h"

#define ANGLE_PITCH_KP_EEPROM_ADDRESS 256
#define ANGLE_PITCH_KI_EEPROM_ADDRESS 260
#define ANGLE_PITCH_KD_EEPROM_ADDRESS 264

#define ANGLE_YAW_KP_EEPROM_ADDRESS 268
#define ANGLE_YAW_KI_EEPROM_ADDRESS 272
#define ANGLE_YAW_KD_EEPROM_ADDRESS 276

#define RATE_PITCH_KP_EEPROM_ADDRESS 280
#define RATE_PITCH_KI_EEPROM_ADDRESS 284
#define RATE_PITCH_KD_EEPROM_ADDRESS 288

#define RATE_YAW_KP_EEPROM_ADDRESS 292
#define RATE_YAW_KI_EEPROM_ADDRESS 296
#define RATE_YAW_KD_EEPROM_ADDRESS 300

#define RATE_MODE 0
#define ANGLE_MODE 1

#define SAMPLE_RATE_MILLS 5

enum Axis {
    YAW, PITCH, ROLL
};

class PIDCalculator {
public:
    void calculate(int throttleInputChannel, int yawInputChannel, int pitchInputChannel, int rollInputChannel);
    void enablePid();
    void disablePid();
    int getCalculatedPulseA();
    int getCalculatedPulseB();
    int getCalculatedPulseC();
    int getCalculatedPulseD();

    static void updateRatePID(Axis axis, float kp, float ki, float kd);
    static void updateAnglePID(Axis axis, float kp, float ki, float kd);

    static float yawAngleKp, yawAngleKi, yawAngleKd;
    static float pitchAngleKp, pitchAngleKi, pitchAngleKd;
    static float rollAngleKp, rollAngleKi, rollAngleKd;

    static float yawRateKp, yawRateKi, yawRateKd;
    static float pitchRateKp, pitchRateKi, pitchRateKd;
    static float rollRateKp, rollRateKi, rollRateKd;

    float yawOutput, pitchOutput, rollOutput;
    float yawAngleSetpoint, pitchAngleSetpoint, rollAngleSetpoint;
    float yawRateSetpoint, pitchRateSetpoint, rollRateSetpoint;
    int pulseA, pulseB, pulseC, pulseD;
    PIDCalculator();

    PID yawAnglePid, yawRatePid;
    PID pitchAnglePid, pitchRatePid;
    PID rollAnglePid, rollRatePid;

private:
    void calculatePID();
    void updateMotorPulse();
    void initPIDValues();
};

extern int MODE;
extern PIDCalculator pidCalculator;
#endif
