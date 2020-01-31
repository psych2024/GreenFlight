#include "HID.h"
#include "PIDCalculator.h"
#include "EEPROM.h"

PIDCalculator pidCalculator;

float PIDCalculator::yawAngleKp;
float PIDCalculator::yawAngleKi;
float PIDCalculator::yawAngleKd;

float PIDCalculator::pitchAngleKp;
float PIDCalculator::pitchAngleKi;
float PIDCalculator::pitchAngleKd;

float PIDCalculator::rollAngleKp;
float PIDCalculator::rollAngleKi;
float PIDCalculator::rollAngleKd;

float PIDCalculator::yawRateKp;
float PIDCalculator::yawRateKi;
float PIDCalculator::yawRateKd;

float PIDCalculator::pitchRateKp;
float PIDCalculator::pitchRateKi;
float PIDCalculator::pitchRateKd;

float PIDCalculator::rollRateKp;
float PIDCalculator::rollRateKi;
float PIDCalculator::rollRateKd;

float throttle;

float maxYaw = 250;
float maxPitch = 200;
float maxRoll = 200;

int pulseA, pulseB, pulseC, pulseD;

int MODE = ANGLE_MODE;

PIDCalculator::PIDCalculator() :
yawAnglePid(greenImu.getYawAngle(), &yawRateSetpoint, &yawAngleSetpoint, 0, 0, 0, DIRECT, SAMPLE_RATE_MILLS),
yawRatePid(greenImu.getYawRate(), &yawOutput, &yawRateSetpoint, 0, 0, 0, DIRECT, SAMPLE_RATE_MILLS),

pitchAnglePid(greenImu.getPitchAngle(), &pitchRateSetpoint, &pitchAngleSetpoint, 0, 0, 0, DIRECT, SAMPLE_RATE_MILLS),
pitchRatePid(greenImu.getPitchRate(), &pitchOutput, &pitchRateSetpoint, 0, 0, 0, DIRECT, SAMPLE_RATE_MILLS),

rollAnglePid(greenImu.getRollAngle(), &rollRateSetpoint, &rollAngleSetpoint, 0, 0, 0, DIRECT, SAMPLE_RATE_MILLS),
rollRatePid(greenImu.getRollRate(), &rollOutput, &rollRateSetpoint, 0, 0, 0, DIRECT, SAMPLE_RATE_MILLS)
{
    initPIDValues();
    yawRatePid.SetOutputLimits(-maxYaw, maxYaw);
    yawAnglePid.SetOutputLimits(-maxYaw, maxYaw);
    yawRatePid.SetTunings(yawRateKp, yawRateKi, yawRateKd);
    yawAnglePid.SetTunings(yawAngleKp, yawAngleKi, yawAngleKd);

    pitchRatePid.SetOutputLimits(-maxPitch, maxPitch);
    pitchAnglePid.SetOutputLimits(-maxPitch, maxPitch);
    pitchRatePid.SetTunings(pitchRateKp, pitchRateKi, pitchRateKd);
    pitchAnglePid.SetTunings(pitchAngleKp, pitchAngleKi, pitchAngleKd);

    rollRatePid.SetOutputLimits(-maxRoll, maxRoll);
    rollAnglePid.SetOutputLimits(-maxRoll, maxRoll);
    rollRatePid.SetTunings(rollRateKp, rollRateKi, rollRateKd);
    rollAnglePid.SetTunings(rollAngleKp, rollAngleKi, rollAngleKd);
}

void PIDCalculator::initPIDValues() {
    //angle pid
    EEPROM.get(ANGLE_PITCH_KP_EEPROM_ADDRESS, pitchAngleKp);
    EEPROM.get(ANGLE_PITCH_KI_EEPROM_ADDRESS, pitchAngleKi);
    EEPROM.get(ANGLE_PITCH_KD_EEPROM_ADDRESS, pitchAngleKd);

    rollAngleKp = pitchAngleKp;
    rollAngleKi = pitchAngleKi;
    rollAngleKd = pitchAngleKd;

    EEPROM.get(ANGLE_YAW_KP_EEPROM_ADDRESS, yawAngleKp);
    EEPROM.get(ANGLE_YAW_KI_EEPROM_ADDRESS, yawAngleKi);
    EEPROM.get(ANGLE_YAW_KD_EEPROM_ADDRESS, yawAngleKd);

    //rate pid
    EEPROM.get(RATE_PITCH_KP_EEPROM_ADDRESS, pitchRateKp);
    EEPROM.get(RATE_PITCH_KI_EEPROM_ADDRESS, pitchRateKi);
    EEPROM.get(RATE_PITCH_KD_EEPROM_ADDRESS, pitchRateKd);

    rollRateKp = pitchRateKp;
    rollRateKi = pitchRateKi;
    rollRateKd = pitchRateKd;

    EEPROM.get(RATE_YAW_KP_EEPROM_ADDRESS, yawRateKp);
    EEPROM.get(RATE_YAW_KI_EEPROM_ADDRESS, yawRateKi);
    EEPROM.get(RATE_YAW_KD_EEPROM_ADDRESS, yawRateKd);
}


void PIDCalculator::calculate(int throttleInputChannel, int yawInputChannel, int pitchInputChannel,
                              int rollInputChannel) {
    //lower throttle value by ~25% to allow room for pid control
    throttle = map(throttleInputChannel, 1000, 2000, 1000, 1800);

    if (MODE == RATE_MODE) {
        yawRateSetpoint = map(yawInputChannel, 1000, 2000, -350, 350);
        pitchRateSetpoint = map(pitchInputChannel, 1000, 2000, -250, 250);
        rollRateSetpoint = map(rollInputChannel, 1000, 2000, -250, 250);
    } else {
        yawAngleSetpoint = map(yawInputChannel, 1000, 2000, -45, 45);
        pitchAngleSetpoint = map(pitchInputChannel, 1000, 2000, -45, 45);
        rollAngleSetpoint = map(rollInputChannel, 1000, 2000, -45, 45);

        //limit setpoint to zero around middle of joystick +-20
        if (yawInputChannel < 1510 && yawInputChannel > 1490) yawAngleSetpoint = 0;
        if (pitchInputChannel < 1510 && pitchInputChannel > 1490) pitchAngleSetpoint = 0;
        if (rollInputChannel < 1510 && rollInputChannel > 1490) rollAngleSetpoint = 0;
    }

    calculatePID();
    updateMotorPulse();
}

void PIDCalculator::calculatePID() {
    if(MODE == ANGLE_MODE) {
        //yawAnglePid.Compute();
        pitchAnglePid.Compute();
        rollAnglePid.Compute();
    }
    yawRatePid.Compute();
    pitchRatePid.Compute();
    rollRatePid.Compute();
}

void PIDCalculator::updateMotorPulse() {
    //update individual motor throttle values
    pulseA = throttle - pitchOutput - rollOutput + yawOutput;  //front left motor
    pulseB = throttle - pitchOutput + rollOutput - yawOutput;  //front right motor
    pulseC = throttle + pitchOutput - rollOutput - yawOutput;  //rear left motor
    pulseD = throttle + pitchOutput + rollOutput + yawOutput;  //rear right motor

    //constrain to minimum 1100us to keep motors running
    pulseA = constrain(pulseA, 1100, 2000);
    pulseB = constrain(pulseB, 1100, 2000);
    pulseC = constrain(pulseC, 1100, 2000);
    pulseD = constrain(pulseD, 1100, 2000);
}

int PIDCalculator::getCalculatedPulseA() {
    return pulseA;
}

int PIDCalculator::getCalculatedPulseB() {
    return pulseB;
}

int PIDCalculator::getCalculatedPulseC() {
    return pulseC;
}

int PIDCalculator::getCalculatedPulseD() {
    return pulseD;
}

void PIDCalculator::updateRatePID(Axis axis, float kp, float ki, float kd) {
    if(axis == YAW) {
        EEPROM.put(RATE_YAW_KP_EEPROM_ADDRESS, kp);
        EEPROM.put(RATE_YAW_KI_EEPROM_ADDRESS, ki);
        EEPROM.put(RATE_YAW_KD_EEPROM_ADDRESS, kd);
    } else if (axis == PITCH || axis == ROLL) {
        EEPROM.put(RATE_PITCH_KP_EEPROM_ADDRESS, kp);
        EEPROM.put(RATE_PITCH_KI_EEPROM_ADDRESS, ki);
        EEPROM.put(RATE_PITCH_KD_EEPROM_ADDRESS, kd);
    }
}

void PIDCalculator::updateAnglePID(Axis axis, float kp, float ki, float kd) {
    if(axis == YAW) {
        EEPROM.put(ANGLE_YAW_KP_EEPROM_ADDRESS, kp);
        EEPROM.put(ANGLE_YAW_KI_EEPROM_ADDRESS, ki);
        EEPROM.put(ANGLE_YAW_KD_EEPROM_ADDRESS, kd);
    } else if (axis == PITCH || axis == ROLL) {
        EEPROM.put(ANGLE_PITCH_KP_EEPROM_ADDRESS, kp);
        EEPROM.put(ANGLE_PITCH_KI_EEPROM_ADDRESS, ki);
        EEPROM.put(ANGLE_PITCH_KD_EEPROM_ADDRESS, kd);
    }
}

void PIDCalculator::enablePid() {
    yawRatePid.SetMode(AUTOMATIC);
    yawAnglePid.SetMode(AUTOMATIC);
    pitchRatePid.SetMode(AUTOMATIC);
    pitchAnglePid.SetMode(AUTOMATIC);
    rollRatePid.SetMode(AUTOMATIC);
    rollAnglePid.SetMode(AUTOMATIC);
}

void PIDCalculator::disablePid() {
    yawRatePid.SetMode(MANUAL);
    yawAnglePid.SetMode(MANUAL);
    pitchRatePid.SetMode(MANUAL);
    pitchAnglePid.SetMode(MANUAL);
    rollRatePid.SetMode(MANUAL);
    rollAnglePid.SetMode(MANUAL);
}
