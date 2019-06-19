#include "HID.h"
#include "PIDCalculator.h"
#include "GreenIMU.h"

float pitchKp = 1.0;
float pitchKi = 0;
float pitchKd = 15.0;

//pid configuration should be the same since quadcopter is symmetrical
float rollKp = pitchKp;
float rollKi = pitchKi;
float rollKd = pitchKd;

float yawKp = 0;
float yawKi = 0;
float yawKd = 0;

float previousPitchError;
float previousRollError;
float previousYawError;

float pitchIntegral;
float rollIntegral;
float yawIntegral;

float pitchSetpoint = 0;
float rollSetpoint = 0;
float yawSetpoint = 0;

float maxPitch = 200;
float maxRoll = 200;
float maxYaw = 200;

float errorTemp;

int throttle;
float calculatedYaw, calculatedPitch, calculatedRoll;

int pulseA, pulseB, pulseC, pulseD;

void PIDCalculator::calculate(int throttleInputChannel, int yawInputChannel, int pitchInputChannel, int rollInputChannel) {
    //lower throttle value by 25% to allow room for pid control
    throttle = map(throttleInputChannel, 1000, 2000, 1000, 1750);
    yawSetpoint = map(yawInputChannel, 1000, 2000, -25, 25);
    pitchSetpoint = map(pitchInputChannel, 1000, 2000, -25, 25);
    rollSetpoint = map(rollInputChannel, 1000, 2000, -25, 25);

    //limit setpoint to zero around middle of joystick +-20
    if(yawInputChannel < 1510 && yawInputChannel > 1490) yawSetpoint = 0;
    if(pitchInputChannel < 1510 && pitchInputChannel > 1490) pitchSetpoint = 0;
    if(rollInputChannel < 1510 && rollInputChannel > 1490) rollSetpoint = 0;

    calculateYawPID();

    calculatePitchPID();

    calculateRollPID();

    updateMotorPulse();
}

void PIDCalculator::calculateYawPID() {
    //calculate yaw pid
    errorTemp = yawSetpoint - greenImu.getYaw();
    yawIntegral += errorTemp;
    calculatedYaw = errorTemp * yawKp + yawIntegral * yawKi + yawKd * (errorTemp - previousYawError);
    previousYawError = errorTemp;

    //restrict yaw to maximum values
    constrain(calculatedYaw, maxYaw * -1, maxYaw);
}

void PIDCalculator::calculatePitchPID() {
    //calculate pitch pid
    errorTemp = pitchSetpoint - greenImu.getPitch();
    pitchIntegral += errorTemp;
    calculatedPitch = errorTemp * pitchKp + pitchIntegral * pitchKi + pitchKd * (errorTemp - previousPitchError);
    previousPitchError = errorTemp;

    //restrict pitch to maximum values
    constrain(calculatedPitch, maxPitch * -1, maxPitch);
}

void PIDCalculator::calculateRollPID() {
    //calculate roll pid
    errorTemp = rollSetpoint - greenImu.getRoll();
    rollIntegral += errorTemp;
    calculatedRoll = errorTemp * rollKp + rollIntegral * rollKi + rollKd * (errorTemp - previousRollError);
    previousRollError = errorTemp;

    //restrict roll to maximum values
    constrain(calculatedRoll, maxRoll * -1, maxRoll);
}

void PIDCalculator::updateMotorPulse() {
    //update individual motor throttle values
    pulseA = throttle + calculatedPitch + calculatedRoll - calculatedYaw;  //front left motor
    pulseB = throttle + calculatedPitch - calculatedRoll + calculatedYaw;  //front right motor
    pulseC = throttle - calculatedPitch + calculatedRoll + calculatedYaw;  //rear left motor
    pulseD = throttle - calculatedPitch - calculatedRoll - calculatedYaw;  //rear right motor

    constrain(pulseA, 1100, 2000);
    constrain(pulseB, 1100, 2000);
    constrain(pulseC, 1100, 2000);
    constrain(pulseD, 1100, 2000);
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

