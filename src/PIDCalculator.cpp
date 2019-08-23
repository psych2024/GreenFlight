#include "HID.h"
#include "PIDCalculator.h"
#include "GreenIMU.h"
#include "EEPROM.h"

const float pitchKp = EEPROM.read(PITCH_KP_EEPROM_ADDRESS);
const float pitchKi = EEPROM.read(PITCH_KI_EEPROM_ADDRESS);
const float pitchKd = EEPROM.read(PITCH_KD_EEPROM_ADDRESS);

//pid configuration should be the same since quadcopter is symmetrical
const float rollKp = pitchKp;
const float rollKi = pitchKi;
const float rollKd = pitchKd;

const float yawKp = EEPROM.read(YAW_KP_EEPROM_ADDRESS);
const float yawKi = EEPROM.read(YAW_KI_EEPROM_ADDRESS);
const float yawKd = EEPROM.read(YAW_KD_EEPROM_ADDRESS);

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
    yawSetpoint = map(yawInputChannel, 1000, 2000, -30, 30);
    pitchSetpoint = map(pitchInputChannel, 1000, 2000, -30, 30);
    rollSetpoint = map(rollInputChannel, 1000, 2000, -30, 30);

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
    calculatedYaw = constrain(calculatedYaw, maxYaw * -1, maxYaw);
}

void PIDCalculator::calculatePitchPID() {
    //calculate pitch pid
    errorTemp = pitchSetpoint - greenImu.getPitch();
    pitchIntegral += errorTemp;
    calculatedPitch = errorTemp * pitchKp + pitchIntegral * pitchKi + pitchKd * (errorTemp - previousPitchError);
    previousPitchError = errorTemp;

    //restrict pitch to maximum values
    calculatedPitch = constrain(calculatedPitch, maxPitch * -1, maxPitch);
}

void PIDCalculator::calculateRollPID() {
    //calculate roll pid
    errorTemp = rollSetpoint - greenImu.getRoll();
    rollIntegral += errorTemp;
    calculatedRoll = errorTemp * rollKp + rollIntegral * rollKi + rollKd * (errorTemp - previousRollError);
    previousRollError = errorTemp;

    //restrict roll to maximum values
    calculatedRoll = constrain(calculatedRoll, maxRoll * -1, maxRoll);
}

void PIDCalculator::updateMotorPulse() {
    //update individual motor throttle values
    pulseA = throttle - calculatedPitch - calculatedRoll + calculatedYaw;  //front left motor
    pulseB = throttle - calculatedPitch + calculatedRoll - calculatedYaw;  //front right motor
    pulseC = throttle + calculatedPitch - calculatedRoll - calculatedYaw;  //rear left motor
    pulseD = throttle + calculatedPitch + calculatedRoll + calculatedYaw;  //rear right motor

    pulseA = constrain(pulseA, 1000, 2000);
    pulseB = constrain(pulseB, 1000, 2000);
    pulseC = constrain(pulseC, 1000, 2000);
    pulseD = constrain(pulseD, 1000, 2000);
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

void PIDCalculator::updatePitchKp(float kp) {
    EEPROM.update(PITCH_KP_EEPROM_ADDRESS, kp);
}

void PIDCalculator::updatePitchKi(float ki) {
    EEPROM.update(PITCH_KI_EEPROM_ADDRESS, ki);
}

void PIDCalculator::updatePitchKd(float kd) {
    EEPROM.update(PITCH_KD_EEPROM_ADDRESS, kd);
}

void PIDCalculator::updateYawKp(float kp) {
    EEPROM.update(YAW_KP_EEPROM_ADDRESS, kp);
}

void PIDCalculator::updateYawKi(float ki) {
    EEPROM.update(YAW_KI_EEPROM_ADDRESS, ki);
}

void PIDCalculator::updateYawKd(float kd) {
    EEPROM.update(YAW_KD_EEPROM_ADDRESS, kd);
}

