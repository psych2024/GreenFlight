#include "HID.h"
#include "PIDCalculator.h"
#include "GreenIMU.h"
#include "EEPROM.h"

float pitchKp;
float pitchKi;
float pitchKd;

float rollKp;
float rollKi;
float rollKd;

float yawKp;
float yawKi;
float yawKd;

float previousPitchError;
float previousRollError;
float previousYawError;

float pitchIntegral;
float rollIntegral;
float yawIntegral;

float pitchSetpoint = 0;
float rollSetpoint = 0;
float yawSetpoint = 0;

float maxPitch = 300;
float maxRoll = 300;
float maxYaw = 300;

float errorTemp;

int throttle;
float calculatedYaw, calculatedPitch, calculatedRoll;

int pulseA, pulseB, pulseC, pulseD;

void PIDCalculator::initPIDValues() {
    EEPROM.get(PITCH_KP_EEPROM_ADDRESS, pitchKp);
    EEPROM.get(PITCH_KI_EEPROM_ADDRESS, pitchKi);
    EEPROM.get(PITCH_KD_EEPROM_ADDRESS, pitchKd);

    rollKp = pitchKp;
    rollKi = pitchKi;
    rollKd = pitchKd;

    EEPROM.get(YAW_KP_EEPROM_ADDRESS, yawKp);
    EEPROM.get(YAW_KI_EEPROM_ADDRESS, yawKi);
    EEPROM.get(YAW_KD_EEPROM_ADDRESS, yawKd);
}

void
PIDCalculator::calculate(int throttleInputChannel, int yawInputChannel, int pitchInputChannel, int rollInputChannel) {
    //lower throttle value by 25% to allow room for pid control
    throttle = map(throttleInputChannel, 1000, 2000, 1000, 1750);
    yawSetpoint = map(yawInputChannel, 1000, 2000, -45, 45);
    pitchSetpoint = map(pitchInputChannel, 1000, 2000, -45, 45);
    rollSetpoint = map(rollInputChannel, 1000, 2000, -45, 45);

    //limit setpoint to zero around middle of joystick +-20
    if (yawInputChannel < 1510 && yawInputChannel > 1490) yawSetpoint = 0;
    if (pitchInputChannel < 1510 && pitchInputChannel > 1490) pitchSetpoint = 0;
    if (rollInputChannel < 1510 && rollInputChannel > 1490) rollSetpoint = 0;

    calculateYawPID();

    calculatePitchPID();

    calculateRollPID();

    updateMotorPulse();
}

void PIDCalculator::calculateYawPID() {
    //do not include yaw if motor is at rest
    if(throttle < 1100) {
        calculatedYaw = 0;
        return;
    }

    //calculate yaw pid
    errorTemp = yawSetpoint - greenImu.getYaw();
    yawIntegral += errorTemp;
    yawIntegral = constrain(yawIntegral, maxYaw * -1, maxYaw);

    calculatedYaw = errorTemp * yawKp + yawIntegral * yawKi + yawKd * (errorTemp - previousYawError);
    previousYawError = errorTemp;

    //restrict yaw to maximum values
    calculatedYaw = constrain(calculatedYaw, maxYaw * -1, maxYaw);
}

void PIDCalculator::calculatePitchPID() {
    //calculate pitch pid
    errorTemp = pitchSetpoint - greenImu.getPitch();
    pitchIntegral += errorTemp;
    pitchIntegral = constrain(pitchIntegral, maxPitch * -1, maxPitch);

    calculatedPitch = errorTemp * pitchKp + pitchIntegral * pitchKi + pitchKd * (errorTemp - previousPitchError);
    previousPitchError = errorTemp;

    //restrict pitch to maximum values
    calculatedPitch = constrain(calculatedPitch, maxPitch * -1, maxPitch);
}

void PIDCalculator::calculateRollPID() {
    //calculate roll pid
    errorTemp = rollSetpoint - greenImu.getRoll();
    rollIntegral += errorTemp;
    rollIntegral = constrain(rollIntegral, maxRoll * -1, maxRoll);

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

void PIDCalculator::updatePitchKp(float kp) {
    EEPROM.put(PITCH_KP_EEPROM_ADDRESS, kp);
}

void PIDCalculator::updatePitchKi(float ki) {
    EEPROM.put(PITCH_KI_EEPROM_ADDRESS, ki);
}

void PIDCalculator::updatePitchKd(float kd) {
    EEPROM.put(PITCH_KD_EEPROM_ADDRESS, kd);
}

void PIDCalculator::updateYawKp(float kp) {
    EEPROM.put(YAW_KP_EEPROM_ADDRESS, kp);
}

void PIDCalculator::updateYawKi(float ki) {
    EEPROM.put(YAW_KI_EEPROM_ADDRESS, ki);
}

void PIDCalculator::updateYawKd(float kd) {
    EEPROM.put(YAW_KD_EEPROM_ADDRESS, kd);
}