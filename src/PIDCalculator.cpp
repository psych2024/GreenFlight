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

float previousPitchError;
float previousRollError;

float pitchIntegral;
float rollIntegral;

float pitchSetpoint = 0;
float rollSetpoint = 0;

float maxPitch = 500;
float maxRoll = 500;

float errorTemp;

float calculatedPitch;
float calculatedRoll;

int throttleA;
int throttleB;
int throttleC;
int throttleD;

void PIDCalculator::calculate(int throttle) {
    int userThrottle = map(throttle, 1000, 2000, 1000, 1500);

    //calculate pitch pid
    errorTemp = pitchSetpoint - greenImu.getPitch();
    pitchIntegral += errorTemp;
    calculatedPitch = errorTemp * pitchKp + pitchIntegral * pitchKi + pitchKd * (errorTemp - previousPitchError);
    previousPitchError = errorTemp;

    //restrict pitch to maximum values
    if (calculatedPitch > maxPitch)
        calculatedPitch = maxPitch;
    if (calculatedPitch < maxPitch * -1)
        calculatedPitch = maxPitch * -1;

    //calculate roll pid
    errorTemp = rollSetpoint - greenImu.getRoll();
    rollIntegral += errorTemp;
    calculatedRoll = errorTemp * rollKp + rollIntegral * rollKi + rollKd * (errorTemp - previousRollError);
    previousRollError = errorTemp;


    //restrict pitch to maximum values
    if (calculatedRoll > maxPitch)
        calculatedRoll = maxPitch;
    if (calculatedRoll < maxPitch * -1)
        calculatedRoll = maxPitch * -1;

    //update individual motor throttle values
    throttleA = throttle + calculatedPitch - calculatedRoll;  //front left motor
    throttleB = throttle + calculatedPitch + calculatedRoll;  //front right motor
    throttleC = throttle - calculatedPitch - calculatedRoll;  //rear left motor
    throttleD = throttle - calculatedPitch + calculatedRoll;  //rear right motor
}

int PIDCalculator::getCalculatedThrottleA() {
    return throttleA;
}

int PIDCalculator::getCalculatedThrottleB() {
    return throttleB;
}

int PIDCalculator::getCalculatedThrottleC() {
    return throttleC;
}

int PIDCalculator::getCalculatedThrottleD() {
    return throttleD;
}


