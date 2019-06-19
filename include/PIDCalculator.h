#ifndef GREENFLIGHT_PIDCALCULATOR_H
#define GREENFLIGHT_PIDCALCULATOR_H

class PIDCalculator {
public:
    void calculate(int throttleInputChannel, int yawInputChannel, int pitchInputChannel, int rollInputChannel);
    int getCalculatedPulseA();
    int getCalculatedPulseB();
    int getCalculatedPulseC();
    int getCalculatedPulseD();

private:
    void calculateYawPID();
    void calculatePitchPID();
    void calculateRollPID();
    void updateMotorPulse();
};

extern PIDCalculator pidCalculator;

#endif
