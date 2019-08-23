#ifndef GREENFLIGHT_PIDCALCULATOR_H
#define GREENFLIGHT_PIDCALCULATOR_H

#define PITCH_KP_EEPROM_ADDRESS 256
#define PITCH_KI_EEPROM_ADDRESS 260
#define PITCH_KD_EEPROM_ADDRESS 264

#define YAW_KP_EEPROM_ADDRESS 268
#define YAW_KI_EEPROM_ADDRESS 272
#define YAW_KD_EEPROM_ADDRESS 276

class PIDCalculator {
public:
    void calculate(int throttleInputChannel, int yawInputChannel, int pitchInputChannel, int rollInputChannel);
    int getCalculatedPulseA();
    int getCalculatedPulseB();
    int getCalculatedPulseC();
    int getCalculatedPulseD();

    void updatePitchKp(float kp);
    void updatePitchKi(float ki);
    void updatePitchKd(float kd);

    void updateYawKp(float kp);
    void updateYawKi(float ki);
    void updateYawKd(float kd);
private:
    void calculateYawPID();
    void calculatePitchPID();
    void calculateRollPID();
    void updateMotorPulse();
};

extern PIDCalculator pidCalculator;

extern const float pitchKp;
extern const float pitchKi;
extern const float pitchKd;
extern const float yawKp;
extern const float yawKi;
extern const float yawKd;

#endif
