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

    static void updatePitchKp(float kp);
    static void updatePitchKi(float ki);
    static void updatePitchKd(float kd);

    static void updateYawKp(float kp);
    static void updateYawKi(float ki);
    static void updateYawKd(float kd);

    static void initPIDValues();

    static float yawKp, yawKi, yawKd;
    static float pitchKp, pitchKi, pitchKd;
    static float rollKp, rollKi, rollKd;
private:
    void calculateYawPID();
    void calculatePitchPID();
    void calculateRollPID();
    static void updateMotorPulse();
};

extern PIDCalculator pidCalculator;
#endif
