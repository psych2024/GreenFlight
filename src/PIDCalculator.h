#ifndef GREENFLIGHT_PIDCALCULATOR_H
#define GREENFLIGHT_PIDCALCULATOR_H

class PIDCalculator {
    void calculate(int throttle);
    int getCalculatedThrottleA();
    int getCalculatedThrottleB();
    int getCalculatedThrottleC();
    int getCalculatedThrottleD();
};

extern PIDCalculator calculator;

#endif
