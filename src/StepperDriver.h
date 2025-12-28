#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <Arduino.h>
#include "Constants.h"

class StepperDriver {
public:
    StepperDriver(int stepPin, int dirPin);
    void begin();
    void step(int steps); // Pozitif sağ/yukarı, negatif sol/aşağı
    void setDirection(bool dir);
    void setEnable(bool state);

private:
    int _stepPin;
    int _dirPin;
};

#endif