#include "StepperDriver.h"

StepperDriver::StepperDriver(int stepPin, int dirPin) 
    : _stepPin(stepPin), _dirPin(dirPin) {}

void StepperDriver::begin() {
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_stepPin, LOW);
}

void StepperDriver::setDirection(bool dir) {
    digitalWrite(_dirPin, dir);
}

void StepperDriver::setEnable(bool state) {
    digitalWrite(MOTOR_ENABLE_PIN, state ? LOW : HIGH);
}

void StepperDriver::step(int steps) {
    bool dir = (steps > 0);
    setDirection(dir);
    
    int absSteps = abs(steps);
    for (int i = 0; i < absSteps; i++) {
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(_stepPin, LOW);
        delayMicroseconds(500);
    }
}