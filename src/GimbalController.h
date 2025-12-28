#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include "Constants.h"
#include "HardwareInterface.h"
#include "StepperDriver.h"
#include "SerialParser.h"

class GimbalController {
public:
    GimbalController();
    void setup();
    void update();
    void handleNewData(const GimbalData &data);

private:
    HardwareInterface _hw;
    StepperDriver _panMotor;
    StepperDriver _tiltMotor;
    
    float _targetPan = 0.0f;
    float _targetTilt = 0.0f;
    float _currentPan = 0.0f;
    float _currentTilt = 0.0f;

    void applySymmetryLimits(float &val);
    void executePID();
};

#endif