#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include "Constants.h"
#include "HardwareInterface.h"
#include "StepperDriver.h"
#include "SerialParser.h"

class GimbalController {
public:
    GimbalController(); //yapıcı metod
    void setup(); //setup metodu, bir kere çalışır
    void update(); //döngü metodu
    void handleNewData(const GimbalData &data); //yeni hedef gelirse onu işler

private:
    HardwareInterface _hw; //sensör nesnesi
    StepperDriver _panMotor; //yatay motor nesnesi
    StepperDriver _tiltMotor; //dikey motor nesnesi
    
    float _targetPan = 0.0f;
    float _targetTilt = 0.0f;
    float _currentPan = 0.0f;
    float _currentTilt = 0.0f;

    PIDConfig PID;

    float _lastPanError = 0, _lastTiltError = 0;
    float _panIntegral = 0, _tiltIntegral = 0;
    unsigned long _lastPIDTime = 0;

    const float MAX_ANGULAR_VELOCITY = 15.0f;

    void applySymmetryLimits(float &val); //limitler için
    void executePID(); // pid için
};

#endif