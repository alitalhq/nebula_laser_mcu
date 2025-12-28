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

    void applySymmetryLimits(float &val); //limitler için
    void executePID(); // pid için
};

#endif