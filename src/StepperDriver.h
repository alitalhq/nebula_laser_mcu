#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H

#include <Arduino.h>
#include "Constants.h"

class StepperDriver {
public:
    StepperDriver(int stepPin, int dirPin); //yapıcı metod
    void begin(); //motor ayarlarını yapar setup gibi
    void step(int steps); // pozitif ise sağ ve yukarı, negatif ise sol ve aşağı
    void setDirection(bool dir); //driverdaki dir pinini yönetir (dönüş yönü)
    void setEnable(bool state); //driverdaki enable pinini yönetir (pin ortaklandı)

private:
    int _stepPin;
    int _dirPin;
};

#endif