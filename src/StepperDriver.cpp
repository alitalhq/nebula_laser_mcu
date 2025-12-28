//step motoru yönetir
#include "StepperDriver.h"

StepperDriver::StepperDriver(int stepPin, int dirPin) 
    : _stepPin(stepPin), _dirPin(dirPin) {} //yapiciya gelen pin numaralarını atama

void StepperDriver::begin() { //setup pin tanımlamaları
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    digitalWrite(_stepPin, LOW);
}

void StepperDriver::setDirection(bool dir) { //parametre olarak verilen 0 veya 1 e göre dir pinini ayarlar + veya - yönde dönmesini buradan ayarlıycaz
    digitalWrite(_dirPin, dir);
}

void StepperDriver::setEnable(bool state) { //motoru durdurma ve çalıştırma servisi ileride lazım olacak
    digitalWrite(MOTOR_ENABLE_PIN, state ? LOW : HIGH); // 0 ise çalışır 1 ise çalışmaz
}

void StepperDriver::step(int steps) {
    bool dir = (steps > 0); //yön belirler
    setDirection(dir);
    
    int absSteps = abs(steps);
    for (int i = 0; i < absSteps; i++) { //step motoru verilen adım karar döndürür
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(500);
        digitalWrite(_stepPin, LOW);
        delayMicroseconds(500);
    }
}