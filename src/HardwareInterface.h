#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <AS5600.h>
#include "Constants.h"

class HardwareInterface {
public:
    HardwareInterface();
    bool begin();
    
    // Sensör okuma
    float getPanAngle();   // AS5600 Pan
    float getTiltAngle();  // AS5600 Tilt
    void getIMUData(float &ax, float &ay, float &az);

    // Lazer kontrol
    void setLaser(bool enable, bool fire);

private:
    AS5600 _panEncoder;    // Wire üzerinde
    AS5600 _tiltEncoder;   // Wire1 üzerinde
    Adafruit_MPU6050 _mpu; // Wire1 üzerinde

    float _panOffset = 0.0f;
    float _tiltOffset = 0.0f;
};

#endif