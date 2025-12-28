#ifndef HARDWARE_INTERFACE_H
#define HARDWARE_INTERFACE_H

#include <Arduino.h>
#include <Wire.h> //i kare ce için
#include <Adafruit_MPU6050.h> //imu için
#include <AS5600.h> //encoder için (berra encoderin ne olduğunu yine unuttu ve şuan araştırıyor kesin)
#include "Constants.h" //sabitler

class HardwareInterface { //sınıf tanımı
public:
    HardwareInterface(); //yapıcı metod
    bool begin(); //başlatma metodu i2c hatlarını açar ve sensörleri kontrol eder
    
    float getPanAngle(); //yatay eksen encoder
    float getTiltAngle(); //dikey eksen encoder
    void getIMUData(float &ax, float &ay, float &az); //imudan ivme verilerini çekiyor
    //ama imu kısmını şuan kullanmıyoruz, stabilize gibi bir mode eklersek orada kullanabiliriz

    void setLaser(bool enable, bool fire); //lazer için

private://iki farklı i2c kanalı açtım çünkü iki encoderin de adresleri aynı. TCA9548A alınca aynı hatta çekeriz
    AS5600 _panEncoder;    // Wire üzerinde
    AS5600 _tiltEncoder;   // Wire1 üzerinde
    Adafruit_MPU6050 _mpu; // Wire1 üzerinde

    float _panOffset = 0.0f; //cihaz açıldığında görülen açılar
    float _tiltOffset = 0.0f;
};

#endif