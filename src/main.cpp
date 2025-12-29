#include <Arduino.h>
#include "Constants.h"
#include "SerialParser.h"
#include "GimbalController.h"

SerialParser parser; //serial monitör okumak için nesne (daha fazla detay için SerialParser'a bak)
GimbalController gimbal; //ana kontrol nesnesi (daha fazla detay için GimbalControll'a bak)
GimbalData incomingData {0, 0, false, false}; //monitörden okunan veri nesnesi

unsigned long lastUpdateTick = 0;
const unsigned long updateInterval = 10; //çalışma hızı 10ms yani saniyede 100 kez 100Hz

void setup() {
    Serial.begin(BAUD_RATE); //serial monitör açıldı
    while (!Serial) {
        ;
    }

    while(Serial.available() > 0){
        Serial.read(); //eski verileri temizledim umarım bug gider
    }

    gimbal.setup(); //gimbal kuruldu (motorlar ve sensörler de bu fonksiyon içinde kurulmuş oluyor daha fazla detay için kontrol et)
    
    Serial.println("MCU is Ready...");
}

void loop() {
    if (parser.readPacket(incomingData)) { //incomingData referans ile gönderildi & bu yüzden değer aldı
        gimbal.handleNewData(incomingData); //değer gimbal controllera gönderildi
    }


    unsigned long currentMillis = millis();
    if (currentMillis - lastUpdateTick >= updateInterval) {
        lastUpdateTick = currentMillis;
        
        gimbal.update();
    }
}