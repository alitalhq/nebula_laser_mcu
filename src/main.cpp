#include <Arduino.h>
#include "Constants.h"
#include "SerialParser.h"
#include "GimbalController.h"

SerialParser parser;
GimbalController gimbal;
GimbalData incomingData;

unsigned long lastUpdateTick = 0;
const unsigned long updateInterval = 10;

void setup() {
    Serial.begin(BAUD_RATE);
    while (!Serial) {
        ;
    }

    gimbal.setup();
    
    Serial.println("MCU Ready. Listening for ROS 2 commands...");
}

void loop() {
    if (parser.readPacket(incomingData)) {
        gimbal.handleNewData(incomingData);
    }


    unsigned long currentMillis = millis();
    if (currentMillis - lastUpdateTick >= updateInterval) {
        lastUpdateTick = currentMillis;
        
        gimbal.update();
    }
}