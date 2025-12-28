#include "SerialParser.h"

SerialParser::SerialParser() {}

bool SerialParser::readPacket(GimbalData &data) {
    if (Serial.available() >= 12) {
        if (Serial.read() == HEADER1) {
            if (Serial.read() == HEADER2) {
                Serial.readBytes(_buffer, 10);

                memcpy(&data.pan_delta, &_buffer[0], 4);
                memcpy(&data.tilt_delta, &_buffer[4], 4);

                data.laser_enable = _buffer[8];
                data.laser_fire = _buffer[9];

                return true;
            }
        }
    }
    return false;
}