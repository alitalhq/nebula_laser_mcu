#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

#include <Arduino.h>
#include "Constants.h"

struct GimbalData {
    float pan_delta;
    float tilt_delta;
    bool laser_enable;
    bool laser_fire;
};

class SerialParser {
public:
    SerialParser();
    bool readPacket(GimbalData &data);

private:
    uint8_t _buffer[12]; // 2 Header + 4 Pan + 4 Tilt + 1 En + 1 Fire
};

#endif