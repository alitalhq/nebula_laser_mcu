#ifndef SERIAL_PARSER_H
#define SERIAL_PARSER_H

#include <Arduino.h>
#include "Constants.h"

struct GimbalData { //serial monitorden gelecek mesaj
    float pan_delta; //yatay fark
    float tilt_delta; //dikey fark
    bool laser_enable; //mekanizma çalışabilir mi
    bool laser_fire; //lazer ateşlenebilir mi
};

class SerialParser { //class tanımlaması
public:
    SerialParser(); //yapıcı metod
    bool readPacket(GimbalData &data); //serialdan mesajı okuyacak olan fonksiyon (& işareti referans için pointerdaki gibi düşünebilirsin)

private:
    uint8_t _buffer[12]; // 2 Header + 4 Pan + 4 Tilt + 1 En + 1 Fire
};

#endif