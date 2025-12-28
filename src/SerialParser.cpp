//serial monitoru okur, ilerleyen zamanlarda serial monitore feedback yayınlanmalı
#include "SerialParser.h"

SerialParser::SerialParser() {} //yapıcı metod ama içi boş çünkü şuanlık gerek yok

bool SerialParser::readPacket(GimbalData &data) {//serial monitorden mesajı okuyan kısım
    if (Serial.available() >= 12) { //12 byte veri olursa okumaya başlar
        if (Serial.read() == HEADER1) { //kodun başına iki tane header koydum bunları kontrol ediyor
            if (Serial.read() == HEADER2) {//header olmasaydı kodlar karışabilir
                Serial.readBytes(_buffer, 10);//12 byte'in ilk ikisini zaten okuduk kalan 10byte'ı _buffer dizisine aldık

                memcpy(&data.pan_delta, &_buffer[0], 4);//veriler bölünüp atamalar yapıldı
                memcpy(&data.tilt_delta, &_buffer[4], 4);

                data.laser_enable = _buffer[8];
                data.laser_fire = _buffer[9];

                return true;
            }
        }
    }
    return false;
}