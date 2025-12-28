//ana kontrol mekanizması burası
#include "GimbalController.h"

GimbalController::GimbalController() //yapıcı metot
    : _panMotor(PAN_STEP_PIN, PAN_DIR_PIN), //motor nesnelerine pin numaraları atandı
      _tiltMotor(TILT_STEP_PIN, TILT_DIR_PIN) {}

void GimbalController::setup() {
    _hw.begin(); //sensörler başlatıldı
    _panMotor.begin(); // motorlar başlatıldı
    _tiltMotor.begin();
    
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, LOW); // motor enable edildi (0 ise çalışır)
    
    _currentPan = _hw.getPanAngle(); // şuanki açılar alınır
    _currentTilt = _hw.getTiltAngle();
    _targetPan = _currentPan; //sistem açılınca o anki açıyı hedef açı olarak ayarlıyoruz yoksa default 0 olur ve yanlış konuma gider
    _targetTilt = _currentTilt;
}

void GimbalController::handleNewData(const GimbalData &data) {//okunan veriyi yakalayıp yeni hedef açıyı oluşturur
    _targetPan = _currentPan + data.pan_delta;
    _targetTilt = _currentTilt + data.tilt_delta;
    
    applySymmetryLimits(_targetPan); //sınırlar kontrol edildi
    applySymmetryLimits(_targetTilt);
    
}

void GimbalController::update() { //anlik konum okunur ve pid çalıştırılır
    _currentPan = _hw.getPanAngle();
    _currentTilt = _hw.getTiltAngle();
    
    executePID();
}

void GimbalController::executePID() { //şuanlık pid yok burası düzeltilecek, şuanda sadece hedef açıya yönlendiriyor
    float panError = _targetPan - _currentPan;
    float tiltError = _targetTilt - _currentTilt;

    if (abs(panError) > 0.1f) {
        int stepsToTake = (int)(panError * STEPS_PER_DEGREE);
        _panMotor.step(stepsToTake);
    }

    if (abs(tiltError) > 0.1f) {
        int stepsToTake = (int)(tiltError * STEPS_PER_DEGREE);
        _tiltMotor.step(stepsToTake);
    }
}

void GimbalController::applySymmetryLimits(float &val) { //sınır kontrolü
    if (val > MAX_ANGLE) val = MAX_ANGLE;
    if (val < MIN_ANGLE) val = MIN_ANGLE;
}