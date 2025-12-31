//ana kontrol mekanizması burası
#include "GimbalController.h"

GimbalController::GimbalController() //yapıcı metot
    : _panMotor(PAN_STEP_PIN, PAN_DIR_PIN), //motor nesnelerine pin numaraları atandı
      _tiltMotor(TILT_STEP_PIN, TILT_DIR_PIN) {}

void GimbalController::setup() {
    _hw.begin(); //sensörler başlatıldı
    _panMotor.begin(); // motorlar başlatıldı
    _tiltMotor.begin();

    // Açılış anındaki gerçek mutlak açıyı oku
    startPan = _hw.getPanAngle(); 
    startTilt = _hw.getTiltAngle();

    _panMinLimit = startPan - 30.0f;
    _panMaxLimit = startPan + 30.0f;

    if (_panMinLimit < 0) _panMinLimit += 360.0f;
    if (_panMaxLimit >= 360) _panMaxLimit -= 360.0f;
    
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, LOW); // motor enable edildi (0 ise çalışır)

    _currentPan = startPan;
    _currentTilt = startTilt;
    
    _targetPan = startPan; //sistem açılınca o anki açıyı hedef açı olarak ayarlıyoruz yoksa default 0 olur ve yanlış konuma gider
    _targetTilt = startTilt;
}

void GimbalController::handleNewData(const GimbalData &data) {
    if (!data.laser_enable) return;

    // 1. Yeni potansiyel hedefi hesapla
    nextTargetPan = _targetPan + data.pan_delta;
    nextTargetTilt = _targetTilt + data.tilt_delta;

    // 2. 0-360 Normalizasyonu (Mutlak enkoder için şart)
    while (nextTargetPan >= 360.0f) nextTargetPan -= 360.0f;
    while (nextTargetPan < 0.0f)    nextTargetPan += 360.0f;

    while (nextTargetTilt >= 360.0f) nextTargetTilt -= 360.0f;
    while (nextTargetTilt < 0.0f)    nextTargetTilt += 360.0f;

    // 3. Pan İçin Güvenli Bölge Kontrolü (Dinamik +-30)
    panInside = false;
    if (_panMinLimit < _panMaxLimit) {
        // Normal durum (Örn: 100 ile 160 arası)
        if (nextTargetPan >= _panMinLimit && nextTargetPan <= _panMaxLimit) panInside = true;
    } else {
        // Başa sarma durumu (Örn: 340 ile 10 arası)
        if (nextTargetPan >= _panMinLimit || nextTargetPan <= _panMaxLimit) panInside = true;
    }

    // 4. Tilt İçin Güvenli Bölge Kontrolü
    tiltInside = false;
    if (_tiltMinLimit < _tiltMaxLimit) {
        if (nextTargetTilt >= _tiltMinLimit && nextTargetTilt <= _tiltMaxLimit) tiltInside = true;
    } else {
        if (nextTargetTilt >= _tiltMinLimit || nextTargetTilt <= _tiltMaxLimit) tiltInside = true;
    }

    // 5. Sadece sınır içindeyse hedefi güncelle
    if (panInside)  _targetPan = nextTargetPan;
    if (tiltInside) _targetTilt = nextTargetTilt;
}

void GimbalController::update() { //anlik konum okunur ve pid çalıştırılır
    _currentPan = _hw.getPanAngle();
    _currentTilt = _hw.getTiltAngle();

    executePID();
}

void GimbalController::executePID() {
    unsigned long now = millis();
    if (_lastPIDTime == 0) {
        _lastPIDTime = now;
        return;
    }

    float dt = (now - _lastPIDTime) / 1000.0f;
    _lastPIDTime = now;
    if (dt <= 0 || dt > 0.1) return; 

    // 1. Ham Hataları hesapla
    float panError = _targetPan - _currentPan;
    float tiltError = _targetTilt - _currentTilt;

    // --- KRİTİK EKLEME: En Kısa Yol (Shortest Path) Hesabı ---
    // Eğer enkoder 350'de ve hedef 10'da ise, -340 derece gitmek yerine +20 derece gider.
    if (panError > 180.0f)  panError -= 360.0f;
    if (panError < -180.0f) panError += 360.0f;

    if (tiltError > 180.0f)  tiltError -= 360.0f;
    if (tiltError < -180.0f) tiltError += 360.0f;
    // ---------------------------------------------------------

    // 2. Türev (Derivative) Hesabı
    float panDerivative = (panError - _lastPanError) / dt;
    float tiltDerivative = (tiltError - _lastTiltError) / dt;
    _lastPanError = panError;
    _lastTiltError = tiltError;

    // 3. PID Çıkışı (Hız hesaplama)
    float panVelocity = (PID.kp * panError) + (PID.kd * panDerivative);
    float tiltVelocity = (PID.kp * tiltError) + (PID.kd * tiltDerivative);

    // 4. MAX Hız Sınırı
    if (panVelocity > MAX_ANGULAR_VELOCITY) panVelocity = MAX_ANGULAR_VELOCITY;
    if (panVelocity < -MAX_ANGULAR_VELOCITY) panVelocity = -MAX_ANGULAR_VELOCITY;

    if (tiltVelocity > MAX_ANGULAR_VELOCITY) tiltVelocity = MAX_ANGULAR_VELOCITY;
    if (tiltVelocity < -MAX_ANGULAR_VELOCITY) tiltVelocity = -MAX_ANGULAR_VELOCITY;

    // 5. Motorlara Adım Komutu Gönder
    // Pan Eksenini Hareket Ettir
    if (abs(panError) > 0.05f) { // Ölü bölge (Deadzone)
        int stepsToTake = (int)(panVelocity * dt * STEPS_PER_DEGREE);
        
        // İşlemciyi kilitlememek ve akıcılığı korumak için adım limiti
        if (stepsToTake > 10) stepsToTake = 10;
        if (stepsToTake < -10) stepsToTake = -10;
        
        if (stepsToTake != 0) _panMotor.step(stepsToTake);
    }

    // Tilt Eksenini Hareket Ettir
    if (abs(tiltError) > 0.15f) {
        int stepsToTake = (int)(tiltVelocity * dt * STEPS_PER_DEGREE);
        
        if (stepsToTake > 10) stepsToTake = 10;
        if (stepsToTake < -10) stepsToTake = -10;
        
        if (stepsToTake != 0) _tiltMotor.step(stepsToTake);
    }
}

void GimbalController::applySymmetryLimits(float &val) { //sınır kontrolü
    if (val > MAX_ANGLE) val = MAX_ANGLE;
    if (val < MIN_ANGLE) val = MIN_ANGLE;
}