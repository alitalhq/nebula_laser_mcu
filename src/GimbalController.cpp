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
    if (!data.laser_enable) {
        return; 
    }
    _targetPan += data.pan_delta;
    _targetTilt += data.tilt_delta;

    while (_targetPan >= 360.0f){
        _targetPan -= 360.0f;
    }
    while (_targetPan < 0.0f){
        _targetPan += 360.0f;
    }
    while (_targetTilt >= 360.0f){
         _targetTilt -= 360.0f;
    }
    while (_targetTilt < 0.0f){
        _targetTilt += 360.0f;
    }
    
    applySymmetryLimits(_targetPan); //sınırlar kontrol edildi
    applySymmetryLimits(_targetTilt);
    
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
    if (abs(panError) > 0.15f) { // Ölü bölge (Deadzone)
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