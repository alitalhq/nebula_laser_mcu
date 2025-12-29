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

void GimbalController::executePID() {
    unsigned long now = millis();
    // İlk çalıştırmada zaman farkını 0 yapmamak için kontrol
    if (_lastPIDTime == 0) {
        _lastPIDTime = now;
        return;
    }

    float dt = (now - _lastPIDTime) / 1000.0f; // Milisaniyeyi saniyeye çevir
    _lastPIDTime = now;

    if (dt <= 0) return;

    // Hata hesaplama
    float panError = _targetPan - _currentPan;
    float tiltError = _targetTilt - _currentTilt;

    // --- PAN (Yatay) HESABI ---
    _panIntegral += panError * dt;
    float panDerivative = (panError - _lastPanError) / dt;
    
    // ÇIKIŞ: İdeal Hız (Derece/Saniye)
    float panVelocity = (PID.kp * panError) + (PID.ki * _panIntegral) + (PID.kd * panDerivative);

    // HIZ SINIRI (Açısal Hız Limiti)
    if (panVelocity > MAX_ANGULAR_VELOCITY) panVelocity = MAX_ANGULAR_VELOCITY;
    if (panVelocity < -MAX_ANGULAR_VELOCITY) panVelocity = -MAX_ANGULAR_VELOCITY;

    // Hızı fiziksel adıma çeviriyoruz (Hız * Zaman * Çarpan)
    if (abs(panError) > 0.1f) {
        int stepsToTake = (int)(panVelocity * dt * STEPS_PER_DEGREE);
        if (stepsToTake != 0) {
            _panMotor.step(stepsToTake);
        }
    }

    // --- TILT (Dikey) HESABI ---
    // (Pan ile aynı mantık, sadece değişkenler tilt olacak)
    _tiltIntegral += tiltError * dt;
    float tiltDerivative = (tiltError - _lastTiltError) / dt;
    float tiltVelocity = (PID.kp * tiltError) + (PID.ki * _tiltIntegral) + (PID.kd * tiltDerivative);

    if (tiltVelocity > MAX_ANGULAR_VELOCITY) tiltVelocity = MAX_ANGULAR_VELOCITY;
    if (tiltVelocity < -MAX_ANGULAR_VELOCITY) tiltVelocity = -MAX_ANGULAR_VELOCITY;

    if (abs(tiltError) > 0.1f) {
        int stepsToTake = (int)(tiltVelocity * dt * STEPS_PER_DEGREE);
        if (stepsToTake != 0) {
            _tiltMotor.step(stepsToTake);
        }
    }

    // Geçmiş hataları sakla (Bir sonraki döngü için)
    _lastPanError = panError;
    _lastTiltError = tiltError;
}

void GimbalController::applySymmetryLimits(float &val) { //sınır kontrolü
    if (val > MAX_ANGLE) val = MAX_ANGLE;
    if (val < MIN_ANGLE) val = MIN_ANGLE;
}