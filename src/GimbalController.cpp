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
    if (_lastPIDTime == 0) {
        _lastPIDTime = now;
        return;
    }

    float dt = (now - _lastPIDTime) / 1000.0f;
    _lastPIDTime = now;
    if (dt <= 0 || dt > 0.1) return; // Zaman sıçramalarını engelle

    // 1. Hataları hesapla
    float panError = _targetPan - _currentPan;
    float tiltError = _targetTilt - _currentTilt;

    // --- ÖNEMLİ: YAVAŞLATMA AYARLARI ---
    // Bu değerleri Constants.h içinde de tanımlayabilirsin.
    const float SLOW_MAX_VELOCITY = 15.0f; // Saniyede max 15 derece (Yavaş ve akıcı)
    const float Kp_slow = 1.2f;            // Hedefe yaklaşırken yavaşlamayı kontrol eder
    const float Kd_soft = 0.08f;           // Salınımı durdurmak için minik bir sönümleme
    // ----------------------------------

    // PAN HESABI
    float panDerivative = (panError - _lastPanError) / dt;
    _lastPanError = panError;

    // Hız hesapla (Integrali şimdilik devre dışı bırakmak daha akıcı sonuç verir)
    float panVelocity = (Kp_slow * panError) + (Kd_soft * panDerivative);

    // HIZI SINIRLA (Yavaşlatma burada gerçekleşiyor)
    if (panVelocity > SLOW_MAX_VELOCITY) panVelocity = SLOW_MAX_VELOCITY;
    if (panVelocity < -SLOW_MAX_VELOCITY) panVelocity = -SLOW_MAX_VELOCITY;

    if (abs(panError) > 0.15f) {
        // Hızı adıma çevirirken çok küçük adımları bile yumuşak atması için:
        int stepsToTake = (int)(panVelocity * dt * STEPS_PER_DEGREE);
        
        // Bir döngüde atılacak adım sayısını da kısıtlayalım (Ekstra güvenlik)
        if (stepsToTake > 8) stepsToTake = 8; 
        if (stepsToTake < -8) stepsToTake = -8;

        if (stepsToTake != 0) {
            _panMotor.step(stepsToTake);
        }
    }

    // TILT HESABI (Pan ile aynı limitler)
    float tiltDerivative = (tiltError - _lastTiltError) / dt;
    _lastTiltError = tiltError;

    float tiltVelocity = (Kp_slow * tiltError) + (Kd_soft * tiltDerivative);

    if (tiltVelocity > SLOW_MAX_VELOCITY) tiltVelocity = SLOW_MAX_VELOCITY;
    if (tiltVelocity < -SLOW_MAX_VELOCITY) tiltVelocity = -SLOW_MAX_VELOCITY;

    if (abs(tiltError) > 0.15f) {
        int stepsToTake = (int)(tiltVelocity * dt * STEPS_PER_DEGREE);
        
        if (stepsToTake > 8) stepsToTake = 8;
        if (stepsToTake < -8) stepsToTake = -8;

        if (stepsToTake != 0) {
            _tiltMotor.step(stepsToTake);
        }
    }
}

void GimbalController::applySymmetryLimits(float &val) { //sınır kontrolü
    if (val > MAX_ANGLE) val = MAX_ANGLE;
    if (val < MIN_ANGLE) val = MIN_ANGLE;
}