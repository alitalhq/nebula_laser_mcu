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
    
    _targetPan = startPan;
    _targetTilt = startTilt;
}

void GimbalController::handleNewData(const GimbalData &data) {
    if (!data.laser_enable) return;

    nextTargetPan = _targetPan + data.pan_delta;
    nextTargetTilt = _targetTilt + data.tilt_delta;

    while (nextTargetPan >= 360.0f) nextTargetPan -= 360.0f;
    while (nextTargetPan < 0.0f)    nextTargetPan += 360.0f;

    while (nextTargetTilt >= 360.0f) nextTargetTilt -= 360.0f;
    while (nextTargetTilt < 0.0f)    nextTargetTilt += 360.0f;

    panInside = false;
    if (_panMinLimit < _panMaxLimit) {
        if (nextTargetPan >= _panMinLimit && nextTargetPan <= _panMaxLimit) panInside = true;
    } else {
        if (nextTargetPan >= _panMinLimit || nextTargetPan <= _panMaxLimit) panInside = true;
    }

    tiltInside = false;
    if (_tiltMinLimit < _tiltMaxLimit) {
        if (nextTargetTilt >= _tiltMinLimit && nextTargetTilt <= _tiltMaxLimit) tiltInside = true;
    } else {
        if (nextTargetTilt >= _tiltMinLimit || nextTargetTilt <= _tiltMaxLimit) tiltInside = true;
    }

    if (panInside)  _targetPan = nextTargetPan;
    if (tiltInside) _targetTilt = nextTargetTilt;
}

void GimbalController::update() {
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

    float panError = _targetPan - _currentPan;
    float tiltError = _targetTilt - _currentTilt;

    if (panError > 180.0f)  panError -= 360.0f;
    if (panError < -180.0f) panError += 360.0f;
    if (tiltError > 180.0f)  tiltError -= 360.0f;
    if (tiltError < -180.0f) tiltError += 360.0f;

    _panIntegral += panError * dt;
    _tiltIntegral += tiltError * dt;
    
    _panIntegral = constrain(_panIntegral, -5.0f, 5.0f);
    _tiltIntegral = constrain(_tiltIntegral, -5.0f, 5.0f);

    float panDerivative = (panError - _lastPanError) / dt;
    float tiltDerivative = (tiltError - _lastTiltError) / dt;
    _lastPanError = panError;
    _lastTiltError = tiltError;

    float panVelocity = (PID.kp * panError) + (PID.ki * _panIntegral) + (PID.kd * panDerivative);
    float tiltVelocity = (PID.kp * tiltError) + (PID.ki * _tiltIntegral) + (PID.kd * tiltDerivative);

    panVelocity = constrain(panVelocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
    tiltVelocity = constrain(tiltVelocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

    if (abs(panError) > DEAD_ZONE) {
        int stepsToTake = (int)(panVelocity * dt * STEPS_PER_DEGREE);
        
        stepsToTake = constrain(stepsToTake, -50, 50);
        
        if (stepsToTake != 0) _panMotor.step(stepsToTake);
    } else {
        _panIntegral = 0;
    }

    if (abs(tiltError) > DEAD_ZONE) {
        int stepsToTake = (int)(tiltVelocity * dt * STEPS_PER_DEGREE);
        stepsToTake = constrain(stepsToTake, -50, 50);
        
        if (stepsToTake != 0) _tiltMotor.step(stepsToTake);
    } else {
        _tiltIntegral = 0;
    }
}

void GimbalController::applySymmetryLimits(float &val) {
    if (val > MAX_ANGLE) val = MAX_ANGLE;
    if (val < MIN_ANGLE) val = MIN_ANGLE;
}