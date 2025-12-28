#include "GimbalController.h"

GimbalController::GimbalController() 
    : _panMotor(PAN_STEP_PIN, PAN_DIR_PIN), 
      _tiltMotor(TILT_STEP_PIN, TILT_DIR_PIN) {}

void GimbalController::setup() {
    _hw.begin();
    _panMotor.begin();
    _tiltMotor.begin();
    
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
    
    _currentPan = _hw.getPanAngle();
    _currentTilt = _hw.getTiltAngle();
    _targetPan = _currentPan;
    _targetTilt = _currentTilt;
}

void GimbalController::handleNewData(const GimbalData &data) {
    _targetPan += data.pan_delta;
    _targetTilt += data.tilt_delta;
    
    applySymmetryLimits(_targetPan);
    applySymmetryLimits(_targetTilt);
    
    _hw.setLaser(data.laser_enable, data.laser_fire);
}

void GimbalController::update() {
    _currentPan = _hw.getPanAngle();
    _currentTilt = _hw.getTiltAngle();
    
    executePID();
}

void GimbalController::executePID() {
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

void GimbalController::applySymmetryLimits(float &val) {
    if (val > MAX_ANGLE) val = MAX_ANGLE;
    if (val < MIN_ANGLE) val = MIN_ANGLE;
}