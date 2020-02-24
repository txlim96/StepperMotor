#include "Stepper_Motor.h"

StepperMotor::StepperMotor(uint8_t dir, uint8_t step, uint8_t en) {
  _maxSpeed = 0;
  _currPos  = 0;
  _endPos   = 0;
  _pins[_DIR] = dir;
  _pins[_STEP] = step;
  _pins[_EN] = en;
  _lastStepTime = micros();

  for (byte i = 0; i < 3; i++) {
    pinMode(_pins[i], OUTPUT);
  }
  
  digitalWrite(_pins[_EN], HIGH);
  digitalWrite(_pins[_STEP], LOW);
  digitalWrite(_pins[_DIR], _direction);
}

void StepperMotor::setMaxSpeed(unsigned int rpm) {
  _maxSpeed = 1000000 / (rpm * _STEPPER_CONST);
}

void StepperMotor::setAcceleration(unsigned int f, unsigned int d) {
  _acceleration.mode = _ACCELERATE;
  _acceleration.repeatSpeed = f;
  _acceleration.duration    = d;
}

void StepperMotor::move(long relative) {
  _stepsToMove = abs(relative);
  _stepCount = 0;
  _endPos += relative;
  
  _lowBound = _acceleration.repeatSpeed*_acceleration.duration;
  _upBound  = _stepsToMove - _lowBound;
  _direction = (relative > 0 ? HIGH : LOW);
  digitalWrite(_pins[_DIR], _direction);
}

void StepperMotor::moveTo(long absolute) {
  _stepsToMove = abs(absolute - _endPos);
  _stepCount = 0;
  _endPos = absolute;

  _lowBound = _acceleration.repeatSpeed*_acceleration.duration;
  _upBound  = _stepsToMove - _lowBound;
  _direction = (absolute > 0 ? HIGH : LOW);
  digitalWrite(_pins[_DIR], _direction);
}

void StepperMotor::setCurrentPosition(long current) {
  _currPos = current;
}

unsigned long StepperMotor::stepsLeft() {
  if (abs(_endPos - _currPos) == 0) digitalWrite(_pins[_EN], HIGH);
  return abs(_endPos - _currPos);
}

bool StepperMotor::runStep() {
  if (_stepCount >= _stepsToMove) {
    digitalWrite(_pins[_EN], HIGH);
    return false;
  }

  unsigned int _stepInterval = 0;
  
  digitalWrite(_pins[_EN], LOW);
  if (_acceleration.mode == _ACCELERATE) {
    if (_lowBound <= _stepCount && _stepCount <= _upBound) {
      _stepInterval = _maxSpeed;
    }
    else {
      unsigned int _accCount = (_stepCount > _upBound ? _stepsToMove-_stepCount : _stepCount);
      _stepInterval = _maxSpeed * floor(_acceleration.duration-_accCount/_acceleration.repeatSpeed);
    }
  }
  else _stepInterval = _maxSpeed;
  
  unsigned long t = micros();
  if (t - _lastStepTime >= _stepInterval) {
    _currPos += (_direction ? 1 : -1);
    _stepCount += step();
    _lastStepTime = t;
    return true;
  }
  else return false;
}

byte StepperMotor::step() {
  digitalWrite(_pins[_STEP], HIGH);
  delayMicroseconds(1);
  digitalWrite(_pins[_STEP], LOW);
  return 1;
}
