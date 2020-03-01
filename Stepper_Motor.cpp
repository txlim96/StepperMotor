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

  digitalWrite(_pins[_EN], DISABLE);
  digitalWrite(_pins[_STEP], LOW);
  digitalWrite(_pins[_DIR], _direction);
}

void StepperMotor::setMaxSpeed(unsigned int rpm) {
  _maxSpeed = 1000000 * _STEPPER_CONST / rpm;
}

void StepperMotor::setAcceleration(unsigned int a) {
  _acceleration.acceleration = a;
  _lowBound = pow((1000000 * _STEPPER_CONST / _maxSpeed), 2) / (2 * a); // v^2 = u^2 + 2*a*s
  _acceleration.mode = _ACCELERATE;

  _u = 0;
}

void StepperMotor::move(long relative) {
  _stepsToMove = abs(relative);
  _stepCount = 0;
  _endPos += relative;

  _u = 0;
  _direction = (relative > 0 ? HIGH : LOW);
  digitalWrite(_pins[_DIR], _direction);
}

void StepperMotor::moveTo(long absolute) {
  _stepsToMove = abs(absolute - _endPos);
  _stepCount = 0;
  _endPos = absolute;

  _direction = (absolute > 0 ? HIGH : LOW);
  digitalWrite(_pins[_DIR], _direction);
}

void StepperMotor::setCurrentPosition(long current) {
  _currPos = current;
}

void StepperMotor::homeConfig(uint8_t sw, unsigned int d) {
  _homePin = sw;
  pinMode(_homePin, INPUT_PULLUP);
  _stepInterval = d;
  _lastStepTime = micros();
}

bool StepperMotor::home(bool inverse) {
  digitalWrite(_pins[_EN], ENABLE);
  unsigned long t = micros();
  if (t - _lastStepTime >= _stepInterval) {
    step();
    _lastStepTime = t;
  }
  bool limit = digitalRead(_homePin);
  if (limit == inverse) _currPos = 0;
  digitalWrite(_pins[_EN], DISABLE);
  return (inverse ? limit : !limit);
}

unsigned long StepperMotor::stepsLeft() {
  //  if (_stepsToMove - _stepCount == 0) digitalWrite(_pins[_EN], HIGH);
  return _stepsToMove - _stepCount;
}

bool StepperMotor::runStep() {
  if (stepsLeft() == 0) {
    digitalWrite(_pins[_EN], DISABLE);
    return false;
  }

  digitalWrite(_pins[_EN], ENABLE);
  unsigned long t = micros();
  if (t - _lastStepTime >= _stepInterval) {
    if (_acceleration.mode == _ACCELERATE) {
      if (_lowBound <= _stepCount && _stepCount <= _stepsToMove - _lowBound) {
        _stepInterval = _maxSpeed;
      }
      else {
        int d = _stepCount < _lowBound ? 1 : -1;
        _v = sqrt(pow(_u, 2) + 2 * _acceleration.acceleration * d);
        _u = _v;
        _stepInterval = 1000000 * _STEPPER_CONST / _v;
      }
    }
    else _stepInterval = _maxSpeed;

    _currPos += (_direction ? 1 : -1);
    _stepCount += step();
    //    Serial.print(_currPos);
    //    Serial.print("\t");
    //    Serial.print(_endPos);
    //    Serial.print("\t");
    //    Serial.println(_stepCount);
    //    Serial.print("\t");
    //    Serial.println(_stepsToMove);
    //    Serial.print("\t");
    //    Serial.println(_stepInterval);

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
