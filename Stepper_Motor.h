#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "Arduino.h"

#define _STEPPER_CONST 6/1.8  // 360/(step_angle*60s)

#define _DIR  0
#define _STEP 1
#define _EN   2

#define _NORMAL     0
#define _ACCELERATE 1

class StepperMotor {
  public:
    StepperMotor(uint8_t dir, uint8_t step, uint8_t en);
    void setMaxSpeed(unsigned int rpm);
    void setAcceleration(unsigned int, unsigned int);
    void move(long);
    void moveTo(long);
    void setCurrentPosition(long);
    unsigned long stepsLeft();
    bool runStep();
    
  private:
    uint8_t _pins[3]  = {2, 3, 4};
    
    bool  _direction  = true;
    unsigned int  _maxSpeed   = 0;
    unsigned long _stepCount  = 0;
    unsigned long _stepsToMove  = 0;
    unsigned long _lastStepTime = 0;

    long  _currPos  = 0;
    long  _endPos   = 0;
    unsigned long _upBound  = 0;
    unsigned long _lowBound = 0;

    typedef struct {
      bool  mode = _NORMAL;
      uint8_t repeatSpeed = 20;
      uint8_t duration    = 5;
    } Acceleration;
    Acceleration _acceleration;

    byte step();
};

#endif
