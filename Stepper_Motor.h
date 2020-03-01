#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "Arduino.h"

#define _STEPPER_CONST 1.8/6  // (step_angle*60s)/360

#define _DIR  0
#define _STEP 1
#define _EN   2

#define _NORMAL     0
#define _ACCELERATE 1

#define ACTIVE_LOW

#ifdef  ACTIVE_LOW
#define ENABLE  false
#define DISABLE true
#else
#define ENABLE  true
#define DISABLE false
#endif

class StepperMotor {
  public:
    StepperMotor(uint8_t dir, uint8_t step, uint8_t en);
    void setMaxSpeed(unsigned int rpm);
    void setAcceleration(unsigned int);
    void move(long);
    void moveTo(long);
    void setCurrentPosition(long);
    void homeConfig(uint8_t, unsigned int d=5000);
    bool home(bool inv = true);
    unsigned long stepsLeft();
    bool runStep();
    
  private:
    uint8_t _pins[3]  = {2, 3, 4};
    uint8_t _homePin;
    
    bool  _direction  = true;
    unsigned int  _maxSpeed     = 0;
    unsigned int  _stepInterval = 0;
    unsigned long _stepCount  = 0;
    unsigned long _stepsToMove  = 0;
    unsigned long _lastStepTime = 0;

    long  _currPos  = 0;
    long  _endPos   = 0;
    unsigned int  _lowBound = 0;
    float _u = 0;
    float _v = 0;
    unsigned long _upBound  = 0;

    typedef struct {
      bool  mode = _NORMAL;
      int acceleration = 0;
    } Acceleration;
    Acceleration _acceleration;

    byte step();
};

#endif
