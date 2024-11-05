#include "gobilda_robot/motor.hpp"

using namespace gobilda_robot;

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

Motor::Motor(int pin, std::string name)
    : pin_{pin}, name_{name}

{

  int PWMstat1 = gpioSetPWMfrequency(pin_, 10000);
  if (PWMstat1 < 0) exit(-1);

  return;
}

Motor::~Motor() { trySetVelocity(0.0); }

bool Motor::trySetVelocity(double velocity) 
{
  
  int PWMstat;

  if (std::fabs(velocity) < 0.001){
    PWMstat = gpioPWM(pin_, 0);
  }

  else if (velocity > 0){
    PWMstat = gpioPWM(pin_, 250);
  }

  else PWMstat = gpioPWM(pin_, 10);

  if (PWMstat < 0) return false;
  else return true;

}