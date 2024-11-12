#include "gobilda_robot/motor.hpp"

using namespace gobilda_robot;

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

Motor::Motor(int pin, std::string name)
    : pin_{pin}, name_{name}

{

  // The frequency for both motors need to be calculated via
  // the specs on the GoBilda Motor Controllers.
  // The motor controllers respond to signals at 1050 micro-secs -- 1950 micro-secs
  int PWMstat1 = gpioSetPWMfrequency(pin_, 333);
  if (PWMstat1 < 0) exit(-1);

  return;
}

Motor::~Motor() { trySetVelocity(0.0); }

int Motor::convertVelocity(double vel){
  
  // Variable to return the PWM speed for the motors
  int newValue = 0;
  // Values for the max and min from the ros2_diff_drive controller
  // These values are tuned arbitrarily to get some values that work
  float rosMax, rosMin;
  // Values for the max, min based on the required frequency and
  // timing for the Gobilda motor controllers
  float pwmMax, pwmMin;

  // These values are for when the robot is turning in place
  if (-2.0 <= vel && vel <= 2.0) {rosMax = 2.0, rosMin = -2.0;}
  // These values are for when the robot is moving forward
  else {rosMax = 40.0; rosMin = -40.0;}

  // Again we are assuming that pin 15 is on the left motor
  if (pin_ == 15) {pwmMax = 148.0; pwmMin = 108.0;}//on my robot motor moves too fast pwmMax = 168.0, pwmMin = 88.0;
  else {pwmMax = 88.0; pwmMin = 168.0;}

  float oldRange = rosMax - rosMin;
  float newRange = pwmMax - pwmMin;

  newValue = (((vel - rosMin) * newRange) / oldRange) + pwmMin;
  return newValue;
}

bool Motor::trySetVelocity(double velocity) 
{
  // Convert the velocity computed by the diff_driver code
  // into a range from 89-167 which corresponds to the
  // correct values for the frequency used above

  // The values for the motors are flipped so make
  // sure the correct signal is sent to the correct
  // motor
  int PWMstat;
  RCLCPP_INFO(
    rclcpp::get_logger("GobildaSystemHardware"),
    "PWM sent to the controller = %d", convertVelocity(velocity)
  );
  PWMstat = gpioPWM(pin_, convertVelocity(velocity));

  if (PWMstat < 0) return false;
  else return true;

}
