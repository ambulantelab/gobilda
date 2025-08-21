#include "gobilda_robot/motor.hpp"

using namespace gobilda_robot;

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>


#include "rclcpp/rclcpp.hpp"

const std::chrono::microseconds PERIOD(20000); // 20ms period for 50Hz

Motor::Motor(int pin, std::string name)
    : pin_{pin}, name_{name}

{
  // The motor controllers respond to signals at 1050 micro-secs -- 1950 micro-secs
  // Set the signal pin as an output
  gpioSetMode(pin_, JET_OUTPUT); // Use the library's set mode function
  return;
}

Motor::~Motor() { trySetVelocity(1500); }// Send neutral signal on desctructor

bool Motor::trySetVelocity(int pulse_width_us) 
{
  // Convert the velocity computed by the diff_driver code
  // into a range from 89-167 which corresponds to the
  // correct values for the frequency used above

  // The values for the motors are flipped so make
  // sure the correct signal is sent to the correct
  // motor
  auto pulse_width = std::chrono::microseconds(pulse_width_us);
    
    // Start the pulse (set pin HIGH)
    int status = gpioWrite(pin_, 1); // Assuming a function like this exists
    if (status < 0) return false;
    
    // Busy-wait for the pulse duration
    auto start = std::chrono::high_resolution_clock::now();
    while (std::chrono::high_resolution_clock::now() - start < pulse_width) {
        // This loop runs until the required pulse width has passed
        // It's CPU intensive but very precise.
    }
    
    // End the pulse (set pin LOW)
    status = gpioWrite(pin_, 0);
    if (status < 0) return false;

    // Sleep for the remaining part of the period
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    auto sleepTime = PERIOD - elapsed;
    if (sleepTime > std::chrono::microseconds(0)) {
        std::this_thread::sleep_for(sleepTime);
    }
    return true;
}
