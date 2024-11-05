#pragma once

#ifndef GOBILDA_ROBOT__MOTOR_HPP_
#define GOBILDA_ROBOT__MOTOR_HPP_

#include <memory>
#include <tuple>
#include <string>

#include "jetgpio.h"

namespace gobilda_robot {
class Motor {
    public:
        Motor() = default;
        Motor(int pin, std::string name);
        ~Motor();
        bool trySetVelocity(double velocity);

    private:
        int pin_;
        std::string name_;
};
}  // namespace GOBILDA

#endif  // GOBILDA_ROBOT__MOTOR_HPP