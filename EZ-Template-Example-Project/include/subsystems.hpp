#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Motor intake(13);  // Make this number negative if you want to reverse the motor
inline ez::Piston mogo('A');
inline ez::Piston doinker('H');
inline pros::MotorGroup lady({12, -18});
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');