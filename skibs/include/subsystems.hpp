#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Motor intake(13);  // Make this number negative if you want to reverse the motor
inline ez::Piston mogo('H');
inline ez::Piston doinker('G');
inline pros::MotorGroup lady({12, -18});
inline pros::Optical colorSensor(1);  // Optical sensor on smart port 1
// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');