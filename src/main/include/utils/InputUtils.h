#pragma once

#include <units/angular_velocity.h>
#include <units/velocity.h>

using namespace units;
using namespace units::velocity;
using namespace units::angular_velocity;

meters_per_second_t ScaleJoystickX(double rawAxis);
meters_per_second_t ScaleJoystickY(double rawAxis);
degrees_per_second_t ScaleJoystickTheta(double rawAxis);
