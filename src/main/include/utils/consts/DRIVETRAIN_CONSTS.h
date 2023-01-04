#pragma once

#include <units/angular_velocity.h>
#include <units/velocity.h>

using namespace units;
using namespace units::angular_velocity;
using namespace units::velocity;

/* Swerve Profiling Values */
// Theoretical max is 4.96824 mps or 16.3 fps
constexpr meters_per_second_t MAX_SPEED = 4.96824_mps;
// Theoretical max is track 360 * MAX_SPEED / (hypotenuse * pi) = ~737, safe max is 450.0
constexpr degrees_per_second_t MAX_ANGULAR_VELOCITY = 450.0_deg_per_s;
