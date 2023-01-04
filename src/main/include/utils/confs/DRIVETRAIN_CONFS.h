#pragma once

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

using namespace units;
using namespace units::acceleration;
using namespace units::angle;
using namespace units::angular_acceleration;
using namespace units::angular_velocity;
using namespace units::length;
using namespace units::velocity;

constexpr int NUM_MODULES = 4;

constexpr meter_t TRACK_WIDTH = 21.5_in;
constexpr meter_t WHEEL_BASE  = 21.5_in;

/* Robot XYTheta PID values */
constexpr double XY_TRANSLATION_PFAC       = 4.0;
constexpr double XY_TRANSLATION_IFAC       = 0.0;
constexpr double XY_TRANSLATION_DFAC       = 0.0;
constexpr meter_t XY_TRANSLATION_TOLERANCE = 3_in;

constexpr double THETA_TRANSLATION_PFAC        = 10.0;
constexpr double THETA_TRANSLATION_IFAC        = 0.0;
constexpr double THETA_TRANSLATION_DFAC        = 0.0;
constexpr degree_t THETA_TRANSLATION_TOLERANCE = 6_deg;

/* Robot XYTheta Trapezoidal Profile Limits */
constexpr meters_per_second_t XY_TRANSLATION_MAX_V             = 3_mps;
constexpr meters_per_second_squared_t XY_TRANSLATION_MAX_A     = 3_mps_sq;
constexpr degrees_per_second_t THETA_TRANSLATION_MAX_V         = 360_deg_per_s;
constexpr degrees_per_second_squared_t THETA_TRANSLATION_MAX_A = 360_deg_per_s_sq;

/* Slew Rate Limits */
// TODO Tune Original 10.0 m/s^2 and ~1146 deg/s^2
constexpr meters_per_second_squared_t LINEAR_SLEW_RATE   = 10.0_mps_sq;
constexpr degrees_per_second_squared_t ANGULAR_SLEW_RATE = 900.0_deg_per_s_sq;
