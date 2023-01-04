#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <numbers>

using namespace units;
using namespace units::angle;
using namespace units::length;

/* Module Configurations */
constexpr meter_t WHEEL_DIAMETER      = 3.94_in;
constexpr meter_t WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * std::numbers::pi;

/* Module Specific Constants */
/* Front Left Module - Module 0 */
constexpr degree_t MOD_0_DEGREE_OFFSET = 81.5_deg;

/* Front Right Module - Module 1 */
constexpr degree_t MOD_1_DEGREE_OFFSET = 212.4_deg;

/* Back Left Module - Module 2 */
constexpr degree_t MOD_2_DEGREE_OFFSET = 338.1_deg;

/* Back Right Module - Module 3 */
constexpr degree_t MOD_3_DEGREE_OFFSET = 8.5_deg;

/* Merge swerve drive constants into easy access vectors */
constexpr degree_t DEGREE_OFFSETS[] = {
    MOD_0_DEGREE_OFFSET, MOD_1_DEGREE_OFFSET, MOD_2_DEGREE_OFFSET, MOD_3_DEGREE_OFFSET};

// (32.0 / 15.0) * (?? / ??)
constexpr double ANGLE_GEAR_RATIO = 12.8 / 1.0;
// L2 ~6.75 : 1
constexpr double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
