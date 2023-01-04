#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>

using namespace units;
using namespace units::angle;
using namespace units::angular_velocity;

// ============= Falcon Constants =============
// ticks per motor rotation
constexpr double FALCON_ENCODER_TICKS = 2048.0;
constexpr auto MAX_FALCON_RPM         = 5800.0_rpm;

// multiply to convert constants
constexpr revolutions_per_minute_t FALCON_TO_RPM = 600.0_rpm / FALCON_ENCODER_TICKS;
constexpr auto RPM_TO_FALCON                     = 1.0 / FALCON_TO_RPM;
constexpr degree_t FALCON_TO_DEG                 = 360.0_deg / FALCON_ENCODER_TICKS;
constexpr auto DEG_TO_FALCON                     = FALCON_ENCODER_TICKS / 360.0_deg;
constexpr double FALCON_TICKS_TO_ROT             = 1.0 / FALCON_ENCODER_TICKS;
constexpr double ROT_TO_FALCON_TICKS             = FALCON_ENCODER_TICKS;
