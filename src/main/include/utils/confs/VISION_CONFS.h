#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/angle.h>
#include <units/length.h>

using namespace units;
using namespace units::angle;
using namespace units::length;

constexpr meter_t TARGET_X         = 27_ft;
constexpr meter_t TARGET_Y         = 13.5_ft;
constexpr degree_t TARGET_ROTATION = 0_deg;  // TODO tune this value
constexpr meter_t TARGET_HEIGHT    = 2.6416_m;
// Tuning procedure, set the goal to be the origin
// constexpr meter_t TARGET_X = meter_t{0};
// constexpr meter_t TARGET_Y = meter_t{0};
// align the robot on the Y axis against the wall (aligned with tape)
// tune TARGET_ROTATION until X = 0
// redo this roughly every meter until. the target is out of vision range
// Remove outliers and average remaining values, set TARGET_ROTATION
// Drive the robot along the tape axis slowly
// As the robot drives, verify X stays stable (+/- 3 inches of the line)

// TODO actually find camera to robot measurement
constexpr meter_t CAMERA_X      = 0_m;     // TODO update to swerve
constexpr meter_t CAMERA_Y      = 0_m;     // TODO update to swerve
constexpr meter_t CAMERA_HEIGHT = 30_in;   // TODO update to swerve
constexpr degree_t CAMERA_PITCH = 25_deg;  // TODO update to swerve

constexpr frc::Pose2d FIELD_TO_TARGET{TARGET_X, TARGET_Y, TARGET_ROTATION};
constexpr frc::Pose2d CAMERA_POSE{CAMERA_X, CAMERA_Y, 0_deg};
constexpr frc::Pose2d ROBOT_ORIGIN{0_m, 0_m, 0_deg};
