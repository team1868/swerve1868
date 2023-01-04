#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_BaseMotorController.h>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <units/voltage.h>

using namespace units;
using namespace units::acceleration;
using namespace units::velocity;
using namespace units::voltage;

/* Drive Motor Characterization Values */
// divide by 12 to convert from volts to percent output for CTRE (not sure if this is needed)
constexpr units::volt_t DRIVE_KS = 0.667_V / 12;
constexpr auto DRIVE_KV          = 2.44_V / 12_mps;
constexpr auto DRIVE_KA          = 0.27_V / 12_mps_sq;

/* Neutral Modes */
constexpr auto ANGLE_NEUTRAL_MODE = ctre::phoenix::motorcontrol::NeutralMode::Coast;
constexpr auto DRIVE_NEUTRAL_MODE = ctre::phoenix::motorcontrol::NeutralMode::Brake;

/* Motor Inverts */
constexpr bool DRIVE_MOTOR_INVERTED = false;
constexpr bool ANGLE_MOTOR_INVERTED = false;

/* Angle Encoder Invert */
constexpr bool CAN_CODER_INVERTED = false;

constexpr double OPEN_LOOP_RAMP   = 0.25;
constexpr double CLOSED_LOOP_RAMP = 0.0;

/* Swerve Current Limiting */
constexpr int ANGLE_CONTINUOUS_CURRENT_LIMIT = 30;
constexpr int ANGLE_PEAK_CURRENT_LIMIT       = 40;
constexpr double ANGLE_PEAK_CURRENT_DURATION = 0.1;
constexpr bool ANGLE_ENABLE_CURRENT_LIMIT    = true;

constexpr int DRIVE_CONTINUOUS_CURRENT_LIMIT = 40;
constexpr int DRIVE_PEAK_CURRENT_LIMIT       = 40;  // 60 stock
constexpr double DRIVE_PEAK_CURRENT_DURATION = 0.1;
constexpr bool DRIVE_ENABLE_CURRENT_LIMIT    = true;

/* Angle Motor Closed Loop PID Values */
constexpr double ANGLE_PFAC = 0.6;
constexpr double ANGLE_IFAC = 0.0;
constexpr double ANGLE_DFAC = 12.0;
constexpr double ANGLE_FFAC = 0.0;

/* Drive Motor Closed Loop PID Values */
constexpr double DRIVE_PFAC = 0.10;
constexpr double DRIVE_IFAC = 0.0;
constexpr double DRIVE_DFAC = 0.0;
constexpr double DRIVE_FFAC = 0.0;

// https://github.com/FRCTeam2910/2021CompetitionRobot/blob/5fabbff6814a8fa71ef614f691342847ad885bf5/src/main/java/org/frcteam2910/c2020/subsystems/DrivetrainSubsystem.java
