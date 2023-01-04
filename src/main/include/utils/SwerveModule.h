#pragma once

#include <ctre/Phoenix.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/SingleJointedArmSim.h>

#include <cmath>
#include <string>

#include "utils/CTREConfigs.h"
#include "utils/confs/MODULE_CONFS.h"
#include "utils/ports/DRIVETRAIN_PORTS.h"

class SwerveModule {
   public:
    SwerveModule(int moduleNumber, std::string canBusName = "");
    void SetDesiredState(frc::SwerveModuleState desiredState, bool isOpenLoop);
    degree_t GetAngleEncoder();
    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();

    int const _moduleNumber;
    std::string const _canBusName;

    std::string const _moduleNumberStr    = std::to_string(_moduleNumber);
    std::string const encancoderLogString = "Mod " + _moduleNumberStr + " Cancoder";
    std::string const integratedLogString = "Mod " + _moduleNumberStr + " Integrated";
    std::string const velocityLogString   = "Mod " + _moduleNumberStr + " Velocity";
    void ResetToAbsolute();

   private:
    void ConfigAngleEncoder();
    void ConfigAngleMotor();
    void ConfigDriveMotor();

    CTREConfigs _ctreConfigs;

    frc::SimpleMotorFeedforward<meters> _feedforward{DRIVE_KS, DRIVE_KV, DRIVE_KA};

    degree_t const _angleOffset = DEGREE_OFFSETS[_moduleNumber];

#ifdef CANENCODER
    CANCoder _angleEncoder{ENCODER_IDS[_moduleNumber], _canBusName};
#else
    // TODO test using offset rather than 0?
    // TODO test sample averaging (500khz/analog ports)
    frc::AnalogPotentiometer _angleEncoder{ENCODER_IDS[_moduleNumber], 360, _angleOffset};
#endif

    TalonFX _angleMotor{ANGLE_MOTOR_IDS[_moduleNumber], _canBusName};
    TalonFX _driveMotor{DRIVE_MOTOR_IDS[_moduleNumber], _canBusName};

    degree_t _lastAngle;
    meters_per_second_t _curVel;
    frc::Rotation2d _curAngle;
    frc::SwerveModuleState _desiredState;

    double _percentOutput;
    double _velocity;
    degree_t _angle;
    double _absolutePosition;

#ifdef SIM
   public:
    void UpdateSimPeriodic();

   private:
    frc::sim::FlywheelSim _driveSim{frc::DCMotor::Falcon500(1),
                                    DRIVE_GEAR_RATIO,
                                    units::kilogram_square_meter_t{0.001},
                                    {2.0 * std::numbers::pi / 2048}};

    frc::sim::SingleJointedArmSim _steerSim{
        frc::DCMotor::Falcon500(1),
        ANGLE_GEAR_RATIO,
        units::kilogram_square_meter_t{0.001},  // MOI
        0.0_m,                                  // Length (m)
        units::radian_t{-HUGE_VAL},             // Min angle
        units::radian_t{HUGE_VAL},              // Max angle
        10.0_kg,                                // Mass (kg)
        false,                                  // Simulate gravity
        {2.0 * std::numbers::pi / 2048}         // Add noise with a std-dev of 1 tick
    };
#endif
};
