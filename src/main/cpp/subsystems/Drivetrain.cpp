#include "subsystems/Drivetrain.h"

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

#include "utils/InputUtils.h"
#include "utils/consts/CONTROL_CONSTS.h"
#include "utils/consts/DRIVETRAIN_CONSTS.h"

Drivetrain::Drivetrain(std::string moduleCanBus) : _moduleCanBus{moduleCanBus}
{
    _gyro.ConfigFactoryDefault();
    ZeroGyro();
    ConfigShuffleboard();

    // PID controller and profiled PID controller configuration
    _xProfiledController.SetTolerance(XY_TRANSLATION_TOLERANCE);
    _yProfiledController.SetTolerance(XY_TRANSLATION_TOLERANCE);
    _angleProfiledController.SetTolerance(THETA_TRANSLATION_TOLERANCE);
    _angleProfiledController.EnableContinuousInput(0_deg, 360_deg);

    _xController.SetTolerance(XY_TRANSLATION_TOLERANCE.value());
    _yController.SetTolerance(XY_TRANSLATION_TOLERANCE.value());
    _angleController.SetTolerance(THETA_TRANSLATION_TOLERANCE.value());
    _angleController.EnableContinuousInput(0, 360);
}

void Drivetrain::ConfigShuffleboard()
{
    auto& drivetrainTab = frc::Shuffleboard::GetTab("Drivetrain");

    // This is 2 ways to do the same thing
    _module0Angle = drivetrainTab.Add("Module 0 Val", 0.0).GetEntry();
    _module1Angle = drivetrainTab.Add("Module 1 Val", 0.0).GetEntry();
    _module2Angle = drivetrainTab.Add("Module 2 Val", 0.0).GetEntry();
    _module3Angle = drivetrainTab.Add("Module 3 Val", 0.0).GetEntry();
    drivetrainTab.AddDoubleArray("ModuleAngles ", [&]() {
        for (int i = 0; i < NUM_MODULES; i++) {
            _angles[i] = _modules[i]->GetAngleEncoder().value();
        }
        return _angles;
    });

    _desiredSpeed = drivetrainTab.Add("Desired Speed", 0.0).GetEntry();
    _actualSpeed  = drivetrainTab.Add("Actual Speed", 0.0).GetEntry();

    _desiredAngle = drivetrainTab.Add("Desired Angle", 0.0).GetEntry();
    _actualAngle  = drivetrainTab.Add("Actual Angle", 0.0).GetEntry();
    _errorAngle   = drivetrainTab.Add("Error Angle", 0.0).GetEntry();
    _rotation     = drivetrainTab.Add("Rotation", 0.0).GetEntry();
    _anglePFac    = drivetrainTab.Add("P Fac", 1).GetEntry();
    _angleIFac    = drivetrainTab.Add("I Fac", 0.0).GetEntry();
    _angleDFac    = drivetrainTab.Add("D Fac", 0.0).GetEntry();

    frc::SmartDashboard::PutData("Field", &_field);
}

void Drivetrain::UpdateShuffleboard()
{
    // for (SwerveModule* module : _modules) {
    //     frc::SmartDashboard::PutNumber(module->encancoderLogString,
    //                                    module->GetAngleEncoder().Degrees().value());
    //     frc::SmartDashboard::PutNumber(
    //         module->integratedLogString,
    //         module->GetState().angle.Degrees().value());
    //     frc::SmartDashboard::PutNumber(module->velocityLogString,
    //                                    module->GetState().speed.value());
    // }

    _module0Angle->SetDouble(_module0.GetAngleEncoder().value());
    _module1Angle->SetDouble(_module1.GetAngleEncoder().value());
    _module2Angle->SetDouble(_module2.GetAngleEncoder().value());
    _module3Angle->SetDouble(_module3.GetAngleEncoder().value());

    _field.SetRobotPose(_swerveOdometry.GetPose());
    // TODO _angleSetpoint.GetPositionError()

    _actualAngle->SetDouble(ToAbsoluteAngle(GetYaw()));
    _errorAngle->SetDouble(_angleProfiledController.GetPositionError().value());

    // _angleProfiledController.SetPID(_anglePFac.GetDouble(1),
    //                         _angleIFac.GetDouble(0.0),
    //                         _angleDFac.GetDouble(0.0));
    // _field.GetObject("Odometry")->SetPose(_swerveOdometry.GetPose());
    // _field.GetObject("Vision Pose")->SetPose(_visionPose);
}

frc::SwerveDriveOdometry<NUM_MODULES>& Drivetrain::GetOdometry() { return _swerveOdometry; }

void Drivetrain::SensorUpdateOdometry()
{
    _swerveOdometry.Update(GetYaw(),
                           {_module0.GetPosition(),
                            _module1.GetPosition(),
                            _module2.GetPosition(),
                            _module3.GetPosition()});
}

void Drivetrain::Periodic()
{
    // _camera0.UpdateVision() ? SetPose(_camera0.UpdateOdomVision(GetYaw())) :
    // SensorUpdateOdometry();
    SensorUpdateOdometry();
    UpdateShuffleboard();
}

void Drivetrain::SimulationPeriodic()
{
    // Implementation of subsystem simulation periodic method goes here.
}

frc::Pose2d Drivetrain::GetPose() { return _swerveOdometry.GetPose(); }

void Drivetrain::ZeroGyro()
{
    _gyro.SetYaw(GYRO_ZERO.value());
    SetPose(_swerveOdometry.GetPose());
}

void Drivetrain::ZeroPose()
{
    _swerveOdometry.ResetPosition(GetYaw(),
                                  {_module0.GetPosition(),
                                   _module1.GetPosition(),
                                   _module2.GetPosition(),
                                   _module3.GetPosition()},
                                  {0_m, 0_m, {0_deg}});
}

void Drivetrain::SetPose(frc::Pose2d pose)
{
    _swerveOdometry.ResetPosition(GetYaw(),
                                  {_module0.GetPosition(),
                                   _module1.GetPosition(),
                                   _module2.GetPosition(),
                                   _module3.GetPosition()},
                                  pose);
}

void Drivetrain::SetGyroAndPose(degree_t yaw, frc::Pose2d pose)
{
    _gyro.SetYaw(yaw.value());
    SetPose(pose);
}

frc::Rotation2d Drivetrain::GetYaw()
{
    return INVERT_GYRO ? degree_t{360 - _gyro.GetYaw()} : degree_t{_gyro.GetYaw()};
}

// Drive is responsible dispatching core drive functions and converting raw driver input
void Drivetrain::Drive(double joystickX, double joystickY, double joystickTheta, Mode mode)
{
    switch (mode) {
        case ROBOT_CENTRIC:
            RobotCentricDrive(ScaleJoystickX(joystickX),
                              ScaleJoystickY(joystickY),
                              ScaleJoystickTheta(joystickTheta));
            break;
        case FIELD_RELATIVE:
            FieldRelativeDrive(ScaleJoystickX(joystickX),
                               ScaleJoystickY(joystickY),
                               ScaleJoystickTheta(joystickTheta));
            break;
        case SNAP_TO_ANGLE:
            SnapToAngleDrive(ScaleJoystickX(joystickX), ScaleJoystickY(joystickY));
            break;
        case TARGET_RELATIVE:
            TargetCentricDrive(ScaleJoystickX(joystickX),
                               ScaleJoystickY(joystickY),
                               ScaleJoystickTheta(joystickTheta));
            break;
        case CHASE_STATIC_TARGET: ChaseStaticTargetDrive(); break;
        case CHASE_DYNAMIC_TARGET:
            ChaseDynamicTargetDrive(FIELD_TO_TARGET, TARGET_RELATIVE_POSE, true);
            // ChaseDynamicTargetDrive(FIELD_TO_TARGET, TARGET_RELATIVE_POSE, _hasTarget);
            break;
        case SLEWING_FIELD_RELATIVE:
            SlewingFieldRelativeDrive(ScaleJoystickX(joystickX),
                                      ScaleJoystickY(joystickY),
                                      ScaleJoystickTheta(joystickTheta));
            break;
        default:
            // ERROR
            // throw, error message, or default behavior
            // return;
            break;
    }

    _actualSpeed->SetDouble(_module0.GetState().speed());
}

void Drivetrain::RobotCentricDrive(meters_per_second_t translationX,
                                   meters_per_second_t translationY,
                                   degrees_per_second_t rotation)
{
    auto goalModuleStates = _swerveKinematics.ToSwerveModuleStates(
        frc::ChassisSpeeds{translationX, translationY, rotation});

    frc::SwerveDriveKinematics<NUM_MODULES>::DesaturateWheelSpeeds(&goalModuleStates, MAX_SPEED);

    for (SwerveModule* module : _modules) {
        module->SetDesiredState(goalModuleStates[module->_moduleNumber],
                                IS_OPEN_LOOP);  // open loop
    }
    _desiredSpeed->SetDouble(goalModuleStates[0].speed());
}

void Drivetrain::FieldRelativeDrive(meters_per_second_t translationX,
                                    meters_per_second_t translationY,
                                    degrees_per_second_t rotation)
{
    auto goalModuleStates =
        _swerveKinematics.ToSwerveModuleStates(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            translationX, translationY, rotation, GetYaw()));

    frc::SwerveDriveKinematics<NUM_MODULES>::DesaturateWheelSpeeds(&goalModuleStates, MAX_SPEED);

    for (SwerveModule* module : _modules) {
        module->SetDesiredState(goalModuleStates[module->_moduleNumber], IS_OPEN_LOOP);
    }
    _desiredSpeed->SetDouble(goalModuleStates[0].speed());
}

void Drivetrain::SnapToAngleDrive(meters_per_second_t translationX,
                                  meters_per_second_t translationY)
{
    // TODO fix pid so you don't need to scale
    // TODO Replace with slewing drive (?)
    // std::cout << _angleController.Calculate(GetYaw().Degrees().value()) << std::endl;

    FieldRelativeDrive(
        translationX,
        translationY,
        // calculate snap to angle rotation here
        _angleController.AtSetpoint()
            ? 0_deg_per_s
            : degrees_per_second_t{_angleController.Calculate(GetYaw().Degrees().value())});
}

void Drivetrain::TargetCentricDrive(meters_per_second_t translationX,
                                    meters_per_second_t translationY,
                                    degrees_per_second_t rotation)
{
    return;
}

void Drivetrain::ChaseStaticTargetDrive()
{
    auto currentPose = GetPose();

    // TODO Replace with slewing drive (?)
    // TODO AtSetpoint logic is flawed (intially will return true rather than forcing an initial
    // false eval) logic needs to be redone around it
    FieldRelativeDrive(
        // _xProfiledController.AtGoal()
        // _xController.AtSetpoint()
        //     ? 0_mps
        //     // : meters_per_second_t{_xProfiledController.Calculate(currentPose.X())},
        //     : meters_per_second_t{_xController.Calculate(currentPose.X().value())},
        meters_per_second_t{_xController.Calculate(currentPose.X().value())},
        // _yProfiledController.AtGoal()
        // _yController.AtSetpoint()
        //     ? 0_mps
        //     // : meters_per_second_t{_yProfiledController.Calculate(currentPose.Y())},
        //     : meters_per_second_t{_yController.Calculate(currentPose.Y().value())},
        meters_per_second_t{_yController.Calculate(currentPose.Y().value())},
        // _angleProfiledController.AtGoal()
        // _angleController.AtSetpoint()
        //     ? 0_deg_per_s
        //     : degrees_per_second_t{_angleController.Calculate(GetYaw().Degrees().value())});
        degrees_per_second_t{_angleController.Calculate(GetYaw().Degrees().value())});
    // : degrees_per_second_t{_angleProfiledController.Calculate(GetYaw().Degrees())});
}

void Drivetrain::SetStaticTarget(frc::Pose2d goalPose)
{
    _xProfiledController.SetGoal(goalPose.X());
    _yProfiledController.SetGoal(goalPose.Y());
    _angleProfiledController.SetGoal(goalPose.Rotation().Degrees());

    _xController.Reset();
    _yController.Reset();
    _angleController.Reset();
    _xController.SetSetpoint(goalPose.X().value());
    _yController.SetSetpoint(goalPose.Y().value());
    _angleController.SetSetpoint(goalPose.Rotation().Degrees().value());
}

void Drivetrain::ChaseDynamicTargetDrive(frc::Pose2d visionTarget,
                                         frc::Pose2d& relativeGoal,
                                         bool changeTarget)
{
    // If the target has moved, update the goal pose without resetting controllers
    if (changeTarget) {
        auto goalPose = visionTarget - relativeGoal;
        _xProfiledController.SetGoal(goalPose.X());
        _yProfiledController.SetGoal(goalPose.Y());
        _angleProfiledController.SetGoal(goalPose.Rotation().Degrees());

        _xController.SetSetpoint(goalPose.X().value());
        _yController.SetSetpoint(goalPose.Y().value());
        _angleController.SetSetpoint(goalPose.Rotation().Degrees().value());
    }
    ChaseStaticTargetDrive();
}

// TODO cleanup, use more efficient calculation in SetSnapAngle either eliminates or moves this
// subroutine
double Drivetrain::ToAbsoluteAngle(frc::Rotation2d angle)
{
    return ToAbsoluteAngle(angle.Degrees());
}
double Drivetrain::ToAbsoluteAngle(degree_t angle) { return ToAbsoluteAngle(angle.value()); }
double Drivetrain::ToAbsoluteAngle(double angle)
{
    double scaled = fmod(angle, 360);
    return scaled + (scaled < 0 ? 360 : 0);
}

void Drivetrain::SetSnapAngle(degree_t angle)
{
    // // setpoint should be +/- 180 degrees from the current orientation
    // // Get gyro and desired absolute angle [0, 360]
    // double currentAbsoluteAngle = ToAbsoluteAngle(GetYaw());
    // double desiredAbsoluteAngle = ToAbsoluteAngle(angle);
    // // [0, 360] - [0-360] --> [-360, 360] --> [-180, 180]
    // double initialError = desiredAbsoluteAngle - currentAbsoluteAngle;
    // initialError += (initialError > 180) ? -360 : ((initialError < -180) ? 360 : 0);
    // // Calculate actual setpoint relative to current yaw and configure
    // degree_t angleSetpoint = GetYaw().Degrees() + degree_t{initialError};

    // /* alternatively
    //  unknown --> [-360, 360] --> [-180, 180]
    // double originalError = angle.value() - _gyro.GetYaw();
    // double absoluteError = fmod(originalError, 360);
    // auto angleSetpoint = degree_t{_gyro.GetYaw() + absoluteError +
    // (absoluteError > 180) ? -360 : ((absoluteError < -180) ? 360 : 0)};
    // */

    _angleController.SetSetpoint(angle.value());
    // _angleController.SetSetpoint(angleSetpoint.value());
    // _angleProfiledController.
    // _angleProfiledController.
    _desiredAngle->SetDouble(angle.value());
}

void Drivetrain::SlewingFieldRelativeDrive(meters_per_second_t translationX,
                                           meters_per_second_t translationY,
                                           degrees_per_second_t rotation)
{
    FieldRelativeDrive(_xSlewRateFilter.Calculate(translationX),
                       _ySlewRateFilter.Calculate(translationY),
                       _angleSlewRateFilter.Calculate(rotation));
}

/* Used by SwerveControllerCommand in Auto */
void Drivetrain::SetModuleStates(wpi::array<frc::SwerveModuleState, NUM_MODULES> desiredStates)
{
    frc::SwerveDriveKinematics<NUM_MODULES>::DesaturateWheelSpeeds(&desiredStates, MAX_SPEED);

    for (SwerveModule* module : _modules) {
        module->SetDesiredState(desiredStates[module->_moduleNumber], IS_OPEN_LOOP);
    }
}

void Drivetrain::ResetModulesToAbsolute()
{
    for (SwerveModule* module : _modules) {
        module->ResetToAbsolute();
    }
}

frc::SwerveDriveKinematics<NUM_MODULES>& Drivetrain::GetSwerveKinematics()
{
    return _swerveKinematics;
}
