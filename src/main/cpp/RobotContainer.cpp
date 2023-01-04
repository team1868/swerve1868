#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer()
{
    // Initialize all of your commands and subsystems here

    // Configure the button bindings
    _drivetrain.SetDefaultCommand(_teleopSwerveCommand);

    ConfigureButtonBindings();
}

void RobotContainer::AutonomousInit() { _drivetrain.ResetModulesToAbsolute(); }

void RobotContainer::TeleopInit() { _drivetrain.ResetModulesToAbsolute(); }

void RobotContainer::ConfigureButtonBindings()
{
    // Configure your button bindings here
    _controlboard._aButton.OnTrue(&_zeroGyroCommand);
    _controlboard._bButton.OnTrue(&_ZeroPoseCommand);
}

frc::SwerveDriveOdometry<NUM_MODULES> RobotContainer::GetRobotPose() { return _odometry; }

frc2::Command* RobotContainer::GetAutonomousCommand() { return &_basicTestAuto; }
