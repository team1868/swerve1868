#pragma once

#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>

#include "commands/SwerveAutoPathCommands.h"
#include "commands/TeleopSwerveCommand.h"
#include "subsystems/Controlboard.h"
#include "subsystems/Drivetrain.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
   public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();
    frc::SwerveDriveOdometry<NUM_MODULES> GetRobotPose();

    void AutonomousInit();
    void TeleopInit();

   private:
    // The robot's subsystems and commands are defined here...
    Controlboard _controlboard{};
    Drivetrain _drivetrain{"Canivore1"};

    // TODO confirm this works, may need to be done in the constructor
    frc::SwerveDriveOdometry<NUM_MODULES>& _odometry{_drivetrain.GetOdometry()};

    TeleopSwerveCommand _teleopSwerveCommand{
        _drivetrain, _controlboard, Drivetrain::Mode::SLEWING_FIELD_RELATIVE};
    frc2::InstantCommand _zeroGyroCommand{[&] { _drivetrain.ZeroGyro(); }, {}};
    frc2::InstantCommand _ZeroPoseCommand{[&] { _drivetrain.ZeroPose(); }, {}};
    BasicTestAuto _basicTestAuto{_drivetrain};
    void ConfigureButtonBindings();
};
