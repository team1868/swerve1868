#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit()
{
    _container.AutonomousInit();

    _autonomousCommand = _container.GetAutonomousCommand();

    if (_autonomousCommand != nullptr) { _autonomousCommand->Schedule(); }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    _container.TeleopInit();

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (_autonomousCommand != nullptr) {
        _autonomousCommand->Cancel();
        _autonomousCommand = nullptr;
    }
}

void Robot::TeleopPeriodic() {}

void Robot::TestInit()
{
    // Cancels all running commands at the start of test mode.
    frc2::CommandScheduler::GetInstance().CancelAll();
}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
