#include "commands/TeleopSwerveCommand.h"

#include <cmath>

#include "utils/consts/CONTROL_CONSTS.h"

TeleopSwerveCommand::TeleopSwerveCommand(Drivetrain& drivetrain,
                                         Controlboard& controlboard,
                                         Drivetrain::Mode initialMode)
    : _drivetrain{drivetrain}, _controlboard{controlboard}, _driveMode{initialMode}
{
    AddRequirements({&drivetrain});
}

void TeleopSwerveCommand::Execute()
{
    // this code should probably move, but here before the state machine
    // management is also ok while dpad is pressed, set drive mode to
    // SNAP_TO_ANGLE and give the drivetrain a new target angle
    int drivePov = _controlboard._xboxDrive.GetPOV();
    if (drivePov != -1) {
        _driveMode = Drivetrain::Mode::SNAP_TO_ANGLE;
        // note that the pov axis does not align, we'll need some sort of flat
        // offset here Our functional 0 will be right facing to align with
        // odometry (3 on a standard clock face) POV 0 is "forward" (12 on a
        // standard clock face)
        _drivetrain.SetSnapAngle(degree_t{(double)(-drivePov)});  // odom zero offset
    } else if (_controlboard._xboxDrive.GetXButtonPressed()) {
        if (_driveMode == Drivetrain::Mode::CHASE_STATIC_TARGET) {
            _driveMode = DEFAULT_DRIVE_MODE;
        } else {
            _driveMode = Drivetrain::Mode::CHASE_STATIC_TARGET;
            _drivetrain.SetStaticTarget({0_m, 0_m, 0_deg});
        }
    }

    double xAxis = _controlboard.GetDriveX();
    double yAxis = _controlboard.GetDriveY();
    double rAxis = _controlboard.GetRotX();

    switch (_driveMode) {
        case (Drivetrain::Mode::ROBOT_CENTRIC): break;
        case (Drivetrain::Mode::FIELD_RELATIVE):
        case (Drivetrain::Mode::SLEWING_FIELD_RELATIVE): break;
        case (Drivetrain::Mode::SNAP_TO_ANGLE):
            if (rAxis > STICK_DEADBAND || rAxis < -STICK_DEADBAND) _driveMode = DEFAULT_DRIVE_MODE;
            // As of right now, it's ok to stay in this state until some
            // rotational change happens
            break;
        // case (Drivetrain::Mode::TARGET_RELATIVE):
        //     break;
        case (Drivetrain::Mode::CHASE_STATIC_TARGET):
            if (rAxis > STICK_DEADBAND || rAxis < -STICK_DEADBAND) _driveMode = DEFAULT_DRIVE_MODE;
            break;
        case (Drivetrain::Mode::CHASE_DYNAMIC_TARGET): break;
        default:
            // ERROR -- we should do some combination of throw, log error
            // message, or default behavior
            _driveMode = DEFAULT_DRIVE_MODE;
            break;
    }

    _drivetrain.Drive(xAxis, yAxis, rAxis, _driveMode);
}
