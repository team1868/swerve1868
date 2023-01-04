#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Controlboard.h"
#include "subsystems/Drivetrain.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TeleopSwerveCommand : public frc2::CommandHelper<frc2::CommandBase, TeleopSwerveCommand> {
   public:
    explicit TeleopSwerveCommand(Drivetrain& subsystem,
                                 Controlboard& controlboard,
                                 Drivetrain::Mode initialMode);

    void Execute() override;

   private:
    Drivetrain& _drivetrain;
    Controlboard& _controlboard;

    Drivetrain::Mode _driveMode;
    static const Drivetrain::Mode DEFAULT_DRIVE_MODE = Drivetrain::Mode::FIELD_RELATIVE;
};
