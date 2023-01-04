#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Drivetrain.h"

class BasicTestAuto : public frc2::CommandHelper<frc2::SequentialCommandGroup, BasicTestAuto> {
   public:
    BasicTestAuto(Drivetrain& drivetrain);

    Drivetrain& _drivetrain;
};
