#include "commands/SwerveAutoPathCommands.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <vector>

using namespace units;
using namespace units::angle;

namespace {
frc2::SwerveControllerCommand<NUM_MODULES> SwervePathLegCommandGenerator(
    Drivetrain& drivetrain,
    frc::Pose2d const& start,
    std::vector<frc::Translation2d> const& waypoints,
    frc::Pose2d const& end)
{
    frc::TrajectoryConfig config(XY_TRANSLATION_MAX_V, XY_TRANSLATION_MAX_A);
    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(drivetrain.GetSwerveKinematics());
    // TODO Apply the voltage or current constraint
    // config.AddConstraint(autoVoltageConstraint);
    // config.AddConstraint(centripetalConstraint);
    return frc2::SwerveControllerCommand<NUM_MODULES>{
        frc::TrajectoryGenerator::GenerateTrajectory(start, waypoints, end, config),
        [&]() { return drivetrain.GetPose(); },
        drivetrain.GetSwerveKinematics(),
        frc2::PIDController{XY_TRANSLATION_PFAC, XY_TRANSLATION_IFAC, XY_TRANSLATION_DFAC},
        frc2::PIDController{XY_TRANSLATION_PFAC, XY_TRANSLATION_IFAC, XY_TRANSLATION_DFAC},
        frc::ProfiledPIDController<radians>{THETA_TRANSLATION_PFAC,
                                            THETA_TRANSLATION_IFAC,
                                            THETA_TRANSLATION_DFAC,
                                            frc::TrapezoidProfile<radians>::Constraints{
                                                THETA_TRANSLATION_MAX_V, THETA_TRANSLATION_MAX_A}},
        [&](auto moduleStates) { drivetrain.SetModuleStates(moduleStates); },
        {&drivetrain}};
}

frc2::InstantCommand SetOdometryCommand(Drivetrain& drivetrain, frc::Pose2d pose)
{
    return {[&] { drivetrain.SetGyroAndPose(pose.Rotation().Degrees(), pose); }, {}};
}
}  // namespace

BasicTestAuto::BasicTestAuto(Drivetrain& drivetrain) : _drivetrain{drivetrain}
{
    auto initialPose = frc::Pose2d{0_m, 0_m, 0_deg};
    AddCommands(
        SetOdometryCommand(drivetrain, initialPose),
        SwervePathLegCommandGenerator(drivetrain,
                                      initialPose,  // swap w/ getting pose from drivetrain later
                                      {frc::Translation2d(0_m, 1_m), frc::Translation2d(1_m, 1_m)},
                                      frc::Pose2d(2_m, 1_m, 0_deg)));
}
