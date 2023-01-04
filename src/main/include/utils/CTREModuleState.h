#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>

using namespace units;
using namespace units::angle;

/**
 * @param scopeReference Current Angle
 * @param newAngle Target Angle
 * @return Closest angle within scope
 */
degree_t placeInAppropriate0To360Scope(degree_t scopeReference, degree_t newAngle);

/**
 * Minimize the change in heading the desired swerve module state would
 * require by potentially reversing the direction the wheel spins.
 * Customized from WPILib's version to include placing in appropriate scope
 * for CTRE onboard control.
 *
 * @param desiredState The desired state.
 * @param currentAngle The current module angle.
 */
frc::SwerveModuleState optimize(frc::SwerveModuleState desiredState, frc::Rotation2d currentAngle);
