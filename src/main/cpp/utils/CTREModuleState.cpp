#include "utils/CTREModuleState.h"

#include <units/math.h>

/**
 * @param scopeReference Current Angle
 * @param newAngle Target Angle
 * @return Closest angle within scope
 */
degree_t placeInAppropriate0To360Scope(degree_t scopeReference, degree_t newAngle)
{
    degree_t lowerBound;
    degree_t upperBound;
    degree_t lowerOffset = units::math::fmod(scopeReference, 360_deg);

    if (lowerOffset >= 0_deg) {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (360_deg - lowerOffset);
    } else {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - (360_deg + lowerOffset);
    }
    while (newAngle < lowerBound) {
        newAngle += 360_deg;
    }
    while (newAngle > upperBound) {
        newAngle -= 360_deg;
    }
    if (newAngle - scopeReference > 180_deg) {
        newAngle -= 360_deg;
    } else if (newAngle - scopeReference < -180_deg) {
        newAngle += 360_deg;
    }
    return newAngle;
}
/**
 * Minimize the change in heading the desired swerve module state would
 * require by potentially reversing the direction the wheel spins.
 * Customized from WPILib's version to include placing in appropriate scope
 * for CTRE onboard control.
 *
 * @param desiredState The desired state.
 * @param currentAngle The current module angle.
 */
frc::SwerveModuleState optimize(frc::SwerveModuleState desiredState, frc::Rotation2d currentAngle)
{
    degree_t targetAngle =
        placeInAppropriate0To360Scope(currentAngle.Degrees(), desiredState.angle.Degrees());
    meters_per_second_t targetSpeed = desiredState.speed;
    degree_t delta                  = targetAngle - currentAngle.Degrees();
    if (math::abs(delta) > 90_deg) {
        targetSpeed = -targetSpeed;
        targetAngle += delta > 90_deg ? -180_deg : 180_deg;
    }
    return frc::SwerveModuleState{targetSpeed, targetAngle};
}
