#include "utils/FalconConversion.h"

#include "utils/consts/FALCON_CONSTS.h"

degree_t falconToDegrees(double ticks, double gearRatio)
{
    return ticks * FALCON_TO_DEG / gearRatio;
}

double degreesToFalcon(degree_t degrees, double gearRatio)
{
    return degrees * DEG_TO_FALCON * gearRatio;
}

meter_t falconToMeters(double const ticks, meter_t const circumference, double const gearRatio)
{
    return ticks * FALCON_TICKS_TO_ROT * circumference / gearRatio;
}

double metersToFalcon(meter_t const meters, meter_t const circumference, double const gearRatio)
{
    return meters * ROT_TO_FALCON_TICKS * gearRatio / circumference;
}

revolutions_per_minute_t falconToRPM(double tickVelocity, double gearRatio)
{
    return tickVelocity * FALCON_TO_RPM / gearRatio;
}

double RPMToFalcon(revolutions_per_minute_t rpm, double gearRatio)
{
    return rpm * RPM_TO_FALCON * gearRatio;
}

meters_per_second_t falconToMPS(double tickVelocity, meter_t circumference, double gearRatio)
{
    return tickVelocity * FALCON_TO_RPM * circumference / turn_t{gearRatio};
}

double MPSToFalcon(meters_per_second_t velocity, meter_t circumference, double gearRatio)
{
    return velocity * RPM_TO_FALCON * turn_t{gearRatio} / circumference;
}
