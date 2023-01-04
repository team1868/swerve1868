#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

using namespace units;
using namespace units::angle;
using namespace units::angular_velocity;
using namespace units::length;
using namespace units::velocity;

/**
 * @param ticks Falcon tick count
 * @param gearRatio Gear Ratio from Falcon to Mechanism
 * @return Mechanism rotation in degrees
 */
degree_t falconToDegrees(double ticks, double gearRatio);

/**
 * @param degrees Mechanism rotation in degrees
 * @param gearRatio Gear Ratio from the Falcon to the Mechanism
 * @return Falcon ticks
 */
double degreesToFalcon(degree_t degrees, double gearRatio);

/**
 * @param velocityTicks Falcon tick count
 * @param circumference Circumference of the wheel
 * @param gearRatio Gear Ratio from the Falcon to the Mechanism
 * @return Mechanism location in meters
 */
meter_t falconToMeters(double const ticks, meter_t const circumference, double const gearRatio);

/**
 * @param meters Mechanism location in meters
 * @param circumference Circumference of the wheel
 * @param gearRatio Gear Ratio from the Falcon to the Mechanism
 * @return Falcon tick count
 */
double metersToFalcon(meter_t const meters, meter_t const circumference, double const gearRatio);

/**
 * @param velocityTicks Falcon velocity in ticks per 100ms
 * @param gearRatio Gear Ratio from the Falcon to the Mechanism
 * @return echanism velocity in rotations per minute
 */
revolutions_per_minute_t falconToRPM(double velocityTicks, double gearRatio);

/**
 * @param RPM Mechanism velocity in rotations per minute
 * @param gearRatio Gear Ratio from the Falcon to the Mechanism
 * @return Falcon velocity in ticks per 100ms
 */
double RPMToFalcon(revolutions_per_minute_t RPM, double gearRatio);

/**
 * @param velocityTicks Falcon velocity in ticks per 100ms
 * @param circumference Circumference of the wheel
 * @param gearRatio Gear Ratio from the Falcon to the Mechanism
 * @return Velocithy in meters per second
 */
meters_per_second_t falconToMPS(double const velocityTicks,
                                meter_t const circumference,
                                double const gearRatio);

/**
 * @param velocity Velocity in meters per second
 * @param circumference Circumference of the wheel
 * @param gearRatio Gear Ratio from the Falcon to the Mechanism
 * @return Falcon velocity in ticks per 100ms
 */
double MPSToFalcon(meters_per_second_t const velocity,
                   meter_t const circumference,
                   double const gearRatio);
