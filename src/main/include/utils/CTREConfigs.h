#pragma once

#include <ctre/Phoenix.h>

#include "utils/confs/DRIVE_MOTOR_CONFS.h"

class CTREConfigs {
   public:
    TalonFXConfiguration swerveAngleFXConfig{};
    TalonFXConfiguration swerveDriveFXConfig{};
    CANCoderConfiguration swerveCanCoderConfig{};

    CTREConfigs()
    {
        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit{ANGLE_ENABLE_CURRENT_LIMIT,
                                                         ANGLE_CONTINUOUS_CURRENT_LIMIT,
                                                         ANGLE_PEAK_CURRENT_LIMIT,
                                                         ANGLE_PEAK_CURRENT_DURATION};
        StatorCurrentLimitConfiguration angleStatorLimit{ANGLE_ENABLE_CURRENT_LIMIT,
                                                         ANGLE_CONTINUOUS_CURRENT_LIMIT,
                                                         ANGLE_PEAK_CURRENT_LIMIT,
                                                         ANGLE_PEAK_CURRENT_DURATION};

        swerveAngleFXConfig.slot0.kP               = ANGLE_PFAC;
        swerveAngleFXConfig.slot0.kI               = ANGLE_IFAC;
        swerveAngleFXConfig.slot0.kD               = ANGLE_DFAC;
        swerveAngleFXConfig.slot0.kF               = ANGLE_FFAC;
        swerveAngleFXConfig.supplyCurrLimit        = angleSupplyLimit;
        swerveAngleFXConfig.statorCurrLimit        = angleStatorLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit{DRIVE_ENABLE_CURRENT_LIMIT,
                                                         DRIVE_CONTINUOUS_CURRENT_LIMIT,
                                                         DRIVE_PEAK_CURRENT_LIMIT,
                                                         DRIVE_PEAK_CURRENT_DURATION};
        StatorCurrentLimitConfiguration driveStatorLimit{DRIVE_ENABLE_CURRENT_LIMIT,
                                                         DRIVE_CONTINUOUS_CURRENT_LIMIT,
                                                         DRIVE_PEAK_CURRENT_LIMIT,
                                                         DRIVE_PEAK_CURRENT_DURATION};

        swerveDriveFXConfig.slot0.kP               = DRIVE_PFAC;
        swerveDriveFXConfig.slot0.kI               = DRIVE_IFAC;
        swerveDriveFXConfig.slot0.kD               = DRIVE_DFAC;
        swerveDriveFXConfig.slot0.kF               = DRIVE_FFAC;
        swerveDriveFXConfig.supplyCurrLimit        = driveSupplyLimit;
        swerveDriveFXConfig.statorCurrLimit        = driveStatorLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy::BootToZero;
        swerveDriveFXConfig.openloopRamp           = OPEN_LOOP_RAMP;
        swerveDriveFXConfig.closedloopRamp         = CLOSED_LOOP_RAMP;

        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange::Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection     = CAN_CODER_INVERTED;
        swerveCanCoderConfig.initializationStrategy =
            SensorInitializationStrategy::BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase::PerSecond;
    }
};
