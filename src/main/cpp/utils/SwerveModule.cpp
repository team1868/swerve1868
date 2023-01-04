#include "utils/SwerveModule.h"

#include <units/math.h>

#include "utils/CTREModuleState.h"
#include "utils/FalconConversion.h"
#include "utils/consts/DRIVETRAIN_CONSTS.h"

SwerveModule::SwerveModule(int moduleNumber, std::string canBusName)
    : _moduleNumber{moduleNumber}, _canBusName{canBusName}
{
    /* Angle Encoder Config */
    ConfigAngleEncoder();

    /* Angle Motor Config */
    ConfigAngleMotor();

    /* Drive Motor Config */
    ConfigDriveMotor();

    _lastAngle = GetState().angle.Degrees();

    _percentOutput    = 0.0;
    _velocity         = 0.0;
    _angle            = 0_deg;
    _absolutePosition = 0.0;
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState desiredState, bool isOpenLoop)
{
    // If the steering motor has reset, redo the zeroing
    if (_angleMotor.HasResetOccurred()) { ResetToAbsolute(); }

    _desiredState = optimize(desiredState, GetState().angle);
    // TODO -- Custom optimize command, since default WPILib optimize assumes continuous controller
    // which CTRE is not

    // TODO redo MAX_SPEED so that it can be adjusted
    if (isOpenLoop) {
        _percentOutput = _desiredState.speed / MAX_SPEED;
        _driveMotor.Set(ControlMode::PercentOutput, _percentOutput);
    } else {
        _velocity = MPSToFalcon(_desiredState.speed, WHEEL_CIRCUMFERENCE, DRIVE_GEAR_RATIO);
        _driveMotor.Set(ControlMode::Velocity,
                        _velocity,
                        DemandType::DemandType_ArbitraryFeedForward,
                        _feedforward.Calculate(_desiredState.speed).value());
    }

    _angle = (math::abs(_desiredState.speed) <= (MAX_SPEED * 0.01)) ? _lastAngle
                                                                    : _desiredState.angle.Degrees();
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    _angleMotor.Set(ControlMode::Position, degreesToFalcon(_angle, ANGLE_GEAR_RATIO));
    _lastAngle = _angle;
}

void SwerveModule::ResetToAbsolute()
{
    _lastAngle        = GetAngleEncoder();
    _absolutePosition = degreesToFalcon(_lastAngle, ANGLE_GEAR_RATIO);
    _angleMotor.SetSelectedSensorPosition(_absolutePosition);
}

void SwerveModule::ConfigAngleEncoder()
{
#ifdef CANENCODER
    _angleEncoder.ConfigFactoryDefault(100);
    _angleEncoder.ConfigAllSettings(_ctreConfigs.swerveCanCoderConfig);
    _angleEncoder.ConfigMagnetOffset(_angleOffset.value());
#else
    // no configurations
#endif
}

void SwerveModule::ConfigAngleMotor()
{
    _angleMotor.ConfigFactoryDefault();
    _angleMotor.ConfigAllSettings(_ctreConfigs.swerveAngleFXConfig);
    _angleMotor.SetInverted(ANGLE_MOTOR_INVERTED);
    _angleMotor.SetNeutralMode(ANGLE_NEUTRAL_MODE);
    // _angleMotor.ConfigStatorCurrentLimit({true, 30, 40, 0.1});
    // _angleMotor.ConfigSupplyCurrentLimit({true, 30, 40, 0.1});
    ResetToAbsolute();
}

void SwerveModule::ConfigDriveMotor()
{
    _driveMotor.ConfigFactoryDefault();
    _driveMotor.ConfigAllSettings(_ctreConfigs.swerveDriveFXConfig);
    _driveMotor.SetInverted(DRIVE_MOTOR_INVERTED);
    _driveMotor.SetNeutralMode(DRIVE_NEUTRAL_MODE);
    _driveMotor.SetSelectedSensorPosition(0);
    // _driveMotor.ConfigStatorCurrentLimit({true, 40, 40, 0.1});
    // _driveMotor.ConfigSupplyCurrentLimit({true, 40, 40, 0.1});
}

degree_t SwerveModule::GetAngleEncoder()
{
#ifdef CANENCODER
    return degree_t{_angleEncoder.GetAbsolutePosition()};
#else
    return degree_t{_angleEncoder.Get()};
#endif
}

frc::SwerveModuleState SwerveModule::GetState()
{
    return frc::SwerveModuleState{
        falconToMPS(_driveMotor.GetSelectedSensorVelocity(), WHEEL_CIRCUMFERENCE, DRIVE_GEAR_RATIO),
        falconToDegrees(_angleMotor.GetSelectedSensorPosition(), ANGLE_GEAR_RATIO)};
}

frc::SwerveModulePosition SwerveModule::GetPosition()
{
    return frc::SwerveModulePosition{
        falconToMeters(
            _driveMotor.GetSelectedSensorPosition(), WHEEL_CIRCUMFERENCE, DRIVE_GEAR_RATIO),
        falconToDegrees(_angleMotor.GetSelectedSensorPosition(), ANGLE_GEAR_RATIO)};
}

#ifdef SIM
void SwerveModule::UpdateSimPeriodic()
{
    // Simulate drive
    _driveSim.SetInput(_driveMotor.GetMotorOutputVoltage() * 1_V);
    // _driveMotor.GetMotorOutputVoltage
    _driveSim.Update(LOOP_PERIOD);
    // _driveSim.GetAngularVelocity()
    double metersPerSecond =
        _driveSim.GetAngularVelocity() * WHEEL_CIRCUMFERENCE / (2.0 * std::numbers::pi);
    // ;
    // _driveMotor.SetSelectedSensorPosition
    // _driveSim.
    _driveMotor.GetSimCollection().SetIntegratedSensorVelocity(
        RPMToFalcon(_driveSim.GetAngularVelocity(), DRIVE_GEAR_RATIO));
    // _driveMotor.SetSelectedSensorVelocity();
    // _driveMotor.setSimSensorVelocity(metersPerSecond, LOOP_PERIOD, DRIVE_GEAR_RATIO);
    _driveMotor.GetSimCollection().SetIntegratedSensorRawPosition();

    // // Simulate steering
    // _steeringSim.SetInput(_steeringMotor.getPhysicalPercentOutput() *
    //                       RobotController.getBatteryVoltage());
    // _steeringSim.Update(LOOP_PERIOD);
    // _steeringMotor.setSimSensorPositionAndVelocity(_steeringSim.getAngleRads(),
    //                                                _steeringSim.getVelocityRadPerSec(),
    //                                                TimedRobot.kDefaultPeriod,
    //                                                _steeringRatio);
}
#endif