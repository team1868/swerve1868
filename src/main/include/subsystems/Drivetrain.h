#pragma once

#include <ctre/phoenix/sensors/Pigeon2.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/GenericEntry.h>

#include <string>

#include "VisionController.h"
#include "utils/SwerveModule.h"
#include "utils/confs/DRIVETRAIN_CONFS.h"
#include "utils/confs/SENSOR_CONFS.h"

class Drivetrain : public frc2::SubsystemBase {
   public:
    Drivetrain(std::string moduleCanBus = "");

    void ConfigShuffleboard();
    void UpdateShuffleboard();

    void Periodic() override;
    void SimulationPeriodic() override;

    frc::Rotation2d GetYaw();
    frc::Pose2d GetPose();

    void SensorUpdateOdometry();
    frc::SwerveDriveOdometry<NUM_MODULES>& GetOdometry();
    frc::SwerveDriveKinematics<NUM_MODULES>& GetSwerveKinematics();

    void ResetModulesToAbsolute();
    void ZeroGyro();
    void ZeroPose();
    void SetPose(frc::Pose2d pose);
    void SetGyroAndPose(degree_t yaw, frc::Pose2d pose);

    /* Used by SwerveControllerCommand in Auto */
    void SetModuleStates(wpi::array<frc::SwerveModuleState, NUM_MODULES> desiredStates);

    /**
     * Drive acts like a dispatch to different swerve functionalities based off
     *of mode. Default mode is field relative.
     **/
    enum Mode {
        UNKNOWN,
        ROBOT_CENTRIC,
        FIELD_RELATIVE,
        SNAP_TO_ANGLE,
        TARGET_RELATIVE,
        CHASE_STATIC_TARGET,
        CHASE_DYNAMIC_TARGET,
        SLEWING_FIELD_RELATIVE
    };

    void Drive(double joystickX,
               double joystickY,
               double joystickTheta,
               Mode mode = FIELD_RELATIVE);

    // individual drive functions
    void RobotCentricDrive(meters_per_second_t translationX,
                           meters_per_second_t translationY,
                           degrees_per_second_t rotation);
    void FieldRelativeDrive(meters_per_second_t translationX,
                            meters_per_second_t translationY,
                            degrees_per_second_t rotation);
    void SlewingFieldRelativeDrive(meters_per_second_t translationX,
                                   meters_per_second_t translationY,
                                   degrees_per_second_t rotation);
    void SnapToAngleDrive(meters_per_second_t translationX, meters_per_second_t translationY);
    void TargetCentricDrive(meters_per_second_t translationX,
                            meters_per_second_t translationY,
                            degrees_per_second_t rotation);
    void ChaseStaticTargetDrive();
    void ChaseDynamicTargetDrive(frc::Pose2d visionTarget,
                                 frc::Pose2d& relativeGoal,
                                 bool targetMoved);

    void SetSnapAngle(degree_t angle);
    void SetStaticTarget(frc::Pose2d goalPose);
    double ToAbsoluteAngle(frc::Rotation2d angle);
    double ToAbsoluteAngle(degree_t angle);
    double ToAbsoluteAngle(double angle);

   private:
    frc::SwerveDriveKinematics<NUM_MODULES> _swerveKinematics{
        frc::Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        frc::Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
        frc::Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
        frc::Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)};

    std::string const _moduleCanBus;
    SwerveModule _module0{0, _moduleCanBus};
    SwerveModule _module1{1, _moduleCanBus};
    SwerveModule _module2{2, _moduleCanBus};
    SwerveModule _module3{3, _moduleCanBus};

    frc::SwerveDriveOdometry<NUM_MODULES> _swerveOdometry{
        _swerveKinematics,
        frc::Rotation2d{INVERT_GYRO ? 360_deg : 0_deg},
        {_module0.GetPosition(),
         _module1.GetPosition(),
         _module2.GetPosition(),
         _module3.GetPosition()}};

    std::vector<SwerveModule*> _modules{&_module0, &_module1, &_module2, &_module3};

    ctre::phoenix::sensors::WPI_Pigeon2 _gyro{PIGEON_ID, _moduleCanBus};

    VisionController _camera0{};

    nt::GenericEntry* _module0Angle;
    nt::GenericEntry* _module1Angle;
    nt::GenericEntry* _module2Angle;
    nt::GenericEntry* _module3Angle;

    nt::GenericEntry* _desiredSpeed;
    nt::GenericEntry* _actualSpeed;
    nt::GenericEntry* _desiredAngle;
    nt::GenericEntry* _actualAngle;
    nt::GenericEntry* _errorAngle;
    nt::GenericEntry* _rotation;
    nt::GenericEntry* _anglePFac;
    nt::GenericEntry* _angleIFac;
    nt::GenericEntry* _angleDFac;

    frc::Field2d _field{};

    std::vector<double> _angles{NUM_MODULES};

    /* Chase resourcers */
    // TODO tune PID values
    // TODO merge SnapToAngleDrive and Chase controllers
    // TODO Tune PID values and trapezoidal constraints
    frc::Pose2d TARGET_RELATIVE_POSE{0_m, 1_m, 0_deg};
    frc::ProfiledPIDController<units::meters> _xProfiledController{
        XY_TRANSLATION_PFAC,
        XY_TRANSLATION_IFAC,
        XY_TRANSLATION_DFAC,
        frc::TrapezoidProfile<units::meters>::Constraints{XY_TRANSLATION_MAX_V,
                                                          XY_TRANSLATION_MAX_A}};
    frc::ProfiledPIDController<units::meters> _yProfiledController{
        XY_TRANSLATION_PFAC,
        XY_TRANSLATION_IFAC,
        XY_TRANSLATION_DFAC,
        frc::TrapezoidProfile<units::meters>::Constraints{XY_TRANSLATION_MAX_V,
                                                          XY_TRANSLATION_MAX_A}};
    frc::ProfiledPIDController<units::degrees> _angleProfiledController{
        THETA_TRANSLATION_PFAC,
        THETA_TRANSLATION_IFAC,
        THETA_TRANSLATION_DFAC,
        frc::TrapezoidProfile<units::degrees>::Constraints{THETA_TRANSLATION_MAX_V,
                                                           THETA_TRANSLATION_MAX_A}};

    frc2::PIDController _xController{XY_TRANSLATION_PFAC, XY_TRANSLATION_IFAC, XY_TRANSLATION_DFAC};

    frc2::PIDController _yController{XY_TRANSLATION_PFAC, XY_TRANSLATION_IFAC, XY_TRANSLATION_DFAC};

    frc2::PIDController _angleController{
        THETA_TRANSLATION_PFAC, THETA_TRANSLATION_IFAC, THETA_TRANSLATION_DFAC};

    /* Slew limiter drive resourcers */
    frc::SlewRateLimiter<units::meters_per_second> _xSlewRateFilter{
        LINEAR_SLEW_RATE, -LINEAR_SLEW_RATE, 0_mps};
    frc::SlewRateLimiter<units::meters_per_second> _ySlewRateFilter{
        LINEAR_SLEW_RATE, -LINEAR_SLEW_RATE, 0_mps};
    frc::SlewRateLimiter<units::degrees_per_second> _angleSlewRateFilter{
        ANGULAR_SLEW_RATE, -ANGULAR_SLEW_RATE, 0_deg_per_s};
};
