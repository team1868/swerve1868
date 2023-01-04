#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/GenericEntry.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>

#include "utils/confs/SENSOR_CONFS.h"
#include "utils/confs/VISION_CONFS.h"

#include <units/angle.h>
#include <units/length.h>

using namespace units::length;
using namespace units::angle;

class VisionController {
   public:
    VisionController();
    frc::Pose2d GetVisionPose(frc::Rotation2d& robotYaw,
                              degree_t& targetPitch,
                              degree_t& targetYaw);
    bool UpdateVision();

    void SetDriverMode();
    frc::Pose2d UpdateOdomVision(frc::Rotation2d robotYaw);
    void SetTapeMode();

   private:
    nt::GenericEntry* _photonDistanceEntry;
    nt::GenericEntry* _photonYawEntry;
    nt::GenericEntry* _photonPitchEntry;
    frc::Pose2d _visionPose;

    // TODO eliminate
    frc::Rotation2d _gyroZeroRotation{GYRO_ZERO};

    bool _hasTarget;
    photonlib::PhotonCamera _photonVision{"gloworm"};
    photonlib::PhotonPipelineResult _photonResult;
    photonlib::PhotonTrackedTarget _target;
    frc::Transform3d _cameraToTarget;
    second_t _prevTimestamp;
    const frc::Transform2d CAMERA_TO_ROBOT{CAMERA_POSE, ROBOT_ORIGIN};
    // degree_t _photonPitch;
    // degree_t _photonYaw;
};
