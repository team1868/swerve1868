#include "subsystems/VisionController.h"

#include "utils/consts/VISION_CONSTS.h"

VisionController::VisionController()
{
    auto& drivetrainTab  = frc::Shuffleboard::GetTab("Drivetrain");
    _photonDistanceEntry = drivetrainTab.Add("Photon Distance", 0.0).GetEntry();
    _photonYawEntry      = drivetrainTab.Add("Photon Yaw", 0.0).GetEntry();
    _photonPitchEntry    = drivetrainTab.Add("Photon Pitch", 0.0).GetEntry();
}

frc::Pose2d VisionController::GetVisionPose(frc::Rotation2d& robotYaw,
                                            degree_t& targetPitch,
                                            degree_t& targetYaw)
{
    return photonlib::PhotonUtils::EstimateFieldToRobot(CAMERA_HEIGHT,
                                                        TARGET_HEIGHT,
                                                        CAMERA_PITCH,
                                                        targetPitch,
                                                        -targetYaw,
                                                        robotYaw - _gyroZeroRotation,
                                                        FIELD_TO_TARGET,
                                                        CAMERA_TO_ROBOT);
}

void VisionController::SetDriverMode()
{
    _photonVision.SetDriverMode(true);
    _photonVision.SetLEDMode(photonlib::LEDMode::kOff);
    _photonVision.SetPipelineIndex(DRIVE_PIPELINE_INDEX);
}

void VisionController::SetTapeMode()
{
    _photonVision.SetDriverMode(false);
    _photonVision.SetLEDMode(photonlib::LEDMode::kOn);
    _photonVision.SetPipelineIndex(TAPE_PIPELINE_INDEX);
}

frc::Pose2d VisionController::UpdateOdomVision(frc::Rotation2d robotYaw)
{
    _target          = _photonResult.GetBestTarget();
    auto photonPitch = degree_t{_target.GetPitch()};
    auto photonYaw   = degree_t{_target.GetYaw()};
    _photonYawEntry->SetDouble(photonYaw.value());
    _photonPitchEntry->SetDouble(photonPitch.value());

    // this should get eliminated, use pose to calculate the distance instead
    auto photonDistance = photonlib::PhotonUtils::CalculateDistanceToTarget(
        CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH, photonPitch);
    _photonDistanceEntry->SetDouble(photonDistance.value());

    _visionPose = GetVisionPose(robotYaw, photonPitch, photonYaw);
    return _visionPose;
}

// returns true if there's a new image with a target in it
bool VisionController::UpdateVision()
{
    _photonResult = _photonVision.GetLatestResult();
    auto latest   = _photonResult.GetTimestamp();
    if (latest != _prevTimestamp) {
        _hasTarget     = _photonResult.HasTargets();
        _prevTimestamp = latest;

        return _hasTarget;
    }
    return false;
}
