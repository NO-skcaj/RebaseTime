#include "subsystems/Vision.hpp"
#include <frc/DriverStation.h>

#include <thread>

Vision::Vision(camera_type cam)
{
    switch (cam)
    {
        case LIMELIGHT:
            bool doRejectUpdate = false;

            LimelightHelpers::SetRobotOrientation("limelight", Gettin_The_ULTIMATE_POSE_ESTIMATOR->Ulitmate_Pose_Estimation.GetEstimatedPosition().Rotation().Degrees(), 0, 0, 0, 0, 0);

            LimelightHelpers::PoseEstimate mt2;

            if (frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue) 
            {
                mt2 = bmt2;
            } else 
            {
                mt2 = rmt2;
            }

            if(std::abs(Gettin_The_ULTIMATE_POSE_ESTIMATOR->navx.GetRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            {
                doRejectUpdate = true;
            }
            if(mt2.tagCount == 0)
            {
                doRejectUpdate = true;
            }
            if(!doRejectUpdate)
            {
                Gettin_The_ULTIMATE_POSE_ESTIMATOR->Ulitmate_Pose_Estimation.SetVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                Gettin_The_ULTIMATE_POSE_ESTIMATOR->Ulitmate_Pose_Estimation.AddVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds
                );
                this->currentPose = mt2.pose;
            }
        case GENERIC_USB:
            std::thread VisionThread(&Vision::VisionOnAnotherThread);
            VisionThread.detach();
    }
}