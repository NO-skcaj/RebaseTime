// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Swerve.hpp"

#include <frc/Timer.h>

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative,
                       units::second_t period) 
{
  auto states = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_poseEstimator.GetEstimatedPosition().Rotation())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
      period
  ));

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft->SetDesiredState(fl);
  m_frontRight->SetDesiredState(fr);
  m_backLeft->SetDesiredState(bl);
  m_backRight->SetDesiredState(br);
}

void Drivetrain::UpdateOdometry() {
  m_poseEstimator.Update(this->Gyro->getRotation2d(),
                         {m_frontLeft->GetModulePosition(), m_frontRight->GetModulePosition(),
                          m_backLeft->GetModulePosition(),  m_backRight->GetModulePosition()});

  // Also apply vision measurements. We use 0.3 seconds in the past as an
  // example -- on a real robot, this must be calculated based either on latency
  // or timestamps.
  m_poseEstimator.AddVisionMeasurement(
      frc::Pose2d(), // DO THIS TODO, THIS DOES VISION POSE ESTIMATION
      frc::Timer::GetFPGATimestamp() - 0.3_s);
}
