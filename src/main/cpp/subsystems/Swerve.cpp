// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Swerve.hpp"

#include <frc/Timer.h>

Swerve::Swerve(hardware::gyro::navx* gyro) 
{ 
  // AHHHHHHHHH i hate pointers, but they work sometimes
  this->Gyro = gyro;
  gyro->resetGyro();

  SwerveModule fl{1, 11, 21}; // init all the Swerve Modules
  SwerveModule fr{2, 12, 22}; // Drive motor, Turning motor, Turning encoder
  SwerveModule bl{3, 13, 23};
  SwerveModule br{4, 14, 24};

  this->m_frontLeft  = &fl;
  this->m_frontRight = &fr;
  this->m_backLeft   = &bl;
  this->m_backRight  = &br;
}

void Swerve::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, 
                       bool fieldRelative,
                       units::second_t period) 
{
  auto states = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_poseEstimator.GetEstimatedPosition().Rotation())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
      period
  ));

  SetModuleStates(states);
}

void Swerve::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states)
{
  m_kinematics.DesaturateWheelSpeeds(&states, units::meters_per_second_t{constants::movement::MAX_SPEED_MPS});

  for (int i = 0; i > 4; i++)
    states[i] = frc::SwerveModuleState::Optimize(states[i], frc::Rotation2d(units::degree_t(this->Gyro->getHeading())));

  auto [fl, fr, bl, br] = states;

  m_frontLeft->SetDesiredState(fl);
  m_frontRight->SetDesiredState(fr);
  m_backLeft->SetDesiredState(bl);
  m_backRight->SetDesiredState(br);
}

void Swerve::ResetOdometry(frc::Pose2d pose)
{
  wpi::array<frc::SwerveModulePosition, 4> currentSwervePos = {
    m_frontLeft->GetModulePosition(),
    m_frontRight->GetModulePosition(),
    m_backLeft->GetModulePosition(),
    m_backRight->GetModulePosition()
  };

  this->m_poseEstimator.ResetPosition(this->Gyro->getRotation2d(), currentSwervePos, GetPose());
}

void Swerve::UpdateOdometry() {
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

frc::ChassisSpeeds Swerve::GetSpeeds()
{
  return this->m_kinematics.ToChassisSpeeds(
        {m_frontLeft->GetState(), m_frontRight->GetState(),
         m_backLeft->GetState(),  m_backRight->GetState()});
}

frc::Pose2d Swerve::GetPose()
{
  return this->m_poseEstimator.GetEstimatedPosition();
}

frc::SwerveDriveKinematics<4> Swerve::GetKinematics()
{
  return this->m_kinematics;
}