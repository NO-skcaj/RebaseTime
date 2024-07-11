// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc2/command/Subsystem.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include "modules/SwerveModule.hpp"
#include "utils/hardware.hpp"
#include "Constants.hpp"

/**
 * Represents a swerve drive style Swerve.
 */
class Swerve : public frc2::Subsystem {
    public:
        Swerve(hardware::gyro::navx* gyro);

        void Drive(units::meters_per_second_t xSpeed,
                    units::meters_per_second_t ySpeed, 
                    units::radians_per_second_t rot,
                    bool fieldRelative, 
                    units::second_t period);

        void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states);

        void ResetOdometry(frc::Pose2d pose = frc::Pose2d());

        void UpdateOdometry();

        frc::ChassisSpeeds GetSpeeds();

        frc::Pose2d GetPose();

        frc::SwerveDriveKinematics<4> GetKinematics();

        frc::Translation2d m_frontLeftLocation {+0.381_m, +0.381_m};
        frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
        frc::Translation2d m_backLeftLocation  {-0.381_m, +0.381_m};
        frc::Translation2d m_backRightLocation {-0.381_m, -0.381_m};

        frc::SwerveDriveKinematics<4> m_kinematics{
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
            m_backRightLocation};

    private:
        hardware::gyro::navx* Gyro;

        SwerveModule* m_frontLeft;  // init all the Swerve Modules
        SwerveModule* m_frontRight; // Drive motor, Turning motor, Turning encoder
        SwerveModule* m_backLeft;
        SwerveModule* m_backRight;

        frc::SwerveModulePosition initModPos[4];

        // Gains are for example purposes only - must be determined for your own
        // robot
        frc::SwerveDrivePoseEstimator<4> m_poseEstimator = frc::SwerveDrivePoseEstimator<4>(
            (frc::SwerveDriveKinematics<4>&) 	m_kinematics,
            (const frc::Rotation2d &) frc::Rotation2d(),
            (const wpi::array<frc::SwerveModulePosition,4> &) 	initModPos,
            (const frc::Pose2d &) frc::Pose2d{}
        );
};
