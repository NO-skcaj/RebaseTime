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

#include "modules/SwerveModule.h"
#include "utils/hardware.hpp"

/**
 * Represents a swerve drive style drivetrain.
 */
class Drivetrain {
    public:
        Drivetrain(hardware::gyro::navx* gyro) 
        { 
            // AHHHHHHHHH i hate pointers, but they work sometimes
            this->Gyro = gyro;
            gyro->resetGyro();

            SwerveModule fl{1, 11, 21, Gyro}; // init all the Swerve Modules
            SwerveModule fr{2, 12, 22, Gyro}; // Drive motor, Turning motor, Turning encoder
            SwerveModule bl{3, 13, 23, Gyro};
            SwerveModule br{4, 14, 24, Gyro};

            this->m_frontLeft  = &fl;
            this->m_frontRight = &fr;
            this->m_backLeft   = &bl;
            this->m_backRight  = &br;
        }

        void Drive(units::meters_per_second_t xSpeed,
                    units::meters_per_second_t ySpeed, 
                    units::radians_per_second_t rot,
                    bool fieldRelative, 
                    units::second_t period);

        void UpdateOdometry();

        static constexpr auto kMaxSpeed = 3.0_mps;  // 3 meters per second
        static constexpr units::radians_per_second_t kMaxAngularSpeed{
        std::numbers::pi};  // 1/2 rotation per second

    private:
        hardware::gyro::navx* Gyro;

        frc::Translation2d m_frontLeftLocation {+0.381_m, +0.381_m};
        frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
        frc::Translation2d m_backLeftLocation  {-0.381_m, +0.381_m};
        frc::Translation2d m_backRightLocation {-0.381_m, -0.381_m};

        SwerveModule* m_frontLeft;  // init all the Swerve Modules
        SwerveModule* m_frontRight; // Drive motor, Turning motor, Turning encoder
        SwerveModule* m_backLeft;
        SwerveModule* m_backRight;

        frc::SwerveDriveKinematics<4> m_kinematics{
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
            m_backRightLocation};

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
