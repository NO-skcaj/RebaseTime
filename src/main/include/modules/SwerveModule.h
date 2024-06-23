// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkLowLevel.h>

#include <ctre/phoenix/sensors/CANCoder.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "modules/SwerveModule.h"
#include "utils/hardware.hpp"
#include "Constants.hpp"

using namespace hardware;

class SwerveModule {
    public:
        SwerveModule(int driveMotorChannel, 
                     int turningMotorChannel,
                     int turningEncoderChannel,
                     gyro::navx* gyro);

        frc::SwerveModuleState GetState();

        frc::SwerveModulePosition GetModulePosition();
        
        void SetDesiredState(const frc::SwerveModuleState& state);

    private:
        static constexpr auto kWheelRadius = 0.0508_m;

        /// @brief What was last outputed to the motors
        frc::SwerveModuleState current_state;

        gyro::navx* Gyro;

        // *** DRIVE MOTORS *** //

        motors::TalonFX     d_motor;

        // *** ANGLE MOTORS *** //

        // motors::CANSparkMax t_motor;
        
        // *** Utils *** //
        /// @brief  Converts funny silly Position Status Signal Object to "normal people units" aka schizo wpilib units lib
        /// @param rotations - Result from TalonFX GetPosition.GetValue()
        /// @return usable units for odometry
        units::meter_t rotationsToMeters(units::turn_t rotations);
};
