// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "modules/SwerveModule.hpp"

using namespace hardware;
using namespace constants;

SwerveModule::SwerveModule(int driveMotorChannel, 
                           int turningMotorChannel,
                           int turningEncoderChannel) 
{
    this->d_motor.AddMotor(driveMotorChannel, hardware::motors::motor_type::DRIVE);

    this->d_motor.Config(swerve::DRIVE_P, 
                         swerve::DRIVE_I, 
                         swerve::DRIVE_D, 
                         swerve::DRIVE_AMPERAGE);
    
    // this->t_motor.AddMotor(turningMotorChannel, hardware::motors::motor_type::DRIVE, true);

    // this->t_motor.Config(ANGLE_P, ANGLE_I, ANGLE_D, ANGLE_AMPERAGE);

    // this->t_motor.GetEncoder(turningEncoderChannel);
    // this->t_motor.SnapZero();
}

frc::SwerveModuleState SwerveModule::GetState()  
{
    return this->current_state;

}

frc::SwerveModulePosition SwerveModule::GetModulePosition()  
{
    return {
        units::meter_t(units::unit_cast<double>(rotationsToMeters(units::angle::turn_t(this->d_motor.GetMotorPosition() / 360))) /* * SWERVE_DRIVE_WHEELS_CIRCUMFERENCE */), 
        units::degree_t(0)// units::degree_t(this->t_motor.GetModulePosition())
    };
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& state) 
{
    double Speed = state.speed();
    double Angle = units::unit_cast<double>(state.angle.Radians() * 0.01745);
    
    // Set the motors to do motor things; applies kinematics
    this->d_motor.Set(Speed);
    // this->t_motor.Set(Angle);
}

/// @brief  Converts funny silly Position Status Signal Object to "normal people units" aka schizo wpilib units lib
/// @param rotations - Result from TalonFX GetPosition.GetValue()
/// @return usable units for odometry
units::meter_t SwerveModule::rotationsToMeters(units::turn_t rotations)
{
    /* Get circumference of wheel */
    // Needed for rotationsToMeters, basically just a special type for the circumfrence in inches
    units::inch_t simpleCircumference {1 * 39.37};
    auto circumference = simpleCircumference * 3.14159 / 1_tr;
    /* Every rotation of the wheel travels this many inches */
    /* Now apply gear ratio to input rotations */
    auto gearedRotations = rotations / 1;
    /* And multiply geared rotations by meters per rotation */
    return gearedRotations * circumference;
}