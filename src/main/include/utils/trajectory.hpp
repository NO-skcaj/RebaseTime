#pragma once

#include <iostream>

#include <frc/TimedRobot.h>
#include <frc/DriverStation.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

#include <frc/controller/PIDController.h>

#include <frc/smartdashboard/Field2d.h>

#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ProfiledPIDCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include <frc2/command/Requirements.h>

// I know I have too many of these, but at this point im done
#include <units/velocity.h>
#include <units/base.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/energy.h>
#include <units/force.h>

#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>

#include "../subsystems/Swerve.hpp"
#include "../Constants.hpp"

using namespace frc;
using namespace frc2;

/// @brief Class to support autonomous commands.
class trajectory
{
    public:
        /// @brief Constructor for the AutoCommands class.
        /// @param swerve - Pointer to the robot swerve subsystem.
        trajectory(Swerve *swerve);

        frc2::CommandPtr getAutonomousCommand();

        Swerve *swerve;

        frc::Field2d field;
    private:

        pathplanner::HolonomicPathFollowerConfig pathFollowerConfig(
            pathplanner::PIDConstants((double)1), // Translation constants * PID controllers here can just be P because 
            pathplanner::PIDConstants((double)1), // Rotation constants    * PID is handled in the swerve modules themselves
            units::meters_per_second_t{constants::movement::MAX_SPEED_MPS},
            units::meter_t            {0.381}, // Drive base radius (distance from center to furthest module) 
            pathplanner::ReplanningConfig()
        );
};