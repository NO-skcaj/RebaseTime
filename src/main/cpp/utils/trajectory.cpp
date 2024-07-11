#include "utils/trajectory.hpp"

using namespace frc;
using namespace frc2;

using namespace pathplanner;

using namespace constants;

/// @brief Constructor for the AutoCommands class.
/// @param swerve - Pointer to the robot swerve subsystem.
/// @param intake - Pointer to the robot intake subsystem.
trajectory::trajectory(Swerve *swerve)
{
  this->swerve = swerve;

  // Configure AutoBuilder
  AutoBuilder::configureHolonomic(
      [this]()                                       {return this->swerve->GetPose();},
      [this](frc::Pose2d pose)                       {this->swerve->ResetOdometry(pose);},
      [this]()                                       {return this->swerve->GetSpeeds();},
      [this](frc::ChassisSpeeds robotRelativeSpeeds) {auto states = this->swerve->m_kinematics.ToSwerveModuleStates(robotRelativeSpeeds);
                                                      this->swerve->SetModuleStates(states);},
      this->pathFollowerConfig,
      []() {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
          if (alliance.has_value()) {
              return alliance.value() == frc::DriverStation::Alliance::kRed;
          }
          return false;
      },
      this
  );

  // Set up custom logging to add the current path to a field 2d widget
  PathPlannerLogging::setLogActivePathCallback(
    [this](auto poses) {this->field.GetObject("path")->SetPoses(poses);}
  );

}

/// @brief This function does something *SPECIAL*. This basically creates the all of the commands for the auto.
frc2::CommandPtr trajectory::getAutonomousCommand() 
{

  // // Set up config for trajectoryb 
  // frc::TrajectoryConfig config(
  //   units::meters_per_second_t        {movement::MAX_SPEED_MPS},
  //   units::meters_per_second_squared_t{movement::MAX_ACCEL_MPS}
  // );
  // // Add kinematics to ensure max speed is actually obeyed
  // config.SetKinematics(swerve->m_kinematics);

  // // An example trajectory to follow.  All units in meters.
  // auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //   frc::Pose2d{0_m, 0_m, 0_deg},
  //   {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
  //   frc::Pose2d{3_m, 0_m, 0_deg},
  //   config
  // );

  // TrapezoidProfile<units::radians>::Constraints thetaConstraints{
  //   units::radians_per_second_t(90),
  //   units::radians_per_second_squared_t(45),
  // };

  // frc::ProfiledPIDController<units::radians> thetaController{
  //   swerve::ANGLE_P, swerve::ANGLE_I, swerve::ANGLE_D, 
  //   thetaConstraints
  // };

  // thetaController.EnableContinuousInput(
  //   units::radian_t{-std::numbers::pi},
  //   units::radian_t{std::numbers::pi}
  // );

  // auto swerveControllerCommandnope = frc2::SwerveControllerCommand<4>(
  //   exampleTrajectory,
  //   [swerve]()                  {return swerve->GetPose(); },
  //   swerve->GetKinematics(),
  //   frc::PIDController          {swerve::DRIVE_P, swerve::DRIVE_I, swerve::DRIVE_D},
  //   frc::PIDController          {swerve::DRIVE_P, swerve::DRIVE_I, swerve::DRIVE_D},
  //   thetaController,
  //   [swerve](auto moduleStates) {swerve->SetModuleStates(moduleStates); },
  //   {swerve}
  // );
  
  // frc2::CommandPtr swerveControllerCommand = swerveControllerCommandnope.ToPtr();

  // Reset odometry to the initial pose of the trajectory, run path following
  // command, then stop at the end.
  // frc2::CommandPtr seqeeenz = frc2::cmd::Sequence (
  //   frc2::InstantCommand(
  //     [swerve, initialPose = exampleTrajectory.InitialPose()]() {
  //     swerve->ResetOdometry(initialPose);
  //     },{}
  //   ).ToPtr(),

  //   std::move(swerveControllerCommand),

  //   frc2::InstantCommand(
  //     [this]() {this->swerve->Drive(units::meters_per_second_t{0}, units::meters_per_second_t{0}, units::radians_per_second_t{0}, true, units::second_t{0.020}); }, {}
  //   ).ToPtr()
  // );

  // return seqeeenz;
}