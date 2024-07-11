#include "Robot.hpp"

void Robot::RobotPeriodic()
{
  m_swerve.UpdateOdometry();

  if(this->driveMode == modes::AUTO)
  {
    AutoCommand.value().get()->Schedule();
  } else 
  if (this->driveMode == modes::DRIVE)
  {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    const units::meters_per_second_t xSpeed{units::unit_cast<double>(-m_xspeedLimiter.Calculate(m_controller.GetLeftY())) * constants::movement::MAX_SPEED_MPS};

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    const units::meters_per_second_t ySpeed{units::unit_cast<double>(-m_yspeedLimiter.Calculate(m_controller.GetLeftX())) * constants::movement::MAX_SPEED_MPS};

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    const units::radians_per_second_t            rot{units::unit_cast<double>(-m_rotLimiter.Calculate(m_controller.GetRightX())) * constants::movement::MAX_ANGULAR_SPEED};

    m_swerve.Drive(xSpeed, ySpeed, rot, this->fieldRelative, GetPeriod());
  }
  
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit()
{
  this->driveMode = modes::AUTO;
}

void Robot::TeleopPeriodic() 
{
  this->driveMode = modes::DRIVE;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
