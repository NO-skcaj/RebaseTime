// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>

#include <frc/filter/SlewRateLimiter.h>

#include <frc2/command/CommandPtr.h>

#include "subsystems/Swerve.hpp"
#include "utils/hardware.hpp"
#include "Constants.hpp"

class Robot : public frc::TimedRobot {
 public:
  
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  hardware::gyro::navx Gyro;
 private:
  frc::XboxController m_controller{0};
  Swerve m_swerve{&Gyro};

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{3 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter   {3 / 1_s};

  enum modes {
    AUTO,
    DRIVE
  };

  modes driveMode = modes::AUTO;

  bool fieldRelative = true;

  std::optional<frc2::CommandPtr> AutoCommand;
};
