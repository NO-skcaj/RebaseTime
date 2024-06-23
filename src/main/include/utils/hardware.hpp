#pragma once
//NavX gyro all i've ever known
#include "AHRS.h"
//ctre my beloved
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
//REV?!?!?!!11?? what is this FILTH doing in MY spagetti-less codebase
#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkPIDController.h>
//wpi, I CAN WRITE MY OWN LIBRARIES, but i wont 
#include <frc/controller/PIDController.h>

namespace hardware {




  namespace motors
  {
    enum motor_type
    {
      DRIVE, // Spin without care
      ANGLE  // Spin with care
    };

    // Default that motors use to simplify the interface, notable exception is CANSparkMax which requires a SnapZero() if ANGLE
    class baseMotor
    {
      public:
        // virtual void GetEncoder(int CAN);
        // // Configs
        // virtual void Config(float P, float I, float D, int max_amperage);
        // // Multi-purpose, Angle/Drive
        // virtual void Set(double iinput);
        // // Get commands aka telemetry
        // virtual double GetSpeed();
        // virtual double GetMotorPosition();
        // //etc
        // virtual void operator=(baseMotor const& newObj);
    };

    class TalonFX : public hardware::motors::baseMotor
    {
      public:

        // MUST also call config
        TalonFX(int CAN, motors::motor_type Mode);

        // I DO MY OWN DEFAULT CONSTRUCTORS, YOU CANT TELL ME WHAT TO DO
        TalonFX();

        /// @brief gets a CANcoder bc they are great
        void GetEncoder(int CAN);

        /// @brief gives the ability to add the motor post initialization
        void AddMotor(int CAN, motors::motor_type Mode);

        /// @brief does config, most just so happens to be PID
        void Config(float P, float I, float D, int max_amperage);

        /// @brief does config, most just so happens to be PIDV
        /// @todo MAGIC MOTION
        void Config(float P, float I, float D, float V, int max_amperage);

        /// @brief When in DRIVE mode it just does 1,-1; when in ANGLE mode it intakes the 
        void Set(double input);

        /// @brief just the current set speed
        /// @returns last setSpeed() input -1.0 - 1
        double GetSpeed();

        /// @brief Gets the angle
        /// @return position/distance driven in degrees
        double GetMotorPosition();

        // PRIVATES
        std::unique_ptr<ctre::phoenix6::hardware::TalonFX>  talonMotor;
        std::unique_ptr<ctre::phoenix6::hardware::CANcoder> encoder;
        
        frc::PIDController angleController{1, 0, 0}; // why is this here? IG it doesnt need to be, OH SHOOT NVM we've got this beut' of a optimization
        motors::motor_type talonMode;

        int CANID;
    };


    // // I hate REV I HATE REV I HATE REV
    // class CANSparkMax : public motors::baseMotor
    // {
    //   public:
    //     CANSparkMax(int CAN, motor_type Mode)
    //     { 
    //       this->mode = Mode;
    //       rev::CANSparkMax& temp{CAN, rev::CANSparkLowLevel::MotorType::kBrushless};
    //       this->motor = temp;
    //     };

    //     // needs to be called with or without CAN id, just dont put in the argument
    //     void GetEncoder(int CAN = 999)  
    //     {
    //       // Get real relative encoders
    //       this->encoder = new rev::SparkRelativeEncoder(this->motor->GetEncoder());
    //       if (CAN != 999)
    //       {
    //         this->AbsEncoder = ctre::phoenix6::hardware::CANcoder{CAN, "rio"}
    //         this->AbsEncoder->ConfigAbsoluteSensorRange(ctre::ph ::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);
    //       }
    //     };
    //     // Configs
    //     void Config(float P, float I, float D, int max_amperage)  
    //     {
    //       // Set current limit
    //       this->motor->SetSmartCurrentLimit(max_amperage);

    //       // Turn on brake coast mode, snappier
    //       this->motor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    //       // Swerve wheel PID contrllers
    //       this->PIDController = new SparkPIDController(this->motor->GetPIDController());
    //       this->PIDController->SetP(P);
    //       this->PIDController->SetI(I);
    //       this->PIDController->SetD(D);

    //       // Burn flash everytime
    //       this->motor->BurnFlash();
    //     };

    //     // Multi-purpose, Angle/Drive
    //     void Set(double input)  
    //     {
    //       switch (mode)
    //       {
    //         case DRIVE:
    //           this->motor->Set(input);
    //         case ANGLE:
    //           this->PIDController->SetReference(input * 21);
    //       }
    //     };
    //     //
    //     void SnapZero()
    //     {
    //       // get pos from encoder, conv to deg, conv to encoder rotations
    //       this->encoder->SetPosition((AbsEncoder->GetAbsolutePosition() / 360 * 21));
    //     };

    //     // Speed in rotations per minute
    //     double GetSpeed()  
    //     {
    //       return encoder->GetVelocity();
    //     };

    //     /// @brief Gets the angle
    //     /// @return In degrees, must be in ANGLE mode to get angle, otherwise it just gives the distance in angles
    //     double GetMotorPosition()  
    //     {
    //       return encoder->GetPosition() * 360;
    //     };

    //   private:
    //     rev::CANSparkMax*                 motor;
    //     rev::SparkRelativeEncoder*        encoder;
    //     rev::SparkPIDController*          PIDController;
    //     ctre::phoenix::sensors::CANCoder* AbsEncoder;
    //     motors::motor_type mode;
    // };

  } // motors



  namespace gyro
  {
    class navx  
    {
      public:
        /// @brief Create an attitude and heading reference system (AHRS).
        std::unique_ptr<AHRS> m_gyro = std::make_unique<AHRS>(frc::SerialPort::SerialPort::Port::kMXP);

        double getHeading() 
        {
          return m_gyro.get()->GetYaw();
        };

        frc::Rotation2d getRotation2d() 
        {
          return m_gyro.get()->GetRotation2d();
        };

        void resetGyro()
        {
          m_gyro.get()->Reset();
        };
    };
  } // gyro
;} // hardware