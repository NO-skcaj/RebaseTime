#include "utils/hardware.hpp"

using namespace hardware;

motors::TalonFX::TalonFX(int CAN, motors::motor_type Mode)
{ 
    this->talonMode = Mode;
    this->CANID     = CAN;
    // Actual motor
    this->talonMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CAN, "rio");
};

// I DO MY OWN DEFAULT CONSTRUCTORS, YOU CANT TELL ME WHAT TO DO
motors::TalonFX::TalonFX() {};

/// @brief gets a CANcoder bc they are great
void motors::TalonFX::GetEncoder(int CAN)
{
    this->encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(CAN, "rio");
};

void motors::TalonFX::AddMotor(int CAN, motors::motor_type Mode)
{
    this->talonMode = Mode;
    this->CANID     = CAN;
    // Actual motor
    this->talonMotor = std::make_unique<ctre::phoenix6::hardware::TalonFX>(CAN, "rio");
};

/// @brief does config, most just so happens to be PID
void motors::TalonFX::Config(float P, float I, float D, int max_amperage)
{
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};

    // set slot 0 gains and leave every other config factory-default
    ctre::phoenix6::configs::Slot0Configs& slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = P;
    slot0Configs.kI = I;
    slot0Configs.kD = D;

    // apply all configs, 50 ms total timeout
    this->talonMotor.get()->GetConfigurator().Apply(talonFXConfigs, 50_ms);

    // Set the current limit
    ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitsConfigs{};
    currentLimitsConfigs.StatorCurrentLimit       = max_amperage;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    this->talonMotor.get()->GetConfigurator().Apply(currentLimitsConfigs);
}

/// @brief does config, most just so happens to be PIDV
/// @todo MAGIC MOTION
void motors::TalonFX::Config(float P, float I, float D, float V, int max_amperage)
{
    angleController.EnableContinuousInput(-3.14159, 3.14159); // OP TIM ICE AYE CHIN also with SwerveModuleState.Optimize()

    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};

    // set slot 0 gains and leave every other config factory-default
    talonFXConfigs.Slot0.kV = V;
    talonFXConfigs.Slot0.kP = P;
    talonFXConfigs.Slot0.kI = I;
    talonFXConfigs.Slot0.kD = D;

    // apply all configs, 50 ms total timeout
    this->talonMotor.get()->GetConfigurator().Apply(talonFXConfigs, 50_ms);

    // Set the current limit
    ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitsConfigs{};
    currentLimitsConfigs.StatorCurrentLimit       = max_amperage;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    this->talonMotor.get()->GetConfigurator().Apply(currentLimitsConfigs);
}

/// @brief When in DRIVE mode it just does 1,-1; when in ANGLE mode it intakes the 
void motors::TalonFX::Set(double input)  
{
    switch (this->talonMode)
    {
    case motors::motor_type::DRIVE:
        this->talonMotor.get()->Set(input);
    case motors::motor_type::ANGLE:
        this->talonMotor.get()->Set(this->angleController.Calculate(this->GetMotorPosition(), input));
    default: /*this will never happen, there are only 2 possiblities*/
        this->CANID = 69;
    }
}

/// @brief just the current set speed
/// @returns last setSpeed() input -1.0 - 1
double motors::TalonFX::GetSpeed()  
{
    return this->talonMotor.get()->Get();
}

/// @brief Gets the angle
/// @return position/distance driven in degrees
double motors::TalonFX::GetMotorPosition()  
{
    return units::unit_cast<double, units::angle::turn_t>(this->talonMotor.get()->GetPosition().GetValue() * 360);
}