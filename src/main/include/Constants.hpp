#pragma once

namespace constants
{

    namespace movement
    {
        constexpr float MAX_SPEED_MPS = 3;
        constexpr float MAX_ACCEL_MPS = 1;
        static constexpr float MAX_ANGULAR_SPEED = std::numbers::pi;  // 1/2 rotation per second, radians per sec
    }

    namespace swerve
    {
        // THESE VALUES ARE NOT TESTED
        constexpr float DRIVE_P     = 0.7;
        constexpr float DRIVE_I     = 0;
        constexpr float DRIVE_D     = 0.03;
        constexpr int DRIVE_AMPERAGE = 30;
        
        constexpr float ANGLE_P     = 0.5;
        constexpr float ANGLE_I     = 0;
        constexpr float ANGLE_D     = 0;
        constexpr int ANGLE_AMPERAGE = 30;
    }

}