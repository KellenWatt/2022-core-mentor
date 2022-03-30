// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#define USE_XBOX_CONTROLS

namespace constants {
    constexpr int CONTROL1 = 0;
    constexpr int CONTROL2 = 1;

    constexpr int XBOX_CONTROL_1 = 0;
    constexpr int XBOX_CONTROL_2 = 1;

    namespace drive {
        constexpr int FRONT_LEFT_MOTOR = 4;
        constexpr int REAR_LEFT_MOTOR = 1;
        constexpr int FRONT_RIGHT_MOTOR = 2;
        constexpr int REAR_RIGHT_MOTOR = 3;
        
        constexpr int LINE_SENSOR = 0;

        constexpr int MOTOR_COUNT = 4;

        constexpr double DEFAULT_DEADBAND = 0.10;
        constexpr double ROTATION_ADJUSTMENT_RATE = 0.15;

        constexpr double ROTATION_REDUCTION = 0.6;
        constexpr double SELF_ROTATION_SPEED = 0.6;

        constexpr double RAMP_COEFFICIENT = 0.45;
    }

    namespace intake {
        constexpr int FORWARD_SOLENOID = 3;//2;
        constexpr int REVERSE_SOLENOID = 4;//5;

        constexpr int ROLLER_MOTOR = 5;

        constexpr double DEFAULT_ROLLER_SPEED = -0.5;
    }

    namespace shooter {
        constexpr int SHOOTER_MOTOR = 8;

        // original: 3800
        constexpr double FLYWHEEL_RPM = 3500;
        namespace pid {
            constexpr double P = 7e-5;
            constexpr double I = 5e-7;
            constexpr double D = 1.5e-4;
            constexpr double FF = 1e-5;
        }
    }

    namespace climber {
        constexpr int LOWER_FORWARD_SOLENOID = 0;
        constexpr int LOWER_REVERSE_SOLENOID = 7;
        constexpr int UPPER_FORWARD_SOLENOID = 6;
        constexpr int UPPER_REVERSE_SOLENOID = 1;

        constexpr int LIFT_SWITCH_RIGHT = 2;
        constexpr int LIFT_SWITCH_LEFT = 5;
    }

    namespace transport {
        constexpr int INNER_BALL_SWITCH = 3;
        constexpr int OUTER_BALL_SWITCH = 4;
        // constexpr int OUTER_COLOR_SENSOR = 0;

        constexpr int INNER_BELT_MOTOR = 6;
        constexpr int OUTER_BELT_MOTOR = 7;

        constexpr double INNER_BELT_SPEED = -0.5;
        constexpr double OUTER_BELT_SPEED = 0.5;
    }
}