// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/WaitCommand.h>

#include "commands/DriveToLineCommand.h"
#include "commands/DriveUntilCommand.h"

#include <units/time.h>

#ifdef USE_XBOX_CONTROLS
  #include <frc/XboxController.h>
#else
  #include <frc/Joystick.h>
#endif
#include <frc/Timer.h>

#include "subsystems/ClimberSubsystem.h"
#include "subsystems/DriveSubsytem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TransportSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
public:
  RobotContainer();

  frc2::Command* autonomousCommand();
private:
  // The robot's subsystems and commands are defined here...
  DriveSubsystem driveSubsystem;
  ClimberSubsystem climberSubsystem;
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  TransportSubsystem transportSubsystem;
#ifdef USE_XBOX_CONTROLS
  frc::XboxController controller1{constants::XBOX_CONTROL_1};
  frc::XboxController controller2{constants::XBOX_CONTROL_2};
#else
  frc::Joystick control1{constants::CONTROL1};
  frc::Joystick control2{constants::CONTROL2};
#endif

  frc::Timer innerTimer;
  void ConfigureButtonBindings();

  frc2::SequentialCommandGroup autocmd{
    frc2::InstantCommand([this]{transportSubsystem.disableInnerBelt();}),
    DriveToLineCommand(&driveSubsystem, false),
    frc2::InstantCommand([this]{driveSubsystem.resetDistance();}),
    DriveUntilCommand(&driveSubsystem, false, [this] {
      return driveSubsystem.distance() <= -30;
    }),
    frc2::RunCommand([this] {
      transportSubsystem.enableInnerBelt();
    })
  };
};
