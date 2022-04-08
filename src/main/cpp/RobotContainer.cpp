// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/button/Trigger.h>

#include <units/time.h>

#include "commands/DriveCommand.h"
#include "commands/FlywheelCommand.h"
#include "commands/DriveToLineCommand.h"
#include "commands/DriveUntilCommand.h"

#include <cstdio>

RobotContainer::RobotContainer() : transportSubsystem(frc::DriverStation::GetAlliance()) {
  singleAutocmd.AddRequirements(&transportSubsystem);
  autoChooser.SetDefaultOption("Single Cargo", &singleAutocmd);
  autoChooser.AddOption("Double Cargo", &doubleAutocmd);
  autoChooser.AddOption("(Don't use) Sideways Double", &sidewaysAutocmd);
  frc::SmartDashboard::PutData("Auto Mode", &autoChooser);
#ifdef USE_XBOX_CONTROLS
  driveSubsystem.SetDefaultCommand(DriveCommand(&driveSubsystem, &controller1));
#else
  driveSubsystem.SetDefaultCommand(DriveCommand(&driveSubsystem, &control1));
#endif
  shooterSubsystem.SetDefaultCommand(FlywheelCommand(&shooterSubsystem));

  innerTimer.Start();
  innerTimer.Reset();
  transportSubsystem.SetDefaultCommand(frc2::RunCommand([this] {
    // uncomment this line if the shooter bug pops up again.
    // if(frc::DriverStation::IsAutonomous() || control1.GetRawButton(1)) return;
    if(frc::DriverStation::IsAutonomous()) return;
    if(!transportSubsystem.hasOuterBall() || !transportSubsystem.hasInnerBall()) {
      transportSubsystem.enableOuterBelt();
    } else {
      transportSubsystem.disableOuterBelt();
    }

    if(!transportSubsystem.hasInnerBall()) {
      transportSubsystem.enableInnerBelt();
      innerTimer.Reset();
    } else if(innerTimer.HasElapsed(0.15_s)) {
        transportSubsystem.disableInnerBelt();
    }
  }, {&transportSubsystem}));

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // extends/retracts the roller arm of the intake
  auto toggle_intake_arm = frc2::InstantCommand([this] {intakeSubsystem.toggleArm();}, 
                                                {&intakeSubsystem});

  auto toggle_shooter_wheel = frc2::StartEndCommand(
    [this]{shooterSubsystem.disableFlywheel();},
    [this]{shooterSubsystem.enableFlywheel();},
    {&shooterSubsystem}
  );

  auto toggle_transport_belts = frc2::StartEndCommand(
    [this]{
      transportSubsystem.disableOuterBelt();
      transportSubsystem.disableInnerBelt();
    },
    []{},
    {&transportSubsystem}
  );
  // Run the intake roller. Run it in reverse if button 11 pressed on Joystick, 
  // or left bumper pressed on Xbox controller.
  auto run_intake_roller = frc2::FunctionalCommand(
      [] {},
      [this]{
        intakeSubsystem.startRoller();
      },
      [this](bool)
      { intakeSubsystem.stopRoller(); },
      []
      { return false; });
  
  auto reverse_intake_roller = frc2::FunctionalCommand(
      [] {},
      [this]{
        intakeSubsystem.reverseRoller();
      },
      [this](bool)
      { intakeSubsystem.stopRoller(); },
      []
      { return false; });

  // Runs the outer transport belt in reverse
  auto reverse_transport = frc2::StartEndCommand(
    [this] {
      transportSubsystem.reverseOuterBelt();
      transportSubsystem.reverseInnerBelt();
    },
    [this] {
      transportSubsystem.disableOuterBelt();
      transportSubsystem.disableInnerBelt();
    },
    {&transportSubsystem}
  );

  auto enable_transport = frc2::RunCommand([this]{
      transportSubsystem.enableInnerBelt();
      transportSubsystem.enableOuterBelt();
    }, {&transportSubsystem});

  auto toggle_lower_arms = frc2::InstantCommand([this]{climberSubsystem.toggleLower();});
  auto upper_arms_release = frc2::StartEndCommand(
    [this]{climberSubsystem.extendUpper(false);},
    [this]{climberSubsystem.retractUpper(false);},
    {&climberSubsystem}
  );

  // Both sets of bindings have equivalant capabilities. 
  // The only difference is which control scheme is being used.
#ifdef USE_XBOX_CONTROLS
  // Button bindings for Xbox Controller
  // Controller 1
  // - Drive
  // D-pad down - drive backwards to line
  frc2::Trigger([this]{return controller1.GetPOV() == 180;}).ToggleWhenActive(DriveToLineCommand(&driveSubsystem, false));
  // D-pad down - drive forwards to line
  frc2::Trigger([this]{return controller1.GetPOV() == 0;}).ToggleWhenActive(DriveToLineCommand(&driveSubsystem, true));
  (frc2::JoystickButton(&controller1, frc::XboxController::Button::kBack) && 
   frc2::JoystickButton(&controller1, frc::XboxController::Button::kLeftBumper) &&
   frc2::JoystickButton(&controller1, frc::XboxController::Button::kRightBumper)).ToggleWhenActive(toggle_shooter_wheel);
  (frc2::JoystickButton(&controller1, frc::XboxController::Button::kStart) && 
   frc2::JoystickButton(&controller1, frc::XboxController::Button::kLeftBumper) &&
   frc2::JoystickButton(&controller1, frc::XboxController::Button::kRightBumper)).ToggleWhenActive(toggle_transport_belts);
  // - Shoot

  // Controller 2
  frc2::JoystickButton(&controller2, frc::XboxController::Button::kB).ToggleWhenPressed(toggle_intake_arm);
  (frc2::JoystickButton(&controller2, frc::XboxController::Button::kA) && 
    !frc2::JoystickButton(&controller2, frc::XboxController::Button::kLeftBumper))
      .WhileActiveOnce(run_intake_roller);
  (frc2::JoystickButton(&controller2, frc::XboxController::Button::kA) && 
    frc2::JoystickButton(&controller2, frc::XboxController::Button::kLeftBumper))
      .WhileActiveOnce(reverse_intake_roller);
  frc2::JoystickButton(&controller2, frc::XboxController::Button::kX).WhileActiveOnce(reverse_transport);
  
  (frc2::JoystickButton(&controller2, frc::XboxController::Button::kY) && 
    !frc2::JoystickButton(&controller2, frc::XboxController::Button::kLeftBumper)).WhenActive(toggle_lower_arms);
  (frc2::JoystickButton(&controller2, frc::XboxController::Button::kY) && 
    frc2::JoystickButton(&controller2, frc::XboxController::Button::kLeftBumper)).ToggleWhenActive(upper_arms_release);

#else
  // Button bindings for Joysticks.
  // Joystick 1 - not including driving
  // drive forwards to line
  frc2::JoystickButton(&control1, 9).ToggleWhenPressed(DriveToLineCommand(&driveSubsystem, true));
  // drive backwards to line
  frc2::JoystickButton(&control1, 11).ToggleWhenPressed(DriveToLineCommand(&driveSubsystem, false));
  // disable/re-enable shooter
  frc2::JoystickButton(&control1, 7).ToggleWhenPressed(toggle_shooter_wheel);
  frc2::JoystickButton(&control1, 8).ToggleWhenPressed(toggle_transport_belts);
  

  // Joystick 2
  frc2::JoystickButton(&control2, 1).WhenPressed(toggle_intake_arm);
  (frc2::JoystickButton(&control2, 2) && !frc2::JoystickButton(&control2, 11))
      .WhileActiveOnce(run_intake_roller);
  (frc2::JoystickButton(&control2, 2) && frc2::JoystickButton(&control2, 11))
      .WhileActiveOnce(reverse_intake_roller);
  frc2::JoystickButton(&control2, 4).WhenHeld(reverse_transport);

  frc2::JoystickButton(&control2, 6).WhenPressed(toggle_lower_arms);
  frc2::JoystickButton(&control2, 5).ToggleWhenPressed(upper_arms_release);
#endif

  // specifically inlined commands. Moved here to reduce duplication between control schemes
  // shooting
#ifdef USE_XBOX_CONTROLS
  frc2::Trigger([this]{return controller1.GetRightTriggerAxis() > 0.5;})
#else
  frc2::JoystickButton(&control1, 1)
#endif
  .WhileActiveContinous(frc2::SequentialCommandGroup(
    frc2::RunCommand([this]{
      transportSubsystem.enableInnerBelt();
      transportSubsystem.enableOuterBelt();
    }, {&transportSubsystem}),
    frc2::WaitUntilCommand([this] {return transportSubsystem.hasInnerBall();}).WithTimeout(2.0_s)
  ));
  
  // auto-climbing
#ifdef USE_XBOX_CONTROLS 
  (frc2::JoystickButton(&controller2, frc::XboxController::Button::kStart) &&
    frc2::JoystickButton(&controller2, frc::XboxController::Button::kBack))
#else
  frc2::JoystickButton(&control2, 8)
#endif
  .WhenActive(frc2::ConditionalCommand(
    frc2::SequentialCommandGroup(
    frc2::FunctionalCommand(
      [this]{climberSubsystem.retractLower();},
      []{},
      [](bool){},
      [this]{return climberSubsystem.isRetracted();}
    ),
    frc2::WaitCommand(0.5_s),
    frc2::InstantCommand([this]{climberSubsystem.extendUpper();}),
    frc2::WaitCommand(3.0_s),
    frc2::InstantCommand([this]{climberSubsystem.extendLower();}),
    frc2::WaitCommand(3.0_s),
    frc2::InstantCommand([this]{climberSubsystem.retractLower();
                                climberSubsystem.retractUpper();})
    ),
    frc2::InstantCommand([]{}),
    [this]{return !climberSubsystem.isRetracted();}
  ));

}

frc2::Command* RobotContainer::autonomousCommand() {
  return autoChooser.GetSelected();
}

