#include "commands/DriveCommand.h"

#ifdef USE_XBOX_CONTROLS
DriveCommand::DriveCommand(DriveSubsystem *drive, frc::XboxController *stick) :
#else
DriveCommand::DriveCommand(DriveSubsystem *drive, frc::Joystick *stick) :
#endif
        driveSubsystem(drive), control(stick) {
    AddRequirements(driveSubsystem);
}

void DriveCommand::Execute() {
#ifdef USE_XBOX_CONTROLS
    driveSubsystem->drive(0.9*control->GetLeftX(), 0.9*-control->GetLeftY(), 0.9*control->GetRightX());
#else
    driveSubsystem->drive(control->GetX(), -control->GetY(), control->GetZ());
#endif
}