#include "commands/DriveUntilCommand.h"

DriveUntilCommand::DriveUntilCommand(DriveSubsystem* system, bool forward, std::function<bool(void)> condition) 
        : drive(system), forward(forward), condition(condition) {
    AddRequirements(system);
}

void DriveUntilCommand::Execute() {
    if(forward) {
        drive->drive(0, 0.5, 0, false);
    } else {
        drive->drive(0, -0.5, 0, false);
    }
}

void DriveUntilCommand::End(bool) {
    drive->drive(0,0,0);
}

bool DriveUntilCommand::IsFinished() {
    return condition();
}