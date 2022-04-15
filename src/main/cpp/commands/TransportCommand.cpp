#include "commands/TransportCommand.h"

TransportCommand::TransportCommand(TransportSubsystem* system): subsystem(system) {
    AddRequirements(system);
    ballTimer.Start();
}

void TransportCommand::Execute() {
    if(!subsystem->hasOuterBall() || !subsystem->hasInnerBall()) {
      subsystem->enableOuterBelt();
    } else {
      subsystem->disableOuterBelt();
    }

    if(!subsystem->hasInnerBall()) {
      subsystem->enableInnerBelt();
      ballTimer.Reset();
    } else if(ballTimer.HasElapsed(0.15_s)) {
        subsystem->disableInnerBelt();
    }
}