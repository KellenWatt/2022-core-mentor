#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/TransportSubsystem.h"

#include <frc/Timer.h>

class TransportCommand : public frc2::CommandHelper<frc2::CommandBase, TransportCommand> {
public:
    TransportCommand(TransportSubsystem* system);

    void Execute();
private:
    TransportSubsystem* subsystem;
    frc::Timer ballTimer;
};