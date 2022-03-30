#include "subsystems/ShooterSubsystem.h"

ShooterSubsystem::ShooterSubsystem() : ShooterSubsystem(constants::shooter::FLYWHEEL_RPM) {}

ShooterSubsystem::ShooterSubsystem(double rpm) :flywheelRPM(rpm) {
    flywheelPID.SetP(constants::shooter::pid::P);
    flywheelPID.SetI(constants::shooter::pid::I);
    flywheelPID.SetFF(constants::shooter::pid::FF);
    flywheelPID.SetD(constants::shooter::pid::D);
    disableFlywheel();
}

void ShooterSubsystem::enableFlywheel() {
    flywheelPID.SetReference(flywheelRPM, rev::CANSparkMax::ControlType::kVelocity);
}

void ShooterSubsystem::disableFlywheel() {
    flywheelPID.SetReference(0, rev::CANSparkMax::ControlType::kVelocity);
}


void ShooterSubsystem::setSpeed(double rpm) { 
    flywheelRPM = rpm;
    flywheelPID.SetReference(rpm, rev::CANSparkMax::ControlType::kVelocity);
}

void ShooterSubsystem::resetSpeed() {
    setSpeed(constants::shooter::FLYWHEEL_RPM);
}

double ShooterSubsystem::getSpeed() {
    return flywheelRPM;
}

double ShooterSubsystem::getRealSpeed() {
    return flywheel.GetEncoder().GetVelocity();
}
