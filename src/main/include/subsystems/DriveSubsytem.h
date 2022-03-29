#pragma once

#include "Constants.h"

#include <functional>

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/DigitalInput.h>
#include <AHRS.h>
#include <frc/SPI.h>

#include <cmath>

class DriveSubsystem : public frc2::SubsystemBase {
public:
    DriveSubsystem();
    DriveSubsystem(double deadband);
    DriveSubsystem(double deadband, bool squareXInput, bool squareYInput, bool squareRotInput);
    ~DriveSubsystem() noexcept {}

    void drive(double xSpeed, double ySpeed, double rotation, bool correctRotation = true);
    // turn without resetting the gyro. Useful for rotating to a 
    // specific angle.
    void freeTurn(double speed);

    double distance();
    void resetDistance();

    void resetGyro();
    double orientation();

    bool seesLine();

    void squareXInput(bool square = false) {squareX = square;}
    void squareYInput(bool square = false) {squareY = square;}
    void squareRotInput( bool square = false) {squareRot = square;}
private:
    double deadband;

    double realX = 0;
    double realY = 0;
    double realZ = 0;

    bool squareX;
    bool squareY;
    bool squareRot;

    rev::CANSparkMax frontLeft{constants::drive::FRONT_LEFT_MOTOR, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax rearLeft{constants::drive::REAR_LEFT_MOTOR, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax frontRight{constants::drive::FRONT_RIGHT_MOTOR, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax rearRight{constants::drive::REAR_RIGHT_MOTOR, rev::CANSparkMax::MotorType::kBrushless};
    frc::MecanumDrive mecanumDrive{frontLeft, rearLeft, frontRight, rearRight}; 

    AHRS gyro{frc::SPI::Port::kMXP};    

    frc::DigitalInput lineSensor{constants::drive::LINE_SENSOR};


    rev::SparkMaxRelativeEncoder frontLeftEnc = frontLeft.GetEncoder();
    rev::SparkMaxRelativeEncoder rearLeftEnc = rearLeft.GetEncoder();
    rev::SparkMaxRelativeEncoder frontRightEnc = frontRight.GetEncoder();
    rev::SparkMaxRelativeEncoder rearRightEnc = rearRight.GetEncoder();
    // Basic functional iterator. Accumulation can be done by 
    // capturing the accumulator in a lambda.
    void eachMotor(std::function<void(rev::CANSparkMax&)> f) {
        f(frontLeft);
        f(rearLeft);
        f(frontRight);
        f(rearRight);
    }

    void eachEncoder(std::function<void(rev::SparkMaxRelativeEncoder&)> f) {
        f(frontLeftEnc);
        f(rearLeftEnc);
        f(frontRightEnc);
        f(rearRightEnc);
    }

    double ramp(double actual, double expected, double adjRate) {
        if((expected > 0 && actual < expected) || (expected < 0 && actual > expected)) {
            return actual + ((expected - actual) * 0.1 * adjRate);
        }
        return expected;
    }

};