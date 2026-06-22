#include "auton.hpp"

AutonDifferentialDrive::AutonDifferentialDrive(pros::MotorGroup& LeftMotors, pros::MotorGroup& RightMotors)
    : LeftMotors(LeftMotors), RightMotors(RightMotors)
{
}

void AutonDifferentialDrive::autonDrive(int velocity, int angularVelocity) {}