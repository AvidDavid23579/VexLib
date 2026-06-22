#pragma once
#include "pros/motor_group.hpp"

class AutonDifferentialDrive
{
  private:
    pros::MotorGroup& LeftMotors;
    pros::MotorGroup& RightMotors;

  public:
    AutonDifferentialDrive(pros::MotorGroup& LeftMotors, pros::MotorGroup& RightMotors);

    void autonDrive(int velocity, int angularVelocity);
};
