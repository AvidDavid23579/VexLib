#pragma once
#include "pros/motor_group.hpp"
#include "const.hpp"
#include "control.hpp"
#include "assistive_teleop.hpp"
#include "pros/imu.hpp"

class DifferentialDrive
{
  private:
    pros::MotorGroup& leftMotors;
    pros::MotorGroup& rightMotors;
    pros::Imu& driveIMU;

    SlewLimiter throttlelimit{250, 500};
    SlewLimiter steerlimit{250, 500}; // joystick units per second

    PID headingPID;
    HeadingHold headinghold{driveIMU, headingPID};

  public:
    DifferentialDrive(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors, pros::Imu& driveIMU, PID headingPID);

    void tank(bool useHeadingHold = false);
    void arcade(bool useHeadingHold = false);
    void curvature(bool useHeadingHold = false);
};

class Intake
{
  private:
    pros::MotorGroup& IntakeMotors;

  public:
    Intake(pros::MotorGroup& IntakeMotors);

    void intakeControl(int voltage = MAX_MILIVOLTS);
};

enum class ClawState
{
    Open,
    Closed
};

class OneDOFArm
{
  private:
    pros::Motor& m_pivot;
    pros::Motor& m_claw;
    double m_gear_ratio;
    ClawState m_claw_state;
    SlewLimiter armLimiter{5, 10};

  public:
    OneDOFArm(pros::Motor& pivot, pros::Motor& claw, double m_gear_ratio = 1);

    void armInitialise();
    void armControl();
    void clawControl();
    void setArmPosition(double deg);
};