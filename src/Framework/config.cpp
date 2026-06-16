#include "custom/config.hpp"
#include "custom/subsystems.hpp"

int IMU_PORT = 9;

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup LeftMotors({1,11});
pros::MotorGroup RightMotors({-10, -20});

pros::Imu driveIMU(IMU_PORT);

PID headingPID(1.0, 0.0, 0.0, 0.0, 0.0, MAX_MILIVOLTS);
DifferentialDrive drive(LeftMotors, RightMotors, driveIMU, headingPID);
OneDOFArm arm(PivotMotor, ClawMotor);

