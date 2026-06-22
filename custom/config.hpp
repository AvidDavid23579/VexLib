#pragma once
#include "pros/motor_group.hpp"
#include "subsystems.hpp"
#include "pros/imu.hpp"

extern pros::Controller master;

extern pros::MotorGroup LeftMotors;
extern pros::MotorGroup RightMotors;

extern pros::Motor PivotMotor;
extern pros::Motor ClawMotor;

extern pros::Imu driveIMU;

extern DifferentialDrive drive;
extern OneDOFArm arm;


