#pragma once
#include "pros/imu.hpp"

double wrapAngle(double x);
/*

Wraps the angle of rotation to always be in [0, 360]

*/

double angleError(double target, double current);
/*

Returns the absolute value of the angle error.

Already takes into account angle wrapping.

e.g:

angleError(350, 10) = 20
*/

int joystickToVoltage(int value);
/*

Maps [-127, 127] to [-12000, 12000]

*/

double degreesToCm(double degrees, double wheel_diam);
/*

Given wheel diameter and degrees traveled, return distance in cm.

*/

double averageVector(const std::vector<double>& v);
/*

Returns the average value of entries of a vector

*/

void imuInit(pros::Imu& IMU);
/*

Initialises IMU sensor at the start

*/