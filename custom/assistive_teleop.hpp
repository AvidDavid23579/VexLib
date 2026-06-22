#pragma once
#include "control.hpp"
#include "pros/imu.hpp"

double deadband(double value, double threshold = 5);
/*

Creates a zone of dead values [-threshold, threshold]. If input > threshold or
input < threshold, input = 0.

Uses linear interpolation to maintain continuity.
*/

double shapeInput(double input, double a = 0.5);
/*

Uses a cubic polynomial to shape the input such that it's smoother for the driver to control.

*/

class HeadingHold

/*

Maintains heading in autonomous and assistive teleOp

*/ 

{
  private:
    pros::Imu& imu;
    PID controller;

    double targetHeading;
    bool enabled;

  public:
    HeadingHold(pros::Imu& imu, PID pid);

    double update(double throttleInput, double steerInput);
    /*
    
    Updates HeadingHold PID every loop

    */
    void setTarget(double target);
    /*
    
    Sets a target angle
    
    */
    void reset();
};