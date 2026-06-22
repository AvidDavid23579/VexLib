#include "assistive_teleop.hpp"
#include "const.hpp"
#include "utils.hpp"


#include <cmath>

// Joystick Deadband
// Makes inputs near zero turn into zero to eliminate small movements

double deadband(double value, double threshold)
{
    if (std::abs(value) < threshold)
        return 0;

    // Linear interpolation to ensure continuity

    double sign = (value > 0) ? 1.0 : -1.0;
    double t = (std::abs(value) - threshold) / (MAX_JOYSTICK - threshold);
    return sign * std::lerp(0.0, MAX_JOYSTICK, t);
}

// Cubic Input Shaping
// Shapes inputs to add weight to high speeds and control in lower speeds

double shapeInput(double input, double a) // a in [0,1]
{
    double x = input / MAX_JOYSTICK;
    double shaped = a * x * x * x + (1 - a) * x;
    return shaped * 127;
}

// Heading Hold
HeadingHold::HeadingHold(pros::Imu& imu, PID pid) : imu(imu), controller(pid), targetHeading(0), enabled(false) {}

double HeadingHold::update(double throttleInput, double steerInput)
{

    const double deadband = 10;

    double heading = imu.get_rotation();

    if ((std::abs(steerInput) > deadband))
    {
        controller.reset();
        return 0.0;
    }

    if (!enabled)
    {
        targetHeading = heading;
        controller.reset();
        enabled = true;
    }

    double error = wrapAngle(targetHeading - heading);

    double correction = controller.update(0, -error);

    return correction;
}

void HeadingHold::setTarget(double target)
{
    targetHeading = target;
}

void HeadingHold::reset()
{
    targetHeading = imu.get_rotation();
    controller.reset();
    enabled = false;
}
