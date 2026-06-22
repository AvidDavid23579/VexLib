#include "utils.hpp"

#include <cmath>
#include <numeric>

#include "const.hpp"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"

double wrapAngle(double x)
{
    x = fmod(x + 180.0, 360.0);
    if (x < 0)
    {
        x += 360.0;
    }
    return x - 180.0;
}

double angleError(double target, double current)
{
    return wrapAngle(target - current);
}

int joystickToVoltage(int value)
{
    return value * MAX_MILIVOLTS / MAX_JOYSTICK;
}

double degreesToCm(double degrees, double wheel_diam)
{
    return (degrees / 360) * wheel_diam * M_PI;
}

double averageVector(std::vector<double>& v)
{
    if (v.empty())
        return 0.0;

    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    return sum / v.size();
}

void imuInit(pros::Imu& IMU)
{
    IMU.reset();

    while (IMU.is_calibrating())
    {
        pros::delay(20);
    }
}
