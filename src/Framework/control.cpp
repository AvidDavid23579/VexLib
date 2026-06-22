#include "control.hpp"
#include "assistive_teleop.hpp"
#include "const.hpp"

#include <algorithm>
#include <cmath>

/*

Slew Delimitation Class

*/
SlewLimiter::SlewLimiter(double accel, double decel)
{
    this->accel = accel;
    this->decel = decel;
    prev = 0;
}

double SlewLimiter::update(double input)
{
    double delta = input - prev;

    if ((input * prev >= 0) && std::abs(input) > std::abs(prev))
    {
        if (delta > accel)
            delta = accel;
        else if (delta < -accel)
            delta = -accel;
    }
    else
    {
        if (delta > decel)
            delta = decel;
        else if (delta < -decel)
            delta = -decel;
    }

    prev += delta;

    return prev;
}

// Update the SlewLimiter
void SlewLimiter::reset(double value)
{
    prev = value;
}

// Bang Bang control
BangBang::BangBang(double setpoint, double correction) : m_setpoint(setpoint), m_correction(correction) {}

double BangBang::update(double variable)
{
    double error = m_setpoint - variable;

    error = deadband(error);

    if (error > 0)
    {
        return m_correction;
    }
    else if (error < 0)
    {
        return -m_correction;
    }
    else
        return 0;
}

/*

PID Class

*/
PID::PID(double p, double i, double d, double iZone, double iMax, double outputLimit)
    : kP(p), kI(i), kD(d), integral(0), integralZone(iZone), integralLimit(iMax), prevMeasurement(0), outputLimit(outputLimit)
{
}

double PID::update(double target, double current)
{
    double error = target - current;

    if (std::abs(error) < integralZone || integralZone == 0)
    {
        integral += error;
    }
    else
        integral = 0;

    integral = std::clamp(integral, -integralLimit, integralLimit);

    double derivative = current - prevMeasurement;

    prevMeasurement = current;

    double output = kP * error + kI * integral - kD * derivative;

    return std::clamp(output, -outputLimit, outputLimit);
}

void PID::reset()
{
    integral = 0;
    prevMeasurement = 0;
}

/*

FeedForward Class

*/

FeedForward::FeedForward(double kS, double kV, double kA)
{
    this->kS = kS;
    this->kV = kV;
    this->kA = kA;
}

double FeedForward::sign(double velocity)
{
    if (velocity > 0.0)
        return 1.0;
    else if (velocity < 0.0)
        return -1.0;
    return 0.0;
}

double FeedForward::calculate(double velocity, double accel)
{
    if (velocity == 0.0 and accel == 0.0)
        return 0.0;

    double output = 0.0;
    output += kS * sign(velocity) + kV * velocity + kA * accel;

    return output;
}

double complementary(double longterm, double shortterm, double coefficient)
{
    return coefficient * longterm + (1 - coefficient) * shortterm;
}

EMAFilter::EMAFilter(double time) : time(time), y(0), initialised(false) {}

double EMAFilter::update(double x)
{
    double alpha = LOOP_DELAY / (time + LOOP_DELAY);   

    if (!initialised)
    {
        y = x;
        initialised = true;
    }
    else
    {
        y = alpha * x + (1 - alpha) * y;
    }
    return y;
}

void EMAFilter::reset(double value)
{
    y = value;
    initialised = false;
}