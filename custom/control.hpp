#pragma once

// Slew Delimitation
// Limits Acceleration to avoid high jerk
class SlewLimiter
{
  private:
    double accel;
    double decel;
    double prev;

  public:
    SlewLimiter(double accel = 5, double decel = 10);

    double update(double input);
    void reset(double value = 0);
};

class BangBang
{
  private:
    double m_setpoint;
    double m_correction;

  public:
    BangBang(double setpoint, double correction);

    double update(double variable);
};

// PID controller
class PID
{
  private:
    double kP, kI, kD;
    double integral;
    double integralZone;
    double integralLimit;
    double prevMeasurement;
    double outputLimit;

  public:
    PID(double p, double i, double d, double iZone, double iMax, double outputLimit);

    double update(double target, double current);
    bool isSettled();
    void reset();
};

// FeedForward controller
class FeedForward
{
  private:
    double kS, kV, kA;

    static double sign(double velocity);

  public:
    FeedForward(double kS, double kV, double kA);

    double calculate(double velocity, double accel);
};

// Complementary Filter
double complementary(double longterm, double shortterm, double coefficient);

class EMAFilter
{
  private:
    double time;
    double y;
    bool initialised;

  public:
    EMAFilter(double time);

    double update(double input);
    void reset(double value = 0);
};