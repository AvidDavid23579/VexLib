#pragma once

// Slew Delimitation
// Limits Acceleration to avoid high jerk
class SlewLimiter {
   private:
    double dt;
    double max_accel;
    double max_decel;
    double prev;

   public:
    SlewLimiter(double dt, double max_accel = 5, double max_decel = 10);

    double update(double input);
    void reset(double value = 0);
};

class JerkSlewLimiter {
   private:
    double m_dt;
    double m_max_jerk;
    double m_max_accel;
    double m_max_decel;
    double m_velocity = 0.0;
    double m_accel = 0.0;

   public:
    JerkSlewLimiter(double dt, double max_jerk, double max_accel = 5, double max_decel = 10);

    double update(double target_velocity);
    void reset(double value = 0);
};

class BangBang {
   private:
    double m_setpoint;
    double m_correction;
    double m_prev;

   public:
    BangBang(double setpoint, double correction);

    double update(double variable);
};

// PID controller
class PID {
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
class FeedForward {
   private:
    double kS, kV, kA;

    static double sign(double velocity);

   public:
    FeedForward(double kS, double kV, double kA);

    double calculate(double velocity, double accel);
};
