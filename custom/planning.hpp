#pragma once
#include <vector>

#include "const.hpp"

struct TrapezoidalProfileConstraints {
    double maxVelocity;
    double maxAcceleration;
};

struct TrapezoidalProfileState {
    double position;
    double velocity;
    double acceleration;
};

class TrapezoidalProfile {
   private:
    TrapezoidalProfileConstraints m_constraints;
    TrapezoidalProfileState m_initial{};
    TrapezoidalProfileState m_goal{};

    double m_direction = 1.0;
    double m_t_accel = 0.0;
    double m_t_cruise = 0.0;
    double m_t_decel = 0.0;
    double m_totalTime = 0.0;
    bool m_generated = false;

   public:
    TrapezoidalProfile(TrapezoidalProfileConstraints constraints);

    void generate(TrapezoidalProfileState initial, TrapezoidalProfileState goal);
    TrapezoidalProfileState sample(double t);
    bool isFinished(double t);
    double totalTime();
};

struct SCurveConstraints {
    double maxVelocity;
    double maxAcceleration;
    double maxJerk;
};

struct SCurveState {
    double position;
    double velocity;
    double acceleration;
    double jerk;
};

struct SCurveSegment {
    double t_start;
    double duration;

    double p0;
    double v0;
    double a0;

    double jerk;
};

class SCurveProfile {
   private:
    SCurveConstraints m_constraints;
    SCurveState m_initial{};
    SCurveState m_goal{};
    double m_time_jerk;
    double m_time_accel;
    double m_time_cruise;

    double m_a_peak;
    double m_v_peak;

    double m_total_time;
    std::vector<SCurveSegment> m_segments;

    double m_direction = 1.0;

   public:
    SCurveProfile(SCurveConstraints constraints);

    void generate(SCurveState initial, SCurveState goal);
    std::vector<SCurveSegment> build_segments();
    std::tuple<double, double, double> advance(double p, double v, double a, double jerk, double dt);
};

/*

Path Planning Class

*/

class CubicHermitePathPlanner {
   private:
    std::vector<WaypointVel> m_waypoints;

    // Cubic Hermite basis functions
    static double h00(double t) { return 2 * t * t * t - 3 * t * t + 1; }
    static double h10(double t) { return t * t * t - 2 * t * t + t; }
    static double h01(double t) { return -2 * t * t * t + 3 * t * t; }
    static double h11(double t) { return t * t * t - t * t; }

    // Derivatives (for heading)
    static double dh00(double t) { return 6 * t * t - 6 * t; }
    static double dh10(double t) { return 3 * t * t - 4 * t + 1; }
    static double dh01(double t) { return -6 * t * t + 6 * t; }
    static double dh11(double t) { return 3 * t * t - 2 * t; }

    Pose sampleSegment(int i, double t) const;

   public:
    CubicHermitePathPlanner();

    void addWaypoint(const WaypointVel& wp);
    void clearWaypoints();
    WaypointVel fromHeading(double x, double y, double heading, double speed = 1.0);
    std::vector<Pose> generate(int samplesPerSegment = 50) const;
};