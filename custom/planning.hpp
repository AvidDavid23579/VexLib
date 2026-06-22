#pragma once
#include <vector>
#include "const.hpp"

struct ProfileConstraints
{
    double maxVelocity;
    double maxAcceleration;
};

struct ProfileState
{
    double position;
    double velocity;
};

class TrapezoidalProfile
{
  private:
    ProfileConstraints m_constraints;
    ProfileState m_initial{};
    ProfileState m_goal{};

    double m_direction = 1.0;
    double m_t_accel = 0.0;
    double m_t_cruise = 0.0;
    double m_t_decel = 0.0;
    double m_totalTime = 0.0;
    bool m_generated = false;

  public:
    TrapezoidalProfile(ProfileConstraints constraints);

    void generate(ProfileState initial, ProfileState goal);
    ProfileState sample(double t);
    bool isFinished(double t);
    double totalTime();
};

/*

Path Planning Class

*/

class CubicHermitePathPlanner
{
  private:
    std::vector<WaypointVel> m_waypoints;

    // Cubic Hermite basis functions
    static double h00(double t)
    {
        return 2 * t * t * t - 3 * t * t + 1;
    }
    static double h10(double t)
    {
        return t * t * t - 2 * t * t + t;
    }
    static double h01(double t)
    {
        return -2 * t * t * t + 3 * t * t;
    }
    static double h11(double t)
    {
        return t * t * t - t * t;
    }

    // Derivatives (for heading)
    static double dh00(double t)
    {
        return 6 * t * t - 6 * t;
    }
    static double dh10(double t)
    {
        return 3 * t * t - 4 * t + 1;
    }
    static double dh01(double t)
    {
        return -6 * t * t + 6 * t;
    }
    static double dh11(double t)
    {
        return 3 * t * t - 2 * t;
    }

    Pose sampleSegment(int i, double t) const;

  public:
  CubicHermitePathPlanner();

  void addWaypoint(const WaypointVel& wp);
  void clearWaypoints();
  WaypointVel fromHeading(double x, double y, double heading, double speed = 1.0);
  std::vector<Pose> generate(int samplesPerSegment = 50) const;
};