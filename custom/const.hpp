#pragma once
#include <cmath>

constexpr double DEG_TO_RAD = M_PI / 180;
constexpr double MAX_JOYSTICK = 127.0;
constexpr double MAX_MILIVOLTS = 12000.0;
constexpr double LOOP_DELAY = 20.0; // ms

struct Pose
{
    double x;
    double y;
    double heading;
};

struct DifferentialDriveState
{
    double velocity;
    double angularVelocity;
};

struct WaypointVel {
    Pose pose;
    double dx, dy;         // tangent vector    
};

struct WaypointAccel {
    Pose pose;
    double dx, dy;         // tangent vector
    double ddx, ddy;       // curvature vector
};

static constexpr int   ARM_SPEED       = 200;   // RPM
static constexpr int   CLAW_OPEN_POS   = 0;     // degrees
static constexpr int   CLAW_CLOSED_POS = 720;    
static constexpr int   CLAW_HOLD_MILIVOLT  = 2000; 