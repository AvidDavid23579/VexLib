#pragma once
#include "cmath"
#include "const.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "utils.hpp"

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

// Odometry
class EncoderOdometry
{
  private:
    pros::MotorGroup& m_leftMotorGroup;
    pros::MotorGroup& m_rightMotorGroup;
    double m_wheelDiam; // cm
    double m_gearRatio;
    double m_lastLeft = averageVector(m_leftMotorGroup.get_position_all());
    double m_lastRight = averageVector(m_rightMotorGroup.get_position_all());
    double m_lastTime = pros::millis();
    double m_trackWidth;
    double m_lastHeading = 0.0;
    double m_velocity;
    double m_angularVelocity;
    EMAFilter m_velocityFilter{0.1};
    Pose m_pose;

  public:
    EncoderOdometry(
        pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors, double wheelDiam, double trackWidth, double gearRatio = 1.0);

    void update();
    void reset();
    void setPose(Pose pose);
    Pose getPose() const;
    DifferentialDriveState getVelocity();
};

class TwoWheelOdometry
{
  private:
    pros::Rotation& m_parallel;
    pros::Rotation& m_perpendicular;

    double m_wheel_circ;

    double m_parallel_offset;
    double m_perpendicular_offset;

    double m_x = 0.0;
    double m_y = 0.0;
    double m_theta = 0.0;

    double m_last_parallel = 0.0;
    double m_last_perpendicular = 0.0;
    double m_last_theta = 0.0;

    double m_last_time = 0.0;

    bool m_initialized = false;

  public:
    TwoWheelOdometry(pros::Rotation& parallel,
                     pros::Rotation& perpendicular,
                     pros::Imu& imu,
                     double wheel_Diam,
                     double parallel_offset,
                     double perpendicular_offset);

    void update();
    void reset();
    void setPose();
    Pose getPose();
    double getVelocity();
};
