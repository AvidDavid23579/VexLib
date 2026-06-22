#include "state_estimation.hpp"
#include "utils.hpp"
#include "config.hpp"

// Constructor
EncoderOdometry::EncoderOdometry(
    pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors, double wheelDiam, double trackWidth, double gearRatio)
    : m_leftMotorGroup(leftMotors), m_rightMotorGroup(rightMotors), m_wheelDiam(wheelDiam), m_trackWidth(trackWidth),
      m_gearRatio(gearRatio)
{
    m_lastLeft = averageVector(m_leftMotorGroup.get_position_all());
    m_lastRight = averageVector(m_rightMotorGroup.get_position_all());
    m_lastTime = pros::millis();
}

void EncoderOdometry::update()
{
    std::vector<double> leftPositions = m_leftMotorGroup.get_position_all();
    std::vector<double> rightPositions = m_rightMotorGroup.get_position_all();
    double now = pros::millis();

    double avgLeft = averageVector(leftPositions);
    double avgRight = averageVector(rightPositions);
    double imuHeading = wrapAngle(driveIMU.get_rotation()) * M_PI / 180.0;

    double dLeft = avgLeft - m_lastLeft;
    double dRight = avgRight - m_lastRight;
    double dt = (now - m_lastTime) / 1000; // Converts to seconds

    double dL = degreesToCm(dLeft, m_wheelDiam) / m_gearRatio;
    double dR = degreesToCm(dRight, m_wheelDiam) / m_gearRatio;

    double dC = (dL + dR) / 2;
    double dEncoderHeading = (dR - dL) / m_trackWidth; // From encoders

    m_pose.heading = complementary(imuHeading, m_pose.heading + dEncoderHeading, 0.98);

    double dHeading = m_pose.heading - m_lastHeading;

    if (fabs(dHeading) > 1e-6)
    {
        double r = dC / dHeading; // Radius of curvature
        m_pose.x += r * (sin(m_lastHeading + dHeading) - sin(m_lastHeading));
        m_pose.y += r * (-cos(m_lastHeading + dHeading) + cos(m_lastHeading));
    }
    else
    {
        m_pose.x += dC * cos(m_lastHeading + dHeading / 2.0);
        m_pose.y += dC * sin(m_lastHeading + dHeading / 2.0);
    }

    m_velocity = (dt > 0) ? (dC / dt) : 0.0;
    m_velocity = m_velocityFilter.update(m_velocity);

    m_angularVelocity = (dt > 0) ? (dHeading / dt) : 0.0;
    m_angularVelocity = m_velocityFilter.update(m_angularVelocity);

    m_lastLeft = avgLeft;
    m_lastRight = avgRight;
    m_lastTime = now;
    m_lastHeading = m_pose.heading;
}

void EncoderOdometry::reset()
{
    m_pose = {0, 0, 0};

    m_lastLeft = averageVector(m_leftMotorGroup.get_position_all());
    m_lastRight = averageVector(m_rightMotorGroup.get_position_all());
    m_lastTime = pros::millis();
}

void EncoderOdometry::setPose(Pose pose)
{
    m_pose = pose;
    m_lastLeft = averageVector(m_leftMotorGroup.get_position_all());
    m_lastRight = averageVector(m_rightMotorGroup.get_position_all());
    m_lastTime = pros::millis();
}

Pose EncoderOdometry::getPose() const
{
    return m_pose;
}

DifferentialDriveState EncoderOdometry::getVelocity()
{
    return {m_velocity, m_angularVelocity};
}
