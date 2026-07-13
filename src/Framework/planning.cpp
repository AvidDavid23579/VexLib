#include "planning.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

TrapezoidalProfile::TrapezoidalProfile(TrapezoidalProfileConstraints constraints) : m_constraints(constraints) {}

void TrapezoidalProfile::generate(TrapezoidalProfileState initial, TrapezoidalProfileState goal) {
    m_initial = initial;
    m_goal = goal;

    double delta_pos = goal.position - initial.position;
    m_direction = (delta_pos >= 0) ? 1.0 : -1.0;
    double dist = std::abs(delta_pos);

    double vMax = m_constraints.maxVelocity;
    double a = m_constraints.maxAcceleration;

    double v0 = initial.velocity * m_direction;
    double vf = goal.velocity * m_direction;

    m_t_accel = (vMax - v0) / a;
    m_t_decel = (vMax - vf) / a;

    double d_accel = (v0 + vMax) / 2.0 * m_t_accel;
    double d_decel = (vf + vMax) / 2.0 * m_t_decel;

    double d_cruise = dist - d_accel - d_decel;

    if (d_cruise < 0) {
        double v_peak = std::sqrt((2.0 * a * dist + v0 * v0 + vf * vf) / 2.0);
        v_peak = std::min(v_peak, vMax);

        m_t_accel = (v_peak - v0) / a;
        m_t_decel = (v_peak - vf) / a;
        m_t_cruise = 0.0;
    } else {
        m_t_cruise = d_cruise / vMax;
    }

    m_totalTime = m_t_accel + m_t_cruise + m_t_decel;
    m_generated = true;
}

TrapezoidalProfileState TrapezoidalProfile::sample(double t) {
    if (!m_generated) return m_initial;

    t = std::clamp(t, 0.0, m_totalTime);

    double a = m_constraints.maxAcceleration;
    double v0 = m_initial.velocity * m_direction;

    double v_peak = v0 + a * m_t_accel;

    double pos, vel;

    if (t <= m_t_accel) {
        vel = v0 + a * t;
        pos = v0 * t + 0.5 * a * t * t;
        a = a;
    } else if (t <= m_t_accel + m_t_cruise) {
        double dt = t - m_t_accel;
        double d_accel = v0 * m_t_accel + 0.5 * a * m_t_accel * m_t_accel;
        vel = v_peak;
        pos = d_accel + v_peak * dt;
        a = 0;
    } else {
        double dt = t - m_t_accel - m_t_cruise;
        double d_accel = v0 * m_t_accel + 0.5 * a * m_t_accel * m_t_accel;
        double d_cruise = v_peak * m_t_cruise;
        vel = v_peak - a * dt;
        pos = d_accel + d_cruise + v_peak * dt - 0.5 * a * dt * dt;
        a = -a;
    }
    return {m_initial.position + m_direction * pos, m_direction * vel, m_direction * a};
}

bool TrapezoidalProfile::isFinished(double t) { return m_generated && t >= m_totalTime; }

double TrapezoidalProfile::totalTime() { return m_totalTime; }

SCurveProfile::SCurveProfile(SCurveConstraints constraints) : m_constraints(constraints) {}

void SCurveProfile::generate(SCurveState initial, SCurveState goal) {
    double delta_pos = goal.position - initial.position;
    m_direction = (delta_pos >= 0) ? 1.0 : -1.0;
    double distance = std::abs(delta_pos);

    if (distance == 0) {
        m_time_jerk = m_time_accel = m_time_cruise = 0;
        m_a_peak = 0.0;
        m_v_peak = 0.0;
        m_total_time = 0.0;
        m_segments = {};
    }

    double time_jerk_full = m_constraints.maxAcceleration / m_constraints.maxJerk;

    if (m_constraints.maxAcceleration * time_jerk_full > m_constraints.maxVelocity) {
        m_time_jerk = sqrt(m_constraints.maxVelocity / m_constraints.maxJerk);
        m_time_accel = 0.0;
        m_a_peak = m_constraints.maxJerk * m_time_jerk;
    } else {
        m_time_jerk = time_jerk_full;
        m_time_accel = m_constraints.maxVelocity - m_constraints.maxAcceleration - m_time_jerk;
        m_a_peak = m_constraints.maxAcceleration;
    }

    m_v_peak = m_constraints.maxVelocity;
    double time_ramp = 2 * m_time_jerk + m_time_accel;
    double distance_ramp = m_v_peak * time_ramp;

    if (distance_ramp <= distance) {
        m_time_cruise = (distance - distance_ramp) / m_v_peak;
    } else {
        m_time_cruise = 0.0;
        double b = m_constraints.maxAcceleration * time_jerk_full;
        double v_candidate = (-b + sqrt(pow(b, 2)) + 4 * m_constraints.maxAcceleration * distance) / 2;

        if (v_candidate >= m_constraints.maxAcceleration * time_jerk_full) {
            m_v_peak = v_candidate;
            m_time_jerk = time_jerk_full;
            m_time_accel = m_v_peak / m_constraints.maxAcceleration - m_time_jerk;
            m_a_peak = m_constraints.maxAcceleration;
        } else {
            m_v_peak = pow((distance * sqrt(m_constraints.maxJerk) / 2), (2.0 / 3.0));
            m_time_jerk = sqrt(m_v_peak / m_constraints.maxJerk);
            m_time_accel = 0.0;
            m_a_peak = m_constraints.maxJerk * time_jerk_full;
        }

        m_total_time = 4 * m_time_jerk + 2 * m_time_accel + m_time_cruise;

        SCurveProfile::build_segments();
    }
}

void SCurveProfile::build_segments() {
    double j = m_constraints.maxJerk * m_direction;

    std::array<double, 7> durations = {m_time_jerk, m_time_accel, m_time_jerk, m_time_cruise, m_time_jerk, m_time_accel, m_time_jerk};

    std::array<double, 7> jerks = {j, 0.0, -j, 0.0, -j, 0.0, j};

    m_segments.clear();
    m_segments.reserve(7);

    SCurveState state{m_initial.position, 0.0, 0.0, 0.0};

    double timeStart = 0.0;

    for (size_t i = 0; i < durations.size(); ++i) {
        m_segments.push_back({timeStart, durations[i], state.position, state.velocity, state.acceleration, jerks[i]});

        state = advance(state.position, state.velocity, state.acceleration, jerks[i], durations[i]);

        timeStart += durations[i];
    }
}

SCurveState SCurveProfile::advance(double p, double v, double a, double j, double dt) {
    double acceleration = a + j * dt;
    double velocity = v + a * dt + 0.5 * j * dt * dt;
    double position = p + v * dt + 0.5 * a * dt * dt + (1.0 / 6.0) * j * dt * dt * dt;

    return {position, velocity, acceleration, j};
}

SCurveState SCurveProfile::sample(double t) {
    if (m_total_time == 0.0)
        return {
            m_initial.position,
            0.0,
            0.0,
            0.0,
        };

    if (t <= 0.0)
        return {
            m_initial.position,
            0.0,
            0.0,
            0.0,
        };

    if (t >= m_total_time)
        return {
            m_goal.position,
            0.0,
            0.0,
            0.0,
        };

    const SCurveSegment* segment = nullptr;
    double localTime = 0.0;

    for (const auto& seg : m_segments) {
        if (t < seg.t_start + seg.duration) {
            segment = &seg;
            localTime = t - seg.t_start;
            break;
        }
    }

    if (segment == nullptr) {
        segment = &m_segments.back();
        localTime = segment->duration;
    }

    return advance(segment->p0, segment->v0, segment->a0, segment->jerk, localTime);
}
/*

Path Planning Class

*/
CubicHermitePathPlanner::CubicHermitePathPlanner() {}

Pose CubicHermitePathPlanner::sampleSegment(int i, double t) const {
    const WaypointVel& p0 = m_waypoints[i];
    const WaypointVel& p1 = m_waypoints[i + 1];

    double x = h00(t) * p0.pose.x + h10(t) * p0.dx + h01(t) * p1.pose.x + h11(t) * p1.dx;
    double y = h00(t) * p0.pose.y + h10(t) * p0.dy + h01(t) * p1.pose.y + h11(t) * p1.dy;

    double dxdt = dh00(t) * p0.pose.x + dh10(t) * p0.dx + dh01(t) * p1.pose.x + dh11(t) * p1.dx;
    double dydt = dh00(t) * p0.pose.y + dh10(t) * p0.dy + dh01(t) * p1.pose.y + dh11(t) * p1.dy;
    double heading = atan2(dydt, dxdt);

    return {x, y, heading};
}

void CubicHermitePathPlanner::addWaypoint(const WaypointVel& wp) { m_waypoints.push_back(wp); }

void CubicHermitePathPlanner::clearWaypoints() { m_waypoints.clear(); }

WaypointVel CubicHermitePathPlanner::fromHeading(double x, double y, double heading, double speed) {
    return {{x, y, heading}, cos(heading) * speed, sin(heading) * speed};
}

std::vector<Pose> CubicHermitePathPlanner::generate(int samplesPerSegment) const {
    if (m_waypoints.size() < 2) throw std::runtime_error("Need at least 2 waypoints");

    std::vector<Pose> path;

    int numSegments = static_cast<int>(m_waypoints.size()) - 1;
    path.reserve(numSegments * samplesPerSegment + 1);

    for (int i = 0; i < numSegments; ++i) {
        for (int s = 0; s < samplesPerSegment; ++s) {
            double t = static_cast<double>(s) / samplesPerSegment;
            path.push_back(sampleSegment(i, t));
        }
    }
    path.push_back(sampleSegment(numSegments - 1, 1.0));
    return path;
}
