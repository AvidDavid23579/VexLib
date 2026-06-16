#include "custom/subsystems.hpp"
#include "custom/const.hpp"
#include "custom/utils.hpp"
#include "custom/config.hpp"
#include "custom/assistive_teleop.hpp"
#include <cmath>

/*

Differential Drive Class

*/

DifferentialDrive::DifferentialDrive(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors, pros::Imu& driveIMU, PID headingPID)
    : leftMotors(leftMotors), rightMotors(rightMotors), driveIMU(driveIMU), headingPID(headingPID)
{
}

// Tank Drive

void DifferentialDrive::tank(bool useHeadingHold)
{
    int left = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    left = shapeInput(left);
    right = shapeInput(right);

    left = throttlelimit.update(left);
    right = steerlimit.update(right);

    if (useHeadingHold)
    {
        double throttle = (left + right) / 2.0;
        double steer = (right - left) / 2.0;

        double correction = headinghold.update(throttle, steer);

        left = std::clamp(left + correction, -127.0, 127.0);
        right = std::clamp(right - correction, -127.0, 127.0);
    }

    leftMotors.move_voltage(joystickToVoltage(left));
    rightMotors.move_voltage(joystickToVoltage(right));
}

// Arcade Drive

void DifferentialDrive::arcade(bool useHeadingHold)
{
    int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int steer = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    throttle = shapeInput(throttle);
    steer = shapeInput(steer);

    throttle = throttlelimit.update(throttle);
    steer = steerlimit.update(steer);

    int left = throttle + steer;
    int right = throttle - steer;

    double maxMagnitude = fmax(std::abs(left), std::abs(right));

    if (maxMagnitude > MAX_JOYSTICK)
    {
        left = left * MAX_JOYSTICK / maxMagnitude;
        right = right * MAX_JOYSTICK / maxMagnitude;
    }

    if (useHeadingHold)
    {
        double correction = headinghold.update(throttle, steer);

        left = std::clamp(left + correction, -127.0, 127.0);
        right = std::clamp(right - correction, -127.0, 127.0);
    }

    leftMotors.move_voltage(joystickToVoltage(left));
    rightMotors.move_voltage(joystickToVoltage(right));
}

// Curvature Drive

void DifferentialDrive::curvature(bool useHeadingHold)
{
    int throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int steer = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    bool quickTurn = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    throttle = shapeInput(throttle);
    steer = shapeInput(steer);

    throttle = throttlelimit.update(throttle);
    steer = steerlimit.update(steer);

    double left, right;

    if (quickTurn || throttle == 0)
    {
        left = steer;
        right = -steer;
    }

    else
    {
        double steerNorm = (double)steer / MAX_JOYSTICK;
        left = throttle + std::abs(throttle) * steerNorm;
        right = throttle - std::abs(throttle) * steerNorm;
    }

    double maxMagnitude = fmax(std::abs(left), std::abs(right));

    if (maxMagnitude > MAX_JOYSTICK)
    {
        left = left * MAX_JOYSTICK / maxMagnitude;
        right = right * MAX_JOYSTICK / maxMagnitude;
    }

    if (useHeadingHold && !quickTurn && throttle != 0)
    {
        double correction = headinghold.update(throttle, steer);

        left = std::clamp(left + correction, -MAX_JOYSTICK, MAX_JOYSTICK);
        right = std::clamp(right - correction, -MAX_JOYSTICK, MAX_JOYSTICK);
    }

    leftMotors.move_voltage(joystickToVoltage(left));
    rightMotors.move_voltage(joystickToVoltage(right));
}

/*

Intake Class

*/

Intake::Intake(pros::MotorGroup& IntakeMotors) : IntakeMotors(IntakeMotors) {}

// Intake Control

void Intake::intakeControl(int voltage)
{
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
    {
        IntakeMotors.move_voltage(voltage);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
    {
        IntakeMotors.move_voltage(-voltage);
    }
    else
    {
        IntakeMotors.move_voltage(0);
    }
}

/*

One Degree of Freedom Claw Class

*/

OneDOFArm::OneDOFArm(pros::Motor& pivot, pros::Motor& claw, double gear_ratio)
    : m_pivot(pivot), m_claw(claw), m_gear_ratio(gear_ratio), m_claw_state(ClawState::Open)
{
}

void OneDOFArm::armInitialise()
{
    m_pivot.tare_position();
    m_claw.tare_position();

    m_pivot.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    m_claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    m_claw.move_absolute(CLAW_OPEN_POS, ARM_SPEED);
    m_claw_state = ClawState::Open;
}

void OneDOFArm::armControl()
{
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
    {
        m_pivot.move_velocity(ARM_SPEED);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        m_pivot.move_velocity(-ARM_SPEED);
    }
    else
    {
        m_pivot.move_velocity(0);
    }
}

void OneDOFArm::clawControl()
{
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    {
        if (m_claw_state == ClawState::Open)
        {
            m_claw.move_absolute(CLAW_CLOSED_POS, ARM_SPEED);
            m_claw_state = ClawState::Closed;
        }
        else
        {
            m_claw.move_absolute(CLAW_OPEN_POS, ARM_SPEED);
            m_claw_state = ClawState::Open;
        }
    }
}

void OneDOFArm::setArmPosition(double deg)
{
    double motorDeg = deg * m_gear_ratio;
    m_pivot.move_absolute(motorDeg, ARM_SPEED);
}