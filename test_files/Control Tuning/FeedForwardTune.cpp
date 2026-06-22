#include "main.h"
#include "pros/misc.h"
#include "custom/auton.hpp"
#include "custom/config.hpp"
#include <fstream>


//* adding manual drive and fix <fstream>//*


AutonDifferentialDrive auton_drive(LeftMotors, RightMotors);


void autonomous() {
    /*
    int TEST_VOLTAGE;
    auton_drive.autonDrive(TEST_VOLTAGE, TEST_VOLTAGE);
    */
    // Create CSV file
    std::ofstream logFile(
        "/usd/FeedForwardData.csv"
    );

    // CSV header
    logFile
        << "time,velocity,voltage\n";
    int voltage = 0;
    int startTime = pros::millis();
    while (voltage <= 12000) {
        // Slowly ramp voltage
        voltage += 200;

        auton_drive.autonDrive(
            voltage,
            voltage
        );

        // Average velocity
        double velocity =
            (
                LeftMotors.get_actual_velocity() +
                RightMotors.get_actual_velocity()
            ) / 2.0;

        double time =
            (pros::millis() - startTime)
            / 1000.0;

        // Save CSV row
        logFile
            << time << ","
            << velocity << ","
            << voltage << "\n";

        pros::delay(50);
    }
    // Stop robot
    auton_drive.autonDrive(0,0);
    logFile.close();
}

void opcontrol() {
    while (true){
        drive.tank(true);
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            autonomous();
        };
    }
}