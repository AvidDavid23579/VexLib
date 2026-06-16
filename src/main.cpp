#include "main.h"
#include "custom/config.hpp"
#include "custom/utils.hpp"


void initialize() {
    imuInit(driveIMU);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol()
{
    while (true)
    {
        drive.curvature(false);

        pros::delay(LOOP_DELAY);
    }
}