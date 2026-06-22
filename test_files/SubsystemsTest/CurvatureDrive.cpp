#include "main.h"
#include "config.hpp"

void opcontrol()
{
    while (true)
    {
        drive.curvature();

        pros::delay(LOOP_DELAY);
    }
}