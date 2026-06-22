#include "main.h"
#include "custom/config.hpp"

void opcontrol()
{
    while (true)
    {
        drive.tank();

        pros::delay(LOOP_DELAY);
    }
}