#include "main.h"
#include "custom/config.hpp"

void opcontrol()
{
    while (true)
    {
        arm.armControl();
        arm.clawControl();

        pros::delay(LOOP_DELAY);
    }
}