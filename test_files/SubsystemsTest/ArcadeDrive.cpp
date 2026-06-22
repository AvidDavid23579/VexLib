#include "main.h"
#include "custom/config.hpp"

void opcontrol()
{
    while (true)
    {
        drive.arcade();

        pros::delay(LOOP_DELAY);
    }
}