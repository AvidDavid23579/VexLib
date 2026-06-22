#include "main.h"
#include "custom/config.hpp"

void opcontrol()
{
    while (true)
    {
        intake.intakeControl();

        pros::delay(LOOP_DELAY);
    }
}