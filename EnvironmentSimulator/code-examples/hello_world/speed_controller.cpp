
#include "esminiLib.hpp"
#include <iostream>

int main(int argc, char **argv)
{
    float acc   = 10;    // initial acceleration
    float speed = 2.0f;  // initial speed

    if (argc > 1)
    {
        SE_Init(argv[1], 0, 1, 0, 0);
    }
    else
    {
        printf("Usage: %s <path to xosc file>\n", argv[0]);
        return -1;
    }

    while (SE_GetQuitFlag() != 1)
    {
        speed += acc * SE_GetSimTimeStep();  // get latest time step from simulation

        // modulate speed by changing sign of acceleration now and then
        if (speed < 2.0f)
        {
            acc = 10;
        }
        else if (speed > 20.0f)
        {
            acc = -10;
        }

        // report updated speed
        SE_ReportObjectSpeed(0, speed);

        // step the simulation in natural speed, change to SE_Step(<time-step>) for fixed timestep
        SE_Step();
    }

    SE_Close();

    return 0;
}
