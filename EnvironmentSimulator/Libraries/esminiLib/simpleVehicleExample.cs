using System;
using ESMini;

// demonstrates how to use the simple vehicle model in esminiLib
// add it together with ESMiniWrapper.cs to your project
// also add ESMiniLib (.dll, .so, .dylib) to the folder from where to execute
// and possibly update the path to the scenario in SE_Init

namespace esmini_csharp
{
    class Program
    {
        static void Main(string[] args)
        {
            if (ESMiniLib.SE_Init("../../../resources/xosc/cut-in_external.xosc", 0, 1, 0, 0) != 0)
            {
                Console.WriteLine("failed to load scenario");
                return;
            }

            IntPtr sv = ESMiniLib.SE_SimpleVehicleCreate(12.0, 40.0, 1.57, 5.0, 0.0);
            ESMiniLib.SE_SimpleVehicleSetMaxSpeed(sv, 90.0);
            SimpleVehicleState sv_state = new SimpleVehicleState();

            int state = 0;
            float dt = 0.05f;
            int throttle = 0;
            int steering = 0;
            while (ESMiniLib.SE_GetQuitFlag() != 1)
            {
                if(state == 0 && ESMiniLib.SE_GetSimulationTime() > 1.0f)
                {
                    throttle = 1;
                    steering = 0;
                    state = 1;
                }
                else if (state == 1 && ESMiniLib.SE_GetSimulationTime() > 5.0f)
                {
                    throttle = 0;
                    steering = 1;
                    state = 2;
                }
                else if (state == 2 && ESMiniLib.SE_GetSimulationTime() > 5.5)
                {
                    throttle = 0;
                    steering = -1;
                    state = 3;
                }
                else if (state == 3 && ESMiniLib.SE_GetSimulationTime() > 6.1)
                {
                    throttle = 1;
                    steering = 0;
                    state = 4;
                }
                else if (state == 4 && ESMiniLib.SE_GetSimulationTime() > 12.0)
                {
                    throttle = -1;
                    steering = 0;
                    state = 5;
                }

                ESMiniLib.SE_SimpleVehicleControlBinary(sv, dt, throttle, steering);
                ESMiniLib.SE_SimpleVehicleGetState(sv, ref sv_state);
                ESMiniLib.SE_ReportObjectPos(0, sv_state.x, sv_state.y, sv_state.z, sv_state.h, sv_state.p, 0.0);
                ESMiniLib.SE_ReportObjectSpeed(0, sv_state.speed);

                ESMiniLib.SE_StepDT(dt);
            }
        }
    }
}
