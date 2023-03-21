using System;
using ESMini;

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

            IntPtr sv = ESMiniLib.SE_SimpleVehicleCreate(12.0f, 40.0f, 1.57f, 5.0f, 0.0f);
            ESMiniLib.SE_SimpleVehicleSetMaxSpeed(sv, 90.0f);
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
                else if (state == 2 && ESMiniLib.SE_GetSimulationTime() > 5.5f)
                {
                    throttle = 0;
                    steering = -1;
                    state = 3;
                }
                else if (state == 3 && ESMiniLib.SE_GetSimulationTime() > 6.1f)
                {
                    throttle = 1;
                    steering = 0;
                    state = 4;
                }

                ESMiniLib.SE_SimpleVehicleControlBinary(sv, dt, throttle, steering);
                ESMiniLib.SE_SimpleVehicleGetState(sv, ref sv_state);
                ESMiniLib.SE_ReportObjectPos(0, 0.0f, sv_state.x, sv_state.y, sv_state.h, sv_state.h, sv_state.p, 0.0f);
                ESMiniLib.SE_ReportObjectSpeed(0, sv_state.speed);

                ESMiniLib.SE_StepDT(dt);
            }
        }
    }
}
