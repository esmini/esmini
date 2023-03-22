using System;
using ESMini;

namespace esmini_csharp
{
    class Program
    {
        static void Main(string[] args)
        {
            if (ESMiniLib.SE_Init("../../../tmp/esmini/resources/xosc/cut-in.xosc", 0, 1, 0, 0) != 0)
            {
                Console.WriteLine("failed to load scenario");
                return;
            }

            ScenarioObjectState state = new ScenarioObjectState();

            while (ESMiniLib.SE_GetQuitFlag() != 1)
            {
                ESMiniLib.SE_GetObjectState(ESMiniLib.SE_GetId(1), ref state);  // Red overtaking car is index 1
                Console.WriteLine(state.wheel_angle);
                ESMiniLib.SE_Step();
            }
        }
    }
}
