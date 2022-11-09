#include "stdio.h"
#include <string>
#include "esminiLib.hpp"


int main(int argc, char* argv[])
{
    SE_AddPath("/eknabe1/GIT/esmini/resources/xosc/Catalogs/Vehicles");
    SE_AddPath("/eknabe1/GIT/esmini/resources/xosc/Catalogs/Controllers");

    if (SE_Init("/users/eknabe1/Downloads/issue/navi.xosc", 0, 1, 0, 0) != 0)
    {
        SE_LogMessage("Failed to initialize the scenario, quit\n");
        return -1;
    }

    SE_OSIFileOpen("kalle.osi");

    // show some road features, including road sensor
    SE_ViewerShowFeature(4 + 8, true);  // NODE_MASK_TRAIL_DOTS (1 << 2) & NODE_MASK_ODR_FEATURES (1 << 3),

    // Run for specified duration or until 'Esc' button is pressed
    while (SE_GetQuitFlag() != 1)
    {
        SE_ScenarioObjectState object_state;
        SE_GetObjectState(0, &object_state);

        SE_RoadInfo road_info;
        if (SE_GetRoadInfoAtDistance(0, 100, &road_info, 0, true) == -1)
        {
            printf("SE_GetRoadInfoAtDistance Failed\n");
        }
        else
        {
            printf("x %.2f y %.2f laneId %d\n", road_info.global_pos_x, road_info.global_pos_y, road_info.laneId);
        }

        SE_StepDT(0.05f);
    }

    SE_Close();

    return 0;
}