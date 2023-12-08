#include <stdio.h>
#include "esminiRMLib.hpp"

void PrintInfo(int pos_id)
{
    RM_RoadLaneInfo laneInfo;
    RM_PositionData posData;

    // Get some info
    RM_GetLaneInfo(pos_id, 0.0, &laneInfo, 2, false);  // LookAheadMode = 2 looks at current lane offset
    RM_GetPositionData(pos_id, &posData);

    printf("current pos: s %.2f laneId %d offset %.2f x %.2f y %.2f z %.2f p %.2f r %.2f \n",
           static_cast<double>(posData.s),
           laneInfo.laneId,
           static_cast<double>(laneInfo.laneOffset),
           static_cast<double>(posData.x),
           static_cast<double>(posData.y),
           static_cast<double>(posData.z),
           static_cast<double>(posData.p),
           static_cast<double>(posData.r));
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    if (RM_Init("C:/tmp/esmini_light/resources/tmp/knee.xodr") != 0)
    {
        printf("Failed init xodr/n");
        return -1;
    }

    // Create a position object
    int p0 = RM_CreatePosition();

    // Any driving lane (default)
    // see enum roadmanager::lane::LANE_TYPE_ANY_DRIVING
    RM_SetLanePosition(p0, 1, -1, 0.0f, 49.5f, true);
    PrintInfo(p0);

    RM_PositionMoveForward(p0, 1.5f, 0.0f);
    PrintInfo(p0);

    // any lanetype (all lanes)
    // see enum roadmanager::lane::LANE_TYPE_ANY
    RM_SetLanePosition(p0, 1, -1, 0.0f, 51.0f, true);
    PrintInfo(p0);

    RM_Close();

    return 0;
}
