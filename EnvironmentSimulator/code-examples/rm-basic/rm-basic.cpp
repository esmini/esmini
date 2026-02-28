#include <stdio.h>
#include "esminiRMLib.hpp"

void PrintInfo(int pos_id)
{
    RM_RoadLaneInfo laneInfo;
    RM_PositionData posData;

    // Get some info
    RM_GetLaneInfo(pos_id, 0.0, &laneInfo, 2, false);  // LookAheadMode = 2 looks at current lane offset
    RM_GetPositionData(pos_id, &posData);

    printf("current pos: s %.2f laneId %d offset %.2f x %.2f y %.2f z %.2f p %.2f r %.2f road_type %d lane_type %d\n",
           posData.s,
           laneInfo.laneId,
           laneInfo.laneOffset,
           posData.x,
           posData.y,
           posData.z,
           posData.p,
           posData.r,
           laneInfo.road_type,
           laneInfo.lane_type);
}

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    if (RM_Init("../resources/xodr/straight_500m.xodr") != 0)
    {
        printf("Failed init xodr/n");
        return -1;
    }

    // Create a position object
    int p0 = RM_CreatePosition();

    RM_SetLanePosition(p0, 1, -1, 0.0, 49.5, true);
    PrintInfo(p0);

    RM_PositionMoveForward(p0, 1.5, 0.0);
    PrintInfo(p0);

    RM_Close();

    return 0;
}
