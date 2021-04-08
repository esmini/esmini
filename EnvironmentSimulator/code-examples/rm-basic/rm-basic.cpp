#include <stdio.h>
#include "esminiRMLib.hpp"

int main(int argc, char* argv[])
{
	RM_Init("../resources/xodr/e6mini.xodr");

	// Print some basic info
	printf("nrOfRoads: %d\n", RM_GetNumberOfRoads());
	for (int i = 0; i < RM_GetNumberOfRoads(); i++)
	{
		int id = RM_GetIdOfRoadFromIndex(i);
		printf("Road[i] ID: %d\n", id);
		printf("Road[i] nrOf(drivable)Lanes (at s=0): %d\n", RM_GetRoadNumberOfLanes(id, 0));
	}

	// Create a position object
	int p0 = RM_CreatePosition();
	RM_SetWorldXYHPosition(p0, 9.0f, 60.0f, 1.57f);
	
	// Get some info
	RM_RoadLaneInfo laneInfo;
	RM_PositionData posData;
	RM_GetLaneInfo(p0, 0.0, &laneInfo, 2, false);  // LookAheadMode = 2 looks at current lane offset
	RM_GetPositionData(p0, &posData);
	
	printf("current pos: s %.2f laneId %d offset %.2f x %.2f y %.2f z %.2f \n",
		posData.s, laneInfo.laneId, laneInfo.laneOffset, posData.x, posData.y, posData.z);

	// Move the position object
	RM_SetLanePosition(p0, laneInfo.roadId, laneInfo.laneId + 1, 0.0, 200.0, true);
	RM_GetLaneInfo(p0, 0.0, &laneInfo, 2, false);  // LookAheadMode = 2 looks at current lane offset
	RM_GetPositionData(p0, &posData);
	printf("current pos: s %.2f laneId %d offset %.2f x %.2f y %.2f z %.2f \n", 
		posData.s, laneInfo.laneId, laneInfo.laneOffset, posData.x, posData.y, posData.z);

	// Move forward in current lane
	RM_PositionMoveForward(p0, 950, 0); // Junction strategy 0 = random choice 
	RM_GetLaneInfo(p0, 0.0, &laneInfo, 2, false);  // LookAheadMode = 2 looks at current lane offset
	RM_GetPositionData(p0, &posData);
	printf("current pos: s %.2f laneId %d offset %.2f x %.2f y %.2f z %.2f \n",
		posData.s, laneInfo.laneId, laneInfo.laneOffset, posData.x, posData.y, posData.z);

	return 0;
}
