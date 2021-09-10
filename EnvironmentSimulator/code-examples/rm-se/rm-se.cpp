// This is a nonsense dummy application for pure test purposes
// exercising usage of roadmanager library in parallel with scenario library

#include <stdio.h>
#include "esminiLib.hpp"
#include "esminiRMLib.hpp"

int main(int argc, char* argv[])
{
	for (int l = 0; l < 1; l++)
	{
		RM_Init("../resources/xodr/straight_500m_signs.xodr");
		SE_Init("../resources/xosc/slow-lead-vehicle.xosc", 0, 1, 0, 0);

		// Create a position object
		int p0 = RM_CreatePosition();
		RM_SetLanePosition(p0, 1, -1, 0.0, 200.0, true);

		// Get some info
		RM_RoadLaneInfo laneInfo;
		RM_PositionData posData;
		RM_GetLaneInfo(p0, 0.0, &laneInfo, 2, false);
		RM_GetPositionData(p0, &posData);

		printf("current pos1: s %.2f laneId %d offset %.2f x %.2f y %.2f z %.2f \n",
			posData.s, laneInfo.laneId, laneInfo.laneOffset, posData.x, posData.y, posData.z);

		// Move the position object
		RM_SetLanePosition(p0, laneInfo.roadId, laneInfo.laneId + 1, 0.0, 200.0, true);
		RM_GetLaneInfo(p0, 0.0, &laneInfo, 2, false);  // LookAheadMode = 2 looks at current lane offset
		RM_GetPositionData(p0, &posData);
		printf("current pos2: s %.2f laneId %d offset %.2f x %.2f y %.2f z %.2f \n",
			posData.s, laneInfo.laneId, laneInfo.laneOffset, posData.x, posData.y, posData.z);

		for (int i = 0; i < 10; i++)
		{
			SE_StepDT(0.1f);

			for (int j = 0; j < SE_GetNumberOfObjects(); j++)
			{
				SE_ScenarioObjectState state;

				SE_GetObjectState(j, &state);
				printf("time %.2f object[%d] type %d category %d pos (%.2f, %.2f) \n", state.timestamp, j, state.objectType, state.objectCategory, state.x, state.y);
			}
		}

		SE_Close();
		RM_Close();
	}

	return 0;
}
