#include <stdio.h>
#include "esminiRMLib.hpp"

int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;

	RM_Init("../resources/xodr/straight_500m_signs.xodr");

	// Print some basic info
	printf("nrOfRoads: %d\n", RM_GetNumberOfRoads());
	for (int i = 0; i < RM_GetNumberOfRoads(); i++)
	{
		int rid = RM_GetIdOfRoadFromIndex(i);
		printf("Road[%d] ID: %d\n", i, rid);
		printf("Road[%d] nrOf(drivable)Lanes (at s=0): %d\n", i, RM_GetRoadNumberOfLanes(rid, 0));

		// Get some road sign info
		for (int j = 0; j < RM_GetNumberOfRoadSigns(rid); j++)
		{
			RM_RoadSign rs;
			RM_GetRoadSign(rid, j, &rs);
			printf("Road[%d] sign[%d] id %d name %s x %.2f y %.2f z %.2f heading %.2f orientation %d z_offset %.2f height: %.2f width %.2f\n",
				i, j, rs.id, rs.name, static_cast<double>(rs.x), static_cast<double>(rs.y), static_cast<double>(rs.z), static_cast<double>(rs.h), rs.orientation, static_cast<double>(rs.z_offset), static_cast<double>(rs.height), static_cast<double>(rs.width));

			int nValidityRecords = RM_GetNumberOfRoadSignValidityRecords(rid, j);
			for (int k = 0; k < nValidityRecords; k++)
			{
				RM_RoadObjValidity validityRecord;
				if (RM_GetRoadSignValidityRecord(rid, j, k, &validityRecord) == 0)
				{
					printf("   Validity rec[%d]: fromLane %d toLane %d\n", k, validityRecord.fromLane, validityRecord.toLane);
				}
			}
		}
	}

	// Create a position object
	int p0 = RM_CreatePosition();
	RM_SetWorldXYHPosition(p0, 9.0f, 60.0f, 1.57f);

	// Get some info
	RM_RoadLaneInfo laneInfo;
	RM_PositionData posData;
	RM_GetLaneInfo(p0, 0.0, &laneInfo, 2, false);  // LookAheadMode = 2 looks at current lane offset
	RM_GetPositionData(p0, &posData);

	printf("\nPosition manipulation:\n");
	printf("current pos: s %.2f laneId %d offset %.2f x %.2f y %.2f z %.2f \n",
		static_cast<double>(posData.s), laneInfo.laneId, static_cast<double>(laneInfo.laneOffset), static_cast<double>(posData.x), static_cast<double>(posData.y), static_cast<double>(posData.z));

	// Move the position object
	RM_SetLanePosition(p0, laneInfo.roadId, laneInfo.laneId + 1, 0.0, 200.0, true);
	RM_GetLaneInfo(p0, 0.0, &laneInfo, 2, false);  // LookAheadMode = 2 looks at current lane offset
	RM_GetPositionData(p0, &posData);
	printf("current pos: s %.2f laneId %d offset %.2f x %.2f y %.2f z %.2f \n",
		static_cast<double>(posData.s), laneInfo.laneId, static_cast<double>(laneInfo.laneOffset), static_cast<double>(posData.x), static_cast<double>(posData.y), static_cast<double>(posData.z));

	// Move forward in current lane
	RM_PositionMoveForward(p0, 950, -1); // Junction selector angle -1 = random choice
	RM_GetLaneInfo(p0, 0.0, &laneInfo, 2, false);  // LookAheadMode = 2 looks at current lane offset
	RM_GetPositionData(p0, &posData);
	printf("current pos: s %.2f laneId %d offset %.2f x %.2f y %.2f z %.2f \n",
		static_cast<double>(posData.s), laneInfo.laneId, static_cast<double>(laneInfo.laneOffset), static_cast<double>(posData.x), static_cast<double>(posData.y), static_cast<double>(posData.z));


	// Re-initialize road manager with another road to demonstrate junction ID
	RM_Init("../resources/xodr/fabriksgatan.xodr");
	p0 = RM_CreatePosition();

	// A position NOT in junction - junction expected to be -1
	RM_SetLanePosition(p0, 0, -1, 0.0f, 1.0f, false);
	RM_GetPositionData(p0, &posData);
	printf("Junction Id: %d\n", posData.junctionId);

	// A position in junction - junction expected to be 4
	RM_SetLanePosition(p0, 9, -1, 0.0f, 1.0f, false);
	RM_GetPositionData(p0, &posData);
	printf("Junction Id: %d\n", posData.junctionId);

	printf("Nr overlapping roads: %d\n", RM_GetNumberOfRoadsOverlapping(p0));
	for (size_t i = 0; static_cast<int>(i) < RM_GetNumberOfRoadsOverlapping(p0); i++)
	{
		printf("  road_id %d\n", RM_GetOverlappingRoadId(p0, static_cast<int>(i)));
	}

	RM_Close();

	return 0;
}
