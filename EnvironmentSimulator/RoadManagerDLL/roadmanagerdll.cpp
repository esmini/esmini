/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include "roadmanagerdll.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"

using namespace roadmanager;

static roadmanager::OpenDrive *odrManager = 0;
static std::vector<Position> position;

static int GetSteeringTarget(int index, float lookahead_distance, SteeringTargetData *data)
{
	double pos_globald[3];
	double pos_locald[3];
	double angled;
	double curvatured;

	if (odrManager == 0)
	{
		return -1;
	}

	if (index >= position.size())
	{
		LOG("Object %d not available, only %d registered", index, position.size());
		return -1;
	}

	if (position[index].GetSteeringTargetPos(lookahead_distance, pos_locald, pos_globald, &angled, &curvatured) != 0)
	{
		return -1;
	}
	else
	{
		// Copy data
		data->local_pos[0] = (float)pos_locald[0];
		data->local_pos[1] = (float)pos_locald[1];
		data->local_pos[2] = (float)pos_locald[2];
		data->global_pos[0] = (float)pos_globald[0];
		data->global_pos[1] = (float)pos_globald[1];
		data->global_pos[2] = (float)pos_globald[2];
		data->angle = (float)angled;
		data->curvature = (float)curvatured;

		return 0;
	}
}

extern "C"
{
	RM_DLL_API int RM_Init(const char *odrFilename)
	{
		if (!roadmanager::Position::LoadOpenDrive(odrFilename))
		{
			printf("Failed to load ODR %s\n", odrFilename);
			return -1;
		}
		odrManager = roadmanager::Position::GetOpenDrive();

		return 0;
	}

	RM_DLL_API int RM_Close()
	{
		position.clear();
		return 0;
	}
	
	RM_DLL_API int RM_CreatePosition()
	{
		roadmanager::Position newPosition;
		position.push_back(newPosition);
		return (int)(position.size() - 1);  // return index of newly created 
	}

	RM_DLL_API int RM_GetNumberOfRoads()
	{
		if (odrManager)
		{
			return odrManager->GetNumOfRoads();
		}
		else
		{
			return 0;
		}
	}

	RM_DLL_API int RM_GetIdOfRoadFromIndex(int index)
	{
		if (odrManager)
		{
			return odrManager->GetRoadByIdx(index)->GetId();
		}
		else
		{
			return -1;
		}
	}

	RM_DLL_API float RM_GetRoadLength(int id)
	{
		if (odrManager)
		{
			return (float)(odrManager->GetRoadById(id)->GetLength());
		}
		else
		{
			return 0.0f;
		}
	}

	RM_DLL_API int RM_GetRoadNumberOfLanes(int roadId, float s)
	{
		int numberOfDrivableLanes = 0;

		if (odrManager)
		{
			roadmanager::Road *road = odrManager->GetRoadById(roadId);

			// Consider only drivable lanes
			roadmanager::LaneSection *laneSection = road->GetLaneSectionByS(s);
			for (size_t i = 0; i < laneSection->GetNumberOfLanes(); i++)
			{
				if (laneSection->GetLaneByIdx((int)i)->IsDriving())
				{
					numberOfDrivableLanes++;
				}
			}
		}

		return numberOfDrivableLanes;
	}

	RM_DLL_API int RM_GetLaneIdByIndex(int roadId, int laneIndex, float s)
	{
		int numberOfDrivableLanes = 0;

		if (odrManager)
		{
			roadmanager::Road *road = odrManager->GetRoadById(roadId);

			// Consider only drivable lanes
			roadmanager::LaneSection *laneSection = road->GetLaneSectionByS(s);
			for (size_t i = 0; i < laneSection->GetNumberOfLanes(); i++)
			{
				if (laneSection->GetLaneByIdx((int)i)->IsDriving())
				{
					if (numberOfDrivableLanes == laneIndex)
					{
						return laneSection->GetLaneByIdx((int)i)->GetId();
					}
					else
					{
						numberOfDrivableLanes++;
					}
				}
			}
		}
		return 0;
	}
		
	RM_DLL_API int RM_SetLanePosition(int handle, int roadId, int laneId, int laneOffset, float s, bool align)
	{
		if (odrManager == 0 || handle >= position.size())
		{
			return -1;
		}
		else
		{
			roadmanager::Position *pos = &position[handle];
			pos->SetLanePos(roadId, laneId, s, laneOffset);

			if (align)
			{
				if (laneId < 0)
				{
					pos->SetHeadingRelative(0);
				}
				else
				{
					pos->SetHeadingRelative(M_PI);
				}
			}
		}

		return 0;
	}

	RM_DLL_API int RM_SetWorldPosition(int handle, float x, float y, float z, float h, float p, float r)
	{
		if (odrManager == 0 || handle >= position.size())
		{
			return -1;
		}
		else
		{
			roadmanager::Position *pos = &position[handle];
			pos->SetInertiaPos(x, y, z, h, p, r);
		}

		return 0;
	}

	RM_DLL_API int RM_SetWorldXYZHPosition(int handle, float x, float y, float z, float h)
	{
		if (odrManager == 0 || handle >= position.size())
		{
			return -1;
		}
		else
		{
			roadmanager::Position *pos = &position[handle];
			pos->XYZH2TrackPos(x, y, z, h, true);
			LOG("s %.2f roadId %d laneId %d", pos->GetS(), pos->GetTrackId(), pos->GetLaneId());
		}

		return 0;
	}

	RM_DLL_API int RM_SetS(int handle, float s)
	{
		if (odrManager == 0 || handle >= position.size())
		{
			return -1;
		}
		else
		{
			roadmanager::Position *pos = &position[handle];
			pos->SetLanePos(pos->GetTrackId(), pos->GetLaneId(), s, pos->GetOffset());
		}

		return 0;
	}

	RM_DLL_API int RM_PositionMoveForward(int handle, float dist)
	{
		if (odrManager == 0 || handle >= position.size())
		{
			return -1;
		}
		else
		{
			roadmanager::Position *pos = &position[handle];
			return(pos->MoveAlongS(dist));
		}
	}

	RM_DLL_API int RM_GetPositionData(int handle, PositionData *data)
	{
		if (odrManager == 0 || handle >= position.size())
		{
			return -1;
		}
		else
		{
			data->x = (float)position[handle].GetX();
			data->y = (float)position[handle].GetY();
			data->z = (float)position[handle].GetZ();
			data->h = (float)position[handle].GetH();
			data->p = (float)position[handle].GetP();
			data->r = (float)position[handle].GetR();
			data->hRelative = (float)position[handle].GetHRelative();
			data->roadId = position[handle].GetTrackId();
			data->laneId = position[handle].GetLaneId();
			data->laneOffset = (float)position[handle].GetOffset();
			data->s = (float)position[handle].GetS();
		}

		return 0;
	}

	RM_DLL_API int RM_GetSteeringTarget(int handle, float lookahead_distance, SteeringTargetData * data)
	{
		if (odrManager == 0 || handle >= position.size())
		{
			return -1;
		}

		if (GetSteeringTarget(handle, lookahead_distance, data) != 0)
		{
			return -1;
		}

		return 0;
	}

}
