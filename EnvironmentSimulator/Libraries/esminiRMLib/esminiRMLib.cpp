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

#include <clocale>

#include "esminiRMLib.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"

using namespace roadmanager;

static roadmanager::OpenDrive *odrManager = 0;
static std::vector<Position> position;
static std::string returnString;  // use this for returning strings

static int GetProbeInfo(int index, float lookahead_distance, RM_RoadProbeInfo *r_data, int lookAheadMode, bool inRoadDrivingDirection)
{
	roadmanager::RoadProbeInfo s_data;

	if (index < 0 || odrManager == 0)
	{
		return -1;
	}

	if (index >= position.size())
	{
		LOG("Object %d not available, only %d registered", index, position.size());
		return -1;
	}

	double adjustedLookaheadDistance = lookahead_distance;

	if (!inRoadDrivingDirection)
	{
		// Find out what direction to look in
		if (fabs(position[index].GetHRelativeDrivingDirection()) > M_PI_2)
		{
			adjustedLookaheadDistance = -lookahead_distance;
		}
	}

	if (position[index].GetProbeInfo(adjustedLookaheadDistance, &s_data, (roadmanager::Position::LookAheadMode)lookAheadMode) != 0)
	{
		return -1;
	}
	else
	{
		// Copy data
		r_data->road_lane_info.pos[0] = (float)s_data.road_lane_info.pos[0];
		r_data->road_lane_info.pos[1] = (float)s_data.road_lane_info.pos[1];
		r_data->road_lane_info.pos[2] = (float)s_data.road_lane_info.pos[2];
		r_data->road_lane_info.curvature = (float)s_data.road_lane_info.curvature;
		r_data->road_lane_info.heading = (float)s_data.road_lane_info.heading;
		r_data->road_lane_info.pitch = (float)s_data.road_lane_info.pitch;
		r_data->road_lane_info.roll = (float)s_data.road_lane_info.roll;
		r_data->road_lane_info.speed_limit = (float)s_data.road_lane_info.speed_limit;
		r_data->relative_pos[0] = (float)s_data.relative_pos[0];
		r_data->relative_pos[1] = (float)s_data.relative_pos[1];
		r_data->relative_pos[2] = (float)s_data.relative_pos[2];
		r_data->relative_h = (float)s_data.relative_h;

		return 0;
	}
}

static int GetRoadLaneInfo(int index, float lookahead_distance, RM_RoadLaneInfo *r_data, int lookAheadMode, bool inRoadDrivingDirection)
{
	roadmanager::RoadLaneInfo s_data;

	if (index < 0 || odrManager == 0)
	{
		return -1;
	}

	if (index >= position.size())
	{
		LOG("Object %d not available, only %d registered", index, position.size());
		return -1;
	}

	double adjustedLookaheadDistance = lookahead_distance;

	if (!inRoadDrivingDirection)
	{
		// Find out what direction to look in
		if (fabs(position[index].GetHRelativeDrivingDirection()) > M_PI_2)
		{
			adjustedLookaheadDistance = -lookahead_distance;
		}
	}

	position[index].GetRoadLaneInfo(adjustedLookaheadDistance, &s_data, (roadmanager::Position::LookAheadMode)lookAheadMode);

	r_data->pos[0] = (float)s_data.pos[0];
	r_data->pos[1] = (float)s_data.pos[1];
	r_data->pos[2] = (float)s_data.pos[2];
	r_data->heading = (float)s_data.heading;
	r_data->pitch = (float)s_data.pitch;
	r_data->roll = (float)s_data.roll;
	r_data->width = (float)s_data.width;
	r_data->curvature = (float)s_data.curvature;
	r_data->speed_limit = (float)s_data.speed_limit;
	r_data->roadId = s_data.roadId;
	r_data->laneId = s_data.laneId;
	r_data->laneOffset = (float)s_data.laneOffset;
	r_data->t = (float)s_data.t;
	r_data->s = (float)s_data.s;

	return 0;
}

extern "C"
{
	RM_DLL_API int RM_Init(const char *odrFilename)
	{
		if (odrManager)
		{
			RM_Close();
		}

		Logger::Inst().OpenLogfile();
		Logger::Inst().LogVersion();

		// Harmonize parsing and printing of floating point numbers. I.e. 1.57e+4 == 15700.0 not 15,700.0 or 1 or 1.57
		std::setlocale(LC_ALL, "C.UTF-8");

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
	
	RM_DLL_API void RM_SetLogFilePath(const char* logFilePath)
	{
		SE_Env::Inst().SetLogFilePath(logFilePath);
	}

	RM_DLL_API int RM_CreatePosition()
	{
		roadmanager::Position newPosition;
		position.push_back(newPosition);
		return (int)(position.size() - 1);  // return index of newly created 
	}

	RM_DLL_API int RM_GetNrOfPositions()
	{
		return (int)position.size();
	}

	RM_DLL_API int RM_DeletePosition(int handle)
	{
		if (handle == -1)
		{
			// Delete all items
			position.clear();
		}
		else if (handle >= 0 && handle < position.size())
		{
			// Delete specific item
			position.erase(position.begin() + handle);
		}
		else
		{
			return -1;
		}

		return 0;
	}

	RM_DLL_API int RM_CopyPosition(int handle)
	{
		if (handle < 0 || handle >= position.size())
		{
			return -1;
		}

		roadmanager::Position newPosition(position[handle]);
		position.push_back(newPosition);
		return (int)(position.size() - 1);  // return index of newly created 
	}

	RM_DLL_API void RM_SetAlignMode(int handle, int mode)
	{
		if (handle < 0 || handle >= position.size())
		{
			return;
		}

		position[handle].SetAlignMode((roadmanager::Position::ALIGN_MODE)mode);
	}

	RM_DLL_API void RM_SetAlignModeH(int handle, int mode)
	{
		if (handle < 0 || handle >= position.size())
		{
			return;
		}

		position[handle].SetAlignModeH((roadmanager::Position::ALIGN_MODE)mode);
	}

	RM_DLL_API void RM_SetAlignModeP(int handle, int mode)
	{
		if (handle < 0 || handle >= position.size())
		{
			return;
		}

		position[handle].SetAlignModeP((roadmanager::Position::ALIGN_MODE)mode);
	}

	RM_DLL_API void RM_SetAlignModeR(int handle, int mode)
	{
		if (handle < 0 || handle >= position.size())
		{
			return;
		}

		position[handle].SetAlignModeR((roadmanager::Position::ALIGN_MODE)mode);
	}

	RM_DLL_API void RM_SetAlignModeZ(int handle, int mode)
	{
		if (handle < 0 || handle >= position.size())
		{
			return;
		}

		position[handle].SetAlignModeZ((roadmanager::Position::ALIGN_MODE)mode);
	}

	RM_DLL_API int RM_SetLockOnLane(int handle, bool mode)
	{
		if (handle < 0 || handle >= position.size())
		{
			return -1;
		}

		position[handle].SetLockOnLane(mode);

		return 0;
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
		
	RM_DLL_API int RM_SetLanePosition(int handle, int roadId, int laneId, float laneOffset, float s, bool align)
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

	RM_DLL_API int RM_SetWorldXYHPosition(int handle, float x, float y, float h)
	{
		if (odrManager == 0 || handle >= position.size())
		{
			return -1;
		}
		else
		{
			roadmanager::Position *pos = &position[handle];
			pos->XYZH2TrackPos(x, y, pos->GetZ(), h);
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
			roadmanager::Position* pos = &position[handle];
			pos->XYZH2TrackPos(x, y, z, h);
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

	RM_DLL_API int RM_PositionMoveForward(int handle, float dist, int strategy)
	{
		if (odrManager == 0 || handle >= position.size())
		{
			return -1;
		}
		else
		{
			roadmanager::Position *pos = &position[handle];
			
			return(pos->MoveAlongS(dist, 0.0, (Junction::JunctionStrategyType)strategy));
		}
	}

	RM_DLL_API int RM_GetPositionData(int handle, RM_PositionData *data)
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

	RM_DLL_API int RM_GetLaneInfo(int handle, float lookahead_distance, RM_RoadLaneInfo *data, int lookAheadMode, bool inRoadDrivingDirection)
	{
		if (odrManager == 0 || handle >= position.size())
		{
			return -1;
		}

		GetRoadLaneInfo(handle, lookahead_distance, data, lookAheadMode, inRoadDrivingDirection);

		return 0;
	}

	RM_DLL_API float RM_GetSpeedLimit(int handle)
	{
		if (odrManager == 0 || handle >= position.size())
		{
			return -1;
		}

		return (float)position[handle].GetSpeedLimit();
	}

	RM_DLL_API int RM_GetProbeInfo(int handle, float lookahead_distance, RM_RoadProbeInfo * data, int lookAheadMode, bool inRoadDrivingDirection)
	{
		if (odrManager == 0 || handle >= position.size())
		{
			return -1;
		}

		if (GetProbeInfo(handle, lookahead_distance, data, lookAheadMode, inRoadDrivingDirection) != 0)
		{
			return -1;
		}

		return 0;
	}

	RM_DLL_API bool RM_SubtractAFromB(int handleA, int handleB, RM_PositionDiff *pos_diff)
	{
		if (odrManager == 0 || handleA >= position.size() || handleB >= position.size())
		{
			return false;
		}

		PositionDiff diff;
		bool result = position[handleA].Delta(&position[handleB], diff);
		if (result == true)
		{
			pos_diff->ds = (float)diff.ds;
			pos_diff->dt = (float)diff.dt;
			pos_diff->dLaneId = diff.dLaneId;
		}

		return result;
	}

	RM_DLL_API int RM_GetNumberOfRoadSigns(int road_id)
	{
		if (odrManager == 0)
		{
			return false;
		}

		roadmanager::Road* road = odrManager->GetRoadById(road_id);
		
		if (road != NULL)
		{
			return road->GetNumberOfSignals();
		}
		
		return 0;
	}

	RM_DLL_API int RM_GetRoadSign(int road_id, int index, RM_RoadSign* road_sign)
	{
		if (odrManager == 0)
		{
			return false;
		}

		roadmanager::Road* road = odrManager->GetRoadById(road_id);

		if (road != NULL)
		{
			roadmanager::Signal* s = road->GetSignal(index);

			if (s)
			{
				// Resolve global cartesian position (x, y, z, h) from the road coordinate
				roadmanager::Position pos;
				pos.SetTrackPos(road_id, s->GetS(), s->GetT());

				road_sign->id = s->GetId();
				returnString = s->GetName();
				road_sign->name = returnString.c_str();
				road_sign->x = (float)pos.GetX();
				road_sign->y = (float)pos.GetY();
				road_sign->z = (float)pos.GetZ();
				road_sign->h = (float)pos.GetH();
				road_sign->s = (float)pos.GetS();
				road_sign->t = (float)pos.GetT();
				road_sign->orientation = s->GetOrientation() == roadmanager::Signal::Orientation::NEGATIVE ? -1 : 1;

				return 0;
			}
		}

		// Couldn't find the sign
		return -1;
	}

}
