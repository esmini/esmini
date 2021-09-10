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

static roadmanager::OpenDrive *odrManager = nullptr;
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

	if (position[index].GetProbeInfo(adjustedLookaheadDistance, &s_data, (roadmanager::Position::LookAheadMode)lookAheadMode) != roadmanager::Position::ErrorCode::ERROR_GENERIC)
	{
		// Copy data
		r_data->road_lane_info.pos[0] = (float)s_data.road_lane_info.pos[0];
		r_data->road_lane_info.pos[1] = (float)s_data.road_lane_info.pos[1];
		r_data->road_lane_info.pos[2] = (float)s_data.road_lane_info.pos[2];
		r_data->road_lane_info.heading = (float)s_data.road_lane_info.heading;
		r_data->road_lane_info.pitch = (float)s_data.road_lane_info.pitch;
		r_data->road_lane_info.roll = (float)s_data.road_lane_info.roll;
		r_data->road_lane_info.width = (float)s_data.road_lane_info.width;
		r_data->road_lane_info.curvature = (float)s_data.road_lane_info.curvature;
		r_data->road_lane_info.speed_limit = (float)s_data.road_lane_info.speed_limit;
		r_data->road_lane_info.roadId = (int)s_data.road_lane_info.roadId;
		r_data->road_lane_info.junctionId = (int)s_data.road_lane_info.junctionId;
		r_data->road_lane_info.laneId = (int)s_data.road_lane_info.laneId;
		r_data->road_lane_info.laneOffset = (float)s_data.road_lane_info.laneOffset;
		r_data->road_lane_info.s = (float)s_data.road_lane_info.s;
		r_data->road_lane_info.t = (float)s_data.road_lane_info.t;
		r_data->relative_pos[0] = (float)s_data.relative_pos[0];
		r_data->relative_pos[1] = (float)s_data.relative_pos[1];
		r_data->relative_pos[2] = (float)s_data.relative_pos[2];
		r_data->relative_h = (float)s_data.relative_h;

		if (position[index].GetStatusBitMask() & static_cast<int>(roadmanager::Position::PositionStatusMode::POS_STATUS_END_OF_ROAD))
		{
			return static_cast<int>(roadmanager::Position::ErrorCode::ERROR_END_OF_ROAD);
		}
		else if (position[index].GetStatusBitMask() & static_cast<int>(roadmanager::Position::PositionStatusMode::POS_STATUS_END_OF_ROUTE))
		{
			return static_cast<int>(roadmanager::Position::ErrorCode::ERROR_END_OF_ROUTE);
		}
		else
		{
			return 0;  // OK
		}
	}

	return -1;  // Error
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
	r_data->junctionId = s_data.junctionId;
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
		if (SE_Env::Inst().GetLogFilePath() == LOG_FILENAME)
		{
			// Set unique log filename to prevent two libraries writing to same file
			// - in case esminiRMLib is used in parallel with esminiLib
			RM_SetLogFilePath("esminiRM_log.txt");
		}

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
		odrManager = nullptr;
		position.clear();

		return 0;
	}

	RM_DLL_API void RM_SetLogFilePath(const char* logFilePath)
	{
		SE_Env::Inst().SetLogFilePath(logFilePath);
	}

	RM_DLL_API int RM_CreatePosition()
	{
		if (odrManager == nullptr)
		{
			return -1;
		}

		roadmanager::Position newPosition;
		position.push_back(newPosition);
		return (int)(position.size() - 1);  // return index of newly created
	}

	RM_DLL_API int RM_GetNrOfPositions()
	{
		if (odrManager == nullptr)
		{
			return -1;
		}

		return (int)position.size();
	}

	RM_DLL_API int RM_DeletePosition(int handle)
	{
		if (odrManager == nullptr)
		{
			return -1;
		}

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
		if (odrManager == nullptr || handle < 0 || handle >= position.size())
		{
			return -1;
		}

		roadmanager::Position newPosition(position[handle]);
		position.push_back(newPosition);
		return (int)(position.size() - 1);  // return index of newly created
	}

	RM_DLL_API void RM_SetAlignMode(int handle, int mode)
	{
		if (odrManager == nullptr || handle < 0 || handle >= position.size())
		{
			return;
		}

		position[handle].SetAlignMode((roadmanager::Position::ALIGN_MODE)mode);
	}

	RM_DLL_API void RM_SetAlignModeH(int handle, int mode)
	{
		if (odrManager == nullptr || handle < 0 || handle >= position.size())
		{
			return;
		}

		position[handle].SetAlignModeH((roadmanager::Position::ALIGN_MODE)mode);
	}

	RM_DLL_API void RM_SetAlignModeP(int handle, int mode)
	{
		if (odrManager == nullptr || handle < 0 || handle >= position.size())
		{
			return;
		}

		position[handle].SetAlignModeP((roadmanager::Position::ALIGN_MODE)mode);
	}

	RM_DLL_API void RM_SetAlignModeR(int handle, int mode)
	{
		if (odrManager == nullptr || handle < 0 || handle >= position.size())
		{
			return;
		}

		position[handle].SetAlignModeR((roadmanager::Position::ALIGN_MODE)mode);
	}

	RM_DLL_API void RM_SetAlignModeZ(int handle, int mode)
	{
		if (odrManager == nullptr || handle < 0 || handle >= position.size())
		{
			return;
		}

		position[handle].SetAlignModeZ((roadmanager::Position::ALIGN_MODE)mode);
	}

	RM_DLL_API int RM_SetLockOnLane(int handle, bool mode)
	{
		if (odrManager == nullptr || handle < 0 || handle >= position.size())
		{
			return -1;
		}

		position[handle].SetLockOnLane(mode);

		return 0;
	}

	RM_DLL_API int RM_GetNumberOfRoads()
	{
		if (odrManager != nullptr)
		{
			return odrManager->GetNumOfRoads();
		}
		else
		{
			return -1;
		}
	}

	RM_DLL_API int RM_GetIdOfRoadFromIndex(int index)
	{
		if (odrManager != nullptr)
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
		if (odrManager != nullptr)
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

		if (odrManager == nullptr)
		{
			return -1;
		}
		else
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

		if (odrManager == nullptr)
		{
			return -1;
		}
		else
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
		if (odrManager == nullptr || handle >= position.size())
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
		if (odrManager == nullptr || handle >= position.size())
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
		if (odrManager == nullptr || handle >= position.size())
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
		if (odrManager == nullptr || handle >= position.size())
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
		if (odrManager == nullptr || handle >= position.size())
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

	RM_DLL_API int RM_PositionMoveForward(int handle, float dist, float junctionSelectorAngle)
	{
		if (odrManager == nullptr || handle >= position.size())
		{
			return -1;
		}
		else
		{
			roadmanager::Position *pos = &position[handle];

			return(static_cast<int>(pos->MoveAlongS(dist, 0.0, junctionSelectorAngle)));
		}
	}

	RM_DLL_API int RM_GetPositionData(int handle, RM_PositionData *data)
	{
		if (odrManager == nullptr || handle >= position.size())
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
			data->junctionId = position[handle].GetJunctionId();
			data->laneId = position[handle].GetLaneId();
			data->laneOffset = (float)position[handle].GetOffset();
			data->s = (float)position[handle].GetS();
		}

		return 0;
	}

	RM_DLL_API int RM_GetLaneInfo(int handle, float lookahead_distance, RM_RoadLaneInfo *data, int lookAheadMode, bool inRoadDrivingDirection)
	{
		if (odrManager == nullptr || handle >= position.size())
		{
			return -1;
		}

		GetRoadLaneInfo(handle, lookahead_distance, data, lookAheadMode, inRoadDrivingDirection);

		return 0;
	}

	RM_DLL_API float RM_GetSpeedLimit(int handle)
	{
		if (odrManager == nullptr || handle >= position.size())
		{
			return -1;
		}

		return (float)position[handle].GetSpeedLimit();
	}

	RM_DLL_API int RM_GetProbeInfo(int handle, float lookahead_distance, RM_RoadProbeInfo * data, int lookAheadMode, bool inRoadDrivingDirection)
	{
		if (odrManager == nullptr || handle >= position.size())
		{
			return -1;
		}

		return GetProbeInfo(handle, lookahead_distance, data, lookAheadMode, inRoadDrivingDirection);
	}

	RM_DLL_API int RM_SubtractAFromB(int handleA, int handleB, RM_PositionDiff *pos_diff)
	{
		if (odrManager == nullptr || handleA >= position.size() || handleB >= position.size())
		{
			return -1;
		}

		PositionDiff diff;
		if (position[handleA].Delta(&position[handleB], diff) == true)
		{
			pos_diff->ds = (float)diff.ds;
			pos_diff->dt = (float)diff.dt;
			pos_diff->dLaneId = diff.dLaneId;

			return 0;
		}
		else
		{
			return -2;
		}
	}

	RM_DLL_API int RM_GetNumberOfRoadSigns(int road_id)
	{
		if (odrManager == nullptr)
		{
			return -1;
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
		if (odrManager == nullptr)
		{
			return -1;
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
				road_sign->z_offset = (float)s->GetZOffset();
				road_sign->length = (float)s->GetLength();
				road_sign->height = (float)s->GetHeight();
				road_sign->width = (float)s->GetWidth();

				return 0;
			}
		}

		// Couldn't find the sign
		return -1;
	}

	RM_DLL_API int RM_GetNumberOfRoadSignValidityRecords(int road_id, int index)
	{
		if (odrManager == nullptr)
		{
			return -1;
		}
		else
		{
			roadmanager::Road* road = odrManager->GetRoadById(road_id);
			if (road != NULL)
			{
				roadmanager::Signal* s = road->GetSignal(index);
				return (int)s->validity_.size();
			}
		}

		return 0;
	}

	RM_DLL_API int RM_GetRoadSignValidityRecord(int road_id, int signIndex, int validityIndex, RM_RoadObjValidity* validity)
	{
		if (odrManager != nullptr)
		{
			roadmanager::Road* road = odrManager->GetRoadById(road_id);
			if (road != NULL)
			{
				roadmanager::Signal* s = road->GetSignal(signIndex);
				if (validityIndex >= 0 && validityIndex < s->validity_.size())
				{
					validity->fromLane = s->validity_[validityIndex].fromLane_;
					validity->toLane = s->validity_[validityIndex].toLane_;
					return 0;
				}
			}
		}

		return -1;
	}
}
