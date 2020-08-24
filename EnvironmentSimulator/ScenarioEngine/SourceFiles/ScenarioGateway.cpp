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

#include "ScenarioGateway.hpp"
#include "CommonMini.hpp"
#include "Replay.hpp"
#include "osi_common.pb.h"
#include "osi_object.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_version.pb.h"
#include <cmath>

#ifdef _WIN32
#include <winsock2.h>
#include <Ws2tcpip.h>
#else
 /* Assume that any non-Windows platform uses POSIX-style sockets instead. */
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>  /* Needed for getaddrinfo() and freeaddrinfo() */
#include <unistd.h> /* Needed for close() */
#endif

#include <utils/geom/PositionVector.h>
#include <libsumo/Simulation.h>
#include <libsumo/Vehicle.h>
#include <libsumo/TraCIDefs.h>

using namespace scenarioengine;


#define OSI_OUT_PORT 48198

static struct {
	osi3::SensorView *sv;
	std::vector<osi3::MovingObject*> mobj;
	std::vector<osi3::Lane*> ln;
	std::vector<osi3::LaneBoundary*> lnb;
} mobj_osi_internal;

static OSISensorView osiSensorView;
static OSIRoadLane osiRoadLane;
static OSIRoadLaneBoundary osiRoadLaneBoundary;
std::ofstream osi_file;

static struct sockaddr_in recvAddr;

ObjectState::ObjectState()
{
	memset(&state_, 0, sizeof(ObjectState));
	state_.id = -1;
}


ObjectState::ObjectState(int id, std::string name, int model_id, int control, OSCBoundingBox boundingbox, double timestamp, double speed, double wheel_angle, double wheel_rot, roadmanager::Position* pos)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.model_id = model_id;
	state_.control = control;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos = *pos;
	state_.speed = (float)speed;
	state_.wheel_angle = (float)wheel_angle;
	state_.wheel_rot = (float)wheel_rot;
	state_.boundingbox = boundingbox;
}

ObjectState::ObjectState(int id, std::string name, int model_id, int control, OSCBoundingBox boundingbox, double timestamp, double speed, double wheel_angle, double wheel_rot, double x, double y, double z, double h, double p, double r)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.model_id = model_id;
	state_.control = control;
	state_.name[0] = 0;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos.Init();
	state_.pos.SetInertiaPos(x, y, z, h, p, r);
	state_.speed = (float)speed;
	state_.wheel_angle = (float)wheel_angle;
	state_.wheel_rot = (float)wheel_rot;
	state_.boundingbox = boundingbox;
}

ObjectState::ObjectState(int id, std::string name, int model_id, int control, OSCBoundingBox boundingbox, double timestamp, double speed, double wheel_angle, double wheel_rot, int roadId, int laneId, double laneOffset, double s)
{
	memset(&state_, 0, sizeof(ObjectStateStruct));

	state_.id = id;
	state_.model_id = model_id;
	state_.control = control;
	state_.timeStamp = (float)timestamp;
	strncpy(state_.name, name.c_str(), NAME_LEN);
	state_.pos.SetLanePos(roadId, laneId, s, laneOffset);
	state_.speed = (float)speed;
	state_.wheel_angle = (float)wheel_angle;
	state_.wheel_rot = (float)wheel_rot;
	state_.boundingbox = boundingbox;
}

void ObjectState::Print()
{
	LOG("state: \n\tid %d\n\tname %s\n\tmodel_id: %d\n\tcontrol: %d\n\ttime %.2f\n\tx %.2f\n\ty %.2f\n\th %.2f\n\tspeed %.2f\twheel_angle %.2f",
		state_.id,
		state_.name,
		state_.model_id,
		state_.control,
		state_.timeStamp,
		state_.pos.GetX(),
		state_.pos.GetY(),
		state_.pos.GetZ(),
		state_.speed,
		state_.wheel_angle
	);
	LOG("state: \n\tbounding box: \ncenter: x: %.2f, y: %.2f, z: %.2f\n\tdimensions: width: %.2f, length: %.2f, height: %.2f",
    state_.boundingbox.center_.x_,
		state_.boundingbox.center_.y_,
		state_.boundingbox.center_.z_,
		state_.boundingbox.dimensions_.width_,
		state_.boundingbox.dimensions_.length_,
		state_.boundingbox.dimensions_.height_
	);
}

// ScenarioGateway

ScenarioGateway::ScenarioGateway()
{
	sendSocket = 0;

	objectState_.clear();

	mobj_osi_internal.sv = new osi3::SensorView();

	mobj_osi_internal.sv->mutable_version()->set_version_major(3);
	mobj_osi_internal.sv->mutable_version()->set_version_minor(0);
	mobj_osi_internal.sv->mutable_version()->set_version_patch(0);

	mobj_osi_internal.sv->mutable_timestamp()->set_seconds(0);
	mobj_osi_internal.sv->mutable_timestamp()->set_seconds(0);
}


ScenarioGateway::~ScenarioGateway()
{
	for (size_t i = 0; i < objectState_.size(); i++)
	{
		delete objectState_[i];
	}
	objectState_.clear();

	osiSensorView.size = 0;
	osiRoadLane.size=0;

	data_file_.flush();
	data_file_.close();

	CloseSocket();
	if (osi_file.is_open())
	{
		osi_file.close();
	}

	// Assume osi cleans up allocated data
}

int ScenarioGateway::OpenSocket(std::string ipaddr)
{
#ifdef _WIN32
	WSADATA wsa_data;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsa_data);
	if (iResult != NO_ERROR)
	{
		wprintf(L"WSAStartup failed with error %d\n", iResult);
		return -1;
	}
#endif

	// create socket for outgoing UDP packages
	sendSocket = (int)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sendSocket < 0) {
		LOG("socket failed");
		return -1;
	}

	// Setup receiver IP address
	recvAddr.sin_family = AF_INET;
	recvAddr.sin_port = htons(OSI_OUT_PORT);
	inet_pton(AF_INET, ipaddr.c_str(), &recvAddr.sin_addr);

	return 0;
}

int ScenarioGateway::CloseSocket()
{
	if (!sendSocket)
	{
		return 0;
	}

#ifdef _WIN32
	if (closesocket(sendSocket) == SOCKET_ERROR)
#else
	if (close(sendSocket) < 0)
#endif
	{
		printf("Failed closing socket");
		return -1;
	}

#ifdef _WIN32
	WSACleanup();
#endif

	return 0;
}

ObjectState* ScenarioGateway::getObjectStatePtrById(int id)
{
	for (size_t i = 0; i < objectState_.size(); i++)
	{
		if (objectState_[i]->state_.id == id)
		{
			return objectState_[i];
		}
	}

	return 0;
}

int ScenarioGateway::getObjectStateById(int id, ObjectState& objectState)
{
	for (size_t i = 0; i < objectState_.size(); i++)
	{
		if (objectState_[i]->state_.id == id)
		{
			objectState = *objectState_[i];
			return 0;
		}
	}

	// Indicate not found by returning non zero
	return -1;
}

bool ScenarioGateway::OpenOSIFile()
{
	osi_file = std::ofstream("move_obj.osi", std::ios_base::binary);
	if (!osi_file.good())
	{
		LOG("Failed open osi file");
		return false; 
	}
	return true; 
}

bool ScenarioGateway::WriteOSIFile()
{
	// write to file, first size of message
	osi_file.write((char*)&osiSensorView.size, sizeof(osiSensorView.size));

	// write to file, actual message - the sensorview object including timestamp and moving objects
	osi_file.write(osiSensorView.sensor_view.c_str(), osiSensorView.size);

	if (!osi_file.good())
	{
		LOG("Failed write osi file");
		return false; 
	}
	return true; 
}

int ScenarioGateway::UpdateOSISensorView()
{
	double time_stamp = objectState_[0]->state_.timeStamp;

	mobj_osi_internal.sv->mutable_global_ground_truth()->mutable_timestamp()->set_seconds((int64_t)objectState_[0]->state_.timeStamp);
	mobj_osi_internal.sv->mutable_global_ground_truth()->mutable_timestamp()->set_nanos((uint32_t)(
		(objectState_[0]->state_.timeStamp - (int64_t)objectState_[0]->state_.timeStamp) * 1e9)
	);

	UpdateOSIMovingObject();
	//collect all information of lanes in the lane section where obj=0 is
	UpdateOSIRoadLane();

	UpdateOSILaneBoundary();

	mobj_osi_internal.sv->SerializeToString(&osiSensorView.sensor_view);
	osiSensorView.size = (unsigned int)mobj_osi_internal.sv->ByteSizeLong();

	if (sendSocket)
	{
		// send over udp - skip size (package size == message size)
		int sendResult = sendto(sendSocket, (char*)osiSensorView.sensor_view.c_str(), osiSensorView.size, 0, (struct sockaddr*)&recvAddr, sizeof(recvAddr));

		if (sendResult != osiSensorView.size)
		{
			LOG("Failed send osi package over UDP");
#ifdef _WIN32
			wprintf(L"send failed with error: %d\n", WSAGetLastError());
#endif
		}
	}

	return 0;
}

int ScenarioGateway::UpdateOSIMovingObject()
{
	// Add instances of moving object according to number of objects in the scenario
	while (mobj_osi_internal.mobj.size() < objectState_.size())
	{
		osi3::MovingObject* mobj = mobj_osi_internal.sv->mutable_global_ground_truth()->add_moving_object();

		mobj->mutable_id()->set_value(mobj_osi_internal.mobj.size());
		mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_MEDIUM_CAR);
		mobj->mutable_base()->mutable_dimension()->set_height(1.5);
		mobj->mutable_base()->mutable_dimension()->set_width(1.75);
		mobj->mutable_base()->mutable_dimension()->set_length(4.0);

		mobj_osi_internal.mobj.push_back(mobj);
	}

	for (size_t i = 0; i < objectState_.size(); i++)
	{
		mobj_osi_internal.mobj[i]->mutable_vehicle_attributes()->mutable_driver_id()->set_value((uint64_t)objectState_[i]->state_.control);  // a placeholder for control mode
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_position()->set_x(objectState_[i]->state_.pos.GetX());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_position()->set_y(objectState_[i]->state_.pos.GetY());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_position()->set_z(objectState_[i]->state_.pos.GetZ());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_orientation()->set_yaw(objectState_[i]->state_.pos.GetH());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_orientation_rate()->set_yaw(objectState_[i]->state_.pos.GetHRate());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_orientation_acceleration()->set_yaw(objectState_[i]->state_.pos.GetHAcc());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_orientation()->set_pitch(objectState_[i]->state_.pos.GetP());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_orientation()->set_roll(objectState_[i]->state_.pos.GetR());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_velocity()->set_x(objectState_[i]->state_.pos.GetVelX());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_velocity()->set_y(objectState_[i]->state_.pos.GetVelY());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_velocity()->set_z(0);  // assume neglectable speed in z dimension
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_acceleration()->set_x(objectState_[i]->state_.pos.GetAccX());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_acceleration()->set_y(objectState_[i]->state_.pos.GetAccY());
		mobj_osi_internal.mobj[i]->mutable_base()->mutable_acceleration()->set_z(0);  // assume neglectable speed in z dimension
	}

	return 0;
}

int ScenarioGateway::UpdateOSILaneBoundary()
{
	//Retrieve opendrive class from RoadManager
	static roadmanager::OpenDrive* opendrive = roadmanager::Position::GetOpenDrive();

	//Loop over all roads
	for (int i = 0; i<opendrive->GetNumOfRoads(); i++)
	{

		roadmanager::Road* road = opendrive->GetRoadByIdx(i);

		// loop over all lane sections
		for (int j= 0; j<road->GetNumberOfLaneSections(); j++)
		{
			roadmanager::LaneSection* lane_section = road->GetLaneSectionByIdx(j);

			// loop over all lanes
			for (int k=0; k<lane_section->GetNumberOfLanes(); k++)
			{
				roadmanager::Lane* lane = lane_section->GetLaneByIdx(k);

				int n_roadmarks = lane->GetNumberOfRoadMarks();
				if (n_roadmarks != 0) // if there are road marks 
				{
					// loop over RoadMarks
					for (int ii = 0; ii < lane->GetNumberOfRoadMarks(); ii++)
					{
						roadmanager::LaneRoadMark* laneroadmark = lane->GetLaneRoadMarkByIdx(ii);

						//loop over road mark types
						for (int jj = 0; jj < laneroadmark->GetNumberOfRoadMarkTypes(); jj++)
						{
							roadmanager::LaneRoadMarkType* laneroadmarktype = laneroadmark->GetLaneRoadMarkTypeByIdx(jj);

							// loop over LaneRoadMarkTypeLine
							for (int kk = 0; kk< laneroadmarktype->GetNumberOfRoadMarkTypeLines(); kk++)
							{
								int num_lines = laneroadmarktype->GetNumberOfRoadMarkTypeLines();
								roadmanager::LaneRoadMarkTypeLine* laneroadmarktypeline = laneroadmarktype->GetLaneRoadMarkTypeLineByIdx(kk);

								osi3::LaneBoundary* osi_laneboundary = 0;
								int line_id = laneroadmarktypeline->GetGlobalId();

								// Check if this line is already pushed to OSI
								for (int h=0; h < mobj_osi_internal.lnb.size(); h++)
								{
									if (mobj_osi_internal.lnb[h]->mutable_id()->value() == line_id)
									{
										osi_laneboundary = mobj_osi_internal.lnb[h];
									}
								}
								if (!osi_laneboundary)
								{
									osi_laneboundary = mobj_osi_internal.sv->mutable_global_ground_truth()->add_lane_boundary();

									// update id
									osi_laneboundary->mutable_id()->set_value(line_id);

									int n_osi_points = laneroadmarktypeline->GetOSIPoints().GetNumOfOSIPoints();
									for (int h = 0; h < n_osi_points; h++)
									{
										osi3::LaneBoundary_BoundaryPoint* boundary_point = osi_laneboundary->add_boundary_line();
										boundary_point->mutable_position()->set_x(laneroadmarktypeline->GetOSIPoints().GetXfromIdx(h));
										boundary_point->mutable_position()->set_y(laneroadmarktypeline->GetOSIPoints().GetYfromIdx(h));
										boundary_point->mutable_position()->set_z(laneroadmarktypeline->GetOSIPoints().GetZfromIdx(h));
										boundary_point->set_width(laneroadmarktypeline->GetWidth());
										boundary_point->set_height(laneroadmark->GetHeight());
									}

									// update classification type
									osi3::LaneBoundary_Classification_Type classific_type;
									roadmanager::LaneRoadMark::RoadMarkType tyype = laneroadmark->GetType();
									osi3::LaneBoundary_Classification_Type osi_type = osi_laneboundary->mutable_classification()->type();
									switch(laneroadmark->GetType())
									{
										case roadmanager::LaneRoadMark::RoadMarkType::NONE_TYPE:
										classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_UNKNOWN;
										break;
										case roadmanager::LaneRoadMark::RoadMarkType::SOLID:
										classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE;
										break;
										case roadmanager::LaneRoadMark::RoadMarkType::BROKEN:
										classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_DASHED_LINE;
										break;
										default: classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_SOLID_LINE;
									}
									osi_laneboundary->mutable_classification()->set_type(classific_type);
									osi_type = osi_laneboundary->mutable_classification()->type();

									// update classification color
									osi3::LaneBoundary_Classification_Color classific_col;
									switch(laneroadmark->GetColor())
									{
										case roadmanager::LaneRoadMark::RoadMarkColor::STANDARD_COLOR:
										classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_WHITE;
										break;
										case roadmanager::LaneRoadMark::RoadMarkColor::BLUE:
										classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_BLUE;
										break;
										case roadmanager::LaneRoadMark::RoadMarkColor::GREEN:
										classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_GREEN;
										break;
										case roadmanager::LaneRoadMark::RoadMarkColor::RED:
										classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_RED;
										break;
										case roadmanager::LaneRoadMark::RoadMarkColor::WHITE:
										classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_WHITE;
										break;
										case roadmanager::LaneRoadMark::RoadMarkColor::YELLOW:
										classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_YELLOW;
										break;
										default: classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_WHITE;
									}
									osi_laneboundary->mutable_classification()->set_color(classific_col);

									// update limiting structure id only if the type of lane boundary is set to TYPE_STRUCTURE - for now it is not implemented
									//osi_laneboundary->mutable_classification()->mutable_limiting_structure_id(0)->set_value(0);

									mobj_osi_internal.lnb.push_back(osi_laneboundary);

								}
							}
						}
					}
				}
				else // if there are no road marks I take the lane boundary 
				{
					roadmanager::LaneBoundaryOSI* laneboundary = lane->GetLaneBoundary();
					// Check if this line is already pushed to OSI
					int boundary_id = laneboundary->GetGlobalId();
					osi3::LaneBoundary* osi_laneboundary = 0;
					for (int h=0; h < mobj_osi_internal.lnb.size(); h++)
					{
						if (mobj_osi_internal.lnb[h]->mutable_id()->value() == boundary_id)
						{
							osi_laneboundary = mobj_osi_internal.lnb[h];
						}
					}
					if (!osi_laneboundary)
					{
						osi_laneboundary = mobj_osi_internal.sv->mutable_global_ground_truth()->add_lane_boundary();

						// update id
						osi_laneboundary->mutable_id()->set_value(boundary_id);

						int n_osi_points = laneboundary->GetOSIPoints().GetNumOfOSIPoints();
						for (int h = 0; h < n_osi_points; h++)
						{
							osi3::LaneBoundary_BoundaryPoint* boundary_point = osi_laneboundary->add_boundary_line();
							boundary_point->mutable_position()->set_x(laneboundary->GetOSIPoints().GetXfromIdx(h));
							boundary_point->mutable_position()->set_y(laneboundary->GetOSIPoints().GetYfromIdx(h));
							boundary_point->mutable_position()->set_z(laneboundary->GetOSIPoints().GetZfromIdx(h));
							//boundary_point->set_width(laneboundary->GetWidth());
							//boundary_point->set_height(laneroadmark->GetHeight());
						}

						osi3::LaneBoundary_Classification_Type classific_type = osi3::LaneBoundary_Classification_Type::LaneBoundary_Classification_Type_TYPE_NO_LINE;
						osi_laneboundary->mutable_classification()->set_type(classific_type);

						osi3::LaneBoundary_Classification_Color classific_col = osi3::LaneBoundary_Classification_Color::LaneBoundary_Classification_Color_COLOR_UNKNOWN;
						osi_laneboundary->mutable_classification()->set_color(classific_col);

						mobj_osi_internal.lnb.push_back(osi_laneboundary);
					}
				}
			}
		}
	}

	return 0;
}

bool ScenarioGateway::IsAngleStraight(double teta)
{
	teta = fmod(teta + M_PI,2*M_PI);

    if (teta < 0)
        teta += 2*M_PI;

    teta = teta - M_PI;

	if (teta >= -(M_PI/2) && teta<= (M_PI/2))
	{
		return true; 
	}
	else
	{
		return false; 
	}
}

int ScenarioGateway::UpdateOSIRoadLane()
{
	// Find ego vehicle 
	roadmanager::Position pos;
	for (size_t i = 0; i < getNumberOfObjects() ; i++)
	{
		if (objectState_[i]->state_.control == 3) // external hybrid is host 
		{
			pos = objectState_[i]->state_.pos;
		}
	}

	//Retrieve opendrive class from RoadManager
	static roadmanager::OpenDrive* opendrive = roadmanager::Position::GetOpenDrive();

	// Loop over all roads
	for (int i = 0; i<opendrive->GetNumOfRoads(); i++)
	{

		roadmanager::Road* road = opendrive->GetRoadByIdx(i);

		// loop over all lane sections
		for (int j= 0; j<road->GetNumberOfLaneSections(); j++)
		{
			roadmanager::LaneSection* lane_section = road->GetLaneSectionByIdx(j);

			// loop over all lanes
			for (int k=0; k<lane_section->GetNumberOfLanes(); k++)
			{
				roadmanager::Lane* lane = lane_section->GetLaneByIdx(k);
				osi3::Lane* osi_lane = 0;
				int lane_global_id = lane->GetGlobalId();
				int lane_id = lane->GetId();


				// Check if this lane is already pushed to OSI - if yes just update
				for (int jj=0; jj < mobj_osi_internal.ln.size(); jj++)
				{
					if (mobj_osi_internal.ln[jj]->mutable_id()->value() == lane_global_id)
					{
						osi_lane = mobj_osi_internal.ln[jj];

						// update classification is_host_vehicle_in_lane
						bool is_ego_on_lane = false;
						if (lane_id == pos.GetLaneId())
						{
							is_ego_on_lane = true;
						}
						osi_lane->mutable_classification()->set_is_host_vehicle_lane(is_ego_on_lane);
						break;
					}
				}
				// if the lane is not already in the osi message we add it all 
				if (!osi_lane) 
				{
					// LANE ID 
					osi_lane = mobj_osi_internal.sv->mutable_global_ground_truth()->add_lane();
					osi_lane->mutable_id()->set_value(lane_global_id);

					// CLASSIFICATION TYPE 
					roadmanager::Lane::LaneType lanetype = lane->GetLaneType();
					osi3::Lane_Classification_Type class_type;

					if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_DRIVING 		|| 
						lanetype == roadmanager::Lane::LaneType::LANE_TYPE_PARKING		|| 	
						lanetype == roadmanager::Lane::LaneType::LANE_TYPE_BIDIRECTIONAL 	 )		
					{
						class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING;
					}
					else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_STOP 		|| 						
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_BIKING 		|| 
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SIDEWALK 	|| 
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_BORDER   	|| 
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_RESTRICTED   || 
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_ROADMARKS    || 
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_TRAM   		|| 
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_RAIL   			 )
					{
						class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING;
					}
					else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_ENTRY 		|| 
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_EXIT 		|| 
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_OFF_RAMP 	|| 
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_ON_RAMP 		|| 
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_MEDIAN 		|| 
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SHOULDER 		 )
					{
						class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION;					
					}
					else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SPECIAL1 	|| 
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SPECIAL2 	|| 
							lanetype == roadmanager::Lane::LaneType::LANE_TYPE_SPECIAL3 		 ) 
					{
						class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_OTHER;					
					}
					else if (lanetype == roadmanager::Lane::LaneType::LANE_TYPE_NONE )
					{
						class_type = osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_UNKNOWN;
					}
					osi_lane->mutable_classification()->set_type(class_type);

					// CENTERLINE POINTS 
					int n_osi_points = lane->GetOSIPoints().GetNumOfOSIPoints();
					for (int jj = 0; jj < n_osi_points; jj++)
					{
						osi3::Vector3d* centerLine = osi_lane->mutable_classification()->add_centerline();
						centerLine->set_x(lane->GetOSIPoints().GetXfromIdx(jj));
						centerLine->set_y(lane->GetOSIPoints().GetYfromIdx(jj));
						centerLine->set_z(lane->GetOSIPoints().GetZfromIdx(jj));
					}

					// DRIVING DIRECTION 
					bool driving_direction = true; 
					if (lane_id >= 0)
					{
						driving_direction = false; 
					} 						 
					osi_lane->mutable_classification()->set_centerline_is_driving_direction(driving_direction);

					// LEFT AND RIGHT LANE IDS 
					int n_lanes_in_section = lane_section->GetNumberOfLanes();
					std::vector< std::pair <int,int> > globalid_ids_left;
					std::vector< std::pair <int,int> > globalid_ids_right;
					for (int jj = 0; jj < n_lanes_in_section; jj++)
					{
						if (lane_section->GetLaneIdByIdx(jj) > lane_id)
						{
							globalid_ids_left.push_back( std::make_pair(lane_section->GetLaneIdByIdx(jj) , lane_section->GetLaneGlobalIdByIdx(jj) ) );
						}
						else if (lane_section->GetLaneIdByIdx(jj) < lane_id)
						{

							globalid_ids_right.push_back( std::make_pair(lane_section->GetLaneIdByIdx(jj) , lane_section->GetLaneGlobalIdByIdx(jj) ) );
						}
					}
						// order global id with local id to maintain geographical order
					std::sort(globalid_ids_left.begin(), globalid_ids_left.end());
					std::sort(globalid_ids_right.begin(), globalid_ids_right.end());
					std::reverse(globalid_ids_right.begin(), globalid_ids_right.end());

					for (int jj = 0; jj < globalid_ids_left.size(); jj++)
					{
						osi3::Identifier* left_id = osi_lane->mutable_classification()->add_left_adjacent_lane_id();
						left_id->set_value((uint64_t)globalid_ids_left[jj].second);
					}
					for (int jj = 0; jj < globalid_ids_right.size(); jj++)
					{
						osi3::Identifier* right_id = osi_lane->mutable_classification()->add_right_adjacent_lane_id();
						right_id->set_value((uint64_t)globalid_ids_right[jj].second);
					}

					// LANE PAIRING 
					// STILL TO DO: when I get a vector of predecessors and successors I need to create all possible combinations
					roadmanager::LaneLink* lane_pre = lane->GetLink(roadmanager::LinkType::PREDECESSOR);
					roadmanager::LaneLink* lane_succ = lane->GetLink(roadmanager::LinkType::SUCCESSOR);
					bool exist_link = false;
					if (lane_pre != 0)
					{
						osi3::Lane_Classification_LanePairing* lane_pair = osi_lane->mutable_classification()->add_lane_pairing();
						lane_pair->mutable_antecessor_lane_id()->set_value(lane_pre->GetId());
						if (lane_succ != 0)
						{
							lane_pair->mutable_successor_lane_id()->set_value(lane_succ->GetId());
						}
					}
					else if (lane_succ != 0)
					{
						osi3::Lane_Classification_LanePairing* lane_pair = osi_lane->mutable_classification()->add_lane_pairing();
						lane_pair->mutable_successor_lane_id()->set_value(lane_succ->GetId());
						if (lane_pre != 0)
						{
							lane_pair->mutable_antecessor_lane_id()->set_value(lane_pre->GetId());
						}
					}

					// LANE BOUNDARY IDS 
					if (lane_id == 0) // for central lane I use the laneboundary osi points as right and left boundary so that it can be used from both sides
					{
						// check if lane has road mark 
						std::vector<int> line_ids = lane->GetLineGlobalIds();
						if (!line_ids.empty()) // lane has RoadMarks 
						{
							for (int jj = 0; jj < line_ids.size(); jj++ )
							{
								osi3::Identifier* left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
								left_lane_bound_id->set_value(line_ids[jj]);
								osi3::Identifier* right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
								right_lane_bound_id->set_value(line_ids[jj]);
							}
						}
						else // no road marks -> we take lane boundary 
						{
							int laneboundary_global_id = lane->GetLaneBoundaryGlobalId(); 
							osi3::Identifier* left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
							left_lane_bound_id->set_value(laneboundary_global_id);
							osi3::Identifier* right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
							right_lane_bound_id->set_value(laneboundary_global_id);
						}
					}
					else
					{
						// Set left/right laneboundary ID for left/right lanes- we use LaneMarks is they exist, if not we take laneboundary
						std::vector<int> line_ids = lane->GetLineGlobalIds();
						if (!line_ids.empty()) // lane has RoadMarks 
						{
							for (int jj = 0; jj < line_ids.size(); jj++ )
							{
								if (lane_id > 0 )
								{
									osi3::Identifier* left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
									left_lane_bound_id->set_value(line_ids[jj]);
								}
								if (lane_id < 0 )
								{
									osi3::Identifier* right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
									right_lane_bound_id->set_value(line_ids[jj]);
								}
							}
						}
						else
						{
							int laneboundary_global_id = lane->GetLaneBoundaryGlobalId(); 
							if (lane_id > 0 )
							{
								osi3::Identifier* left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
								left_lane_bound_id->set_value(laneboundary_global_id);
							}
							if (lane_id < 0 )
							{
								osi3::Identifier* right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
								right_lane_bound_id->set_value(laneboundary_global_id);
							}
						}

						// Set right/left laneboundary ID for left/right lanes - we look at neightbour lanes
						int next_lane_id = 0;
						if (lane_id < 0) // if lane is on the right, then it contains its right boundary. So I need to look into its left lane for the left boundary 
						{
							next_lane_id = lane_id+1;
						}
						else if (lane_id > 0) // if lane is on the left, then it contains its left boundary. So I need to look into its right lane for the right boundary
						{
							next_lane_id = lane_id-1;
						}
						// look at right lane and check if it has Lines for RoadMarks
						roadmanager::Lane* next_lane = lane_section->GetLaneById(next_lane_id);
						std::vector<int> nextlane_line_ids = next_lane->GetLineGlobalIds();
						if (!nextlane_line_ids.empty())
						{
							for (int jj = 0; jj < nextlane_line_ids.size(); jj++ )
							{
								if (lane_id<0)
								{
									osi3::Identifier* left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();
									left_lane_bound_id->set_value(nextlane_line_ids[jj]);								
								}
								else if (lane_id>0)
								{
									osi3::Identifier* right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
									right_lane_bound_id->set_value(nextlane_line_ids[jj]);
								}
							}
						}
						else // if the neightbour lane does not have Lines for RoadMakrs we take the LaneBoundary
						{
							int next_laneboundary_global_id = next_lane->GetLaneBoundaryGlobalId(); 
							if (lane_id < 0 )
							{
								osi3::Identifier* left_lane_bound_id = osi_lane->mutable_classification()->add_left_lane_boundary_id();							
								left_lane_bound_id->set_value(next_laneboundary_global_id);
							}
							if (lane_id > 0 )
							{								
								osi3::Identifier* right_lane_bound_id = osi_lane->mutable_classification()->add_right_lane_boundary_id();
								right_lane_bound_id->set_value(next_laneboundary_global_id);
							}
						}
					}   

					// STILL TO DO:
					int free_bound_id = 0;
					osi3::Identifier* free_lane_bound_id = osi_lane->mutable_classification()->add_free_lane_boundary_id();
					free_lane_bound_id->set_value(free_bound_id);

					// STILL TO DO:
					double temp = 0;
					osi_lane->mutable_classification()->mutable_road_condition()->set_surface_temperature(temp);
					osi_lane->mutable_classification()->mutable_road_condition()->set_surface_water_film(temp);
					osi_lane->mutable_classification()->mutable_road_condition()->set_surface_freezing_point(temp);
					osi_lane->mutable_classification()->mutable_road_condition()->set_surface_ice(temp);
					osi_lane->mutable_classification()->mutable_road_condition()->set_surface_roughness(temp);
					osi_lane->mutable_classification()->mutable_road_condition()->set_surface_texture(temp);

					mobj_osi_internal.ln.push_back(osi_lane);
				}
			}
		}
	}

	return 0;
}

const char* ScenarioGateway::GetOSISensorView(int* size)
{
	*size = osiSensorView.size;
	return osiSensorView.sensor_view.data();
}

const char* ScenarioGateway::GetOSIRoadLane(int* size, int object_id)
{
	// Check if object_id exists
	if (object_id >= getNumberOfObjects())
	{
		LOG("Object %d not available, only %d registered", object_id, getNumberOfObjects());
		*size = 0; 
		return 0; 
	}

	// Find position of the object
	roadmanager::Position pos; 
	for (size_t i = 0; i < getNumberOfObjects() ; i++)
	{
		if (object_id == objectState_[i]->state_.id)
		{
			pos = objectState_[i]->state_.pos;
		}
	}

	// find the lane in the sensor view and save its index in the sensor view
	int lane_id_of_vehicle = pos.GetLaneGlobalId();
	int idx;
	for (int i = 0; i<mobj_osi_internal.ln.size(); i++)
	{
		osi3::Identifier identifier = mobj_osi_internal.ln[i]->id();
		int found_id = (int)identifier.value();
		if (found_id == lane_id_of_vehicle)
		{
			idx = i;
			break; 
		}
	}

	// check if object is moving in the same direction of the lane centerline points
	//double teta = pos.GetHRelative(); 
	//bool driving_direction = IsAngleStraight(teta); 	

	// serialize to string the single lane
	mobj_osi_internal.ln[idx]->SerializeToString(&osiRoadLane.osi_lane_info);
	osiRoadLane.size = (unsigned int)mobj_osi_internal.ln[idx]->ByteSizeLong();
	*size = osiRoadLane.size;
	return osiRoadLane.osi_lane_info.data();
}


const char* ScenarioGateway::GetOSIRoadLaneBoundary(int* size, int global_id)
{
	// find the lane bounday in the sensor view and save its index
	int idx = -1;
	for (int i = 0; i<mobj_osi_internal.lnb.size(); i++)
	{
		osi3::Identifier identifier = mobj_osi_internal.lnb[i]->id();
		int found_id = (int)identifier.value();
		if (found_id == global_id)
		{
			idx = i;
			break; 
		}
	}

	if (idx == -1)
	{
		return 0; 
	}

	// serialize to string the single lane
	mobj_osi_internal.lnb[idx]->SerializeToString(&osiRoadLaneBoundary.osi_lane_boundary_info);
	osiRoadLaneBoundary.size = (unsigned int)mobj_osi_internal.lnb[idx]->ByteSizeLong();
	*size = osiRoadLaneBoundary.size;
	return osiRoadLaneBoundary.osi_lane_boundary_info.data();
}

bool ScenarioGateway::IsCentralOSILane(int lane_idx)
{
	// to check if the lane is a central lane we check if the right and left lane boundary have the same global id. 
	osi3::Identifier Left_lb_id = mobj_osi_internal.ln[lane_idx]->mutable_classification()->left_lane_boundary_id(0);
	int left_lb_id = (int)Left_lb_id.value();

	osi3::Identifier Right_lb_id = mobj_osi_internal.ln[lane_idx]->mutable_classification()->right_lane_boundary_id(0);
	int right_lb_id = (int)Right_lb_id.value();

	if (left_lb_id == right_lb_id) 
	{
		return true; 
	}
	else
	{
		return false; 
	}
}

int ScenarioGateway::GetLaneIdxfromIdOSI(int lane_id)
{
	int idx = -1; 
	for (int i = 0; i<mobj_osi_internal.ln.size(); i++)
	{
		osi3::Identifier identifier = mobj_osi_internal.ln[i]->id();
		int found_id = (int)identifier.value(); 
		if (found_id == lane_id)
		{
			idx = i; 
			break; 
		}
	}
	return idx; 
}

void ScenarioGateway::GetOSILaneBoundaryIds(std::vector<int> &ids, int object_id)
{
	int idx_central, idx_left, idx_right; 
	int left_lb_id, right_lb_id; 
	int far_left_lb_id, far_right_lb_id;  
	std::vector<int> final_lb_ids;   

	// Check if object_id exists
	if (object_id >= getNumberOfObjects())
	{
		LOG("Object %d not available, only %d registered", object_id, getNumberOfObjects());
		ids = {-1, -1, -1, -1}; 
		return; 
	}	

	// Find position of the object 
	roadmanager::Position pos;
	for (size_t i = 0; i < getNumberOfObjects() ; i++)
	{
		if (object_id == objectState_[i]->state_.id)
		{
			pos = objectState_[i]->state_.pos;
		}		
	} 

	// find the lane in the sensor view and save its index
	int lane_id_of_vehicle = pos.GetLaneGlobalId();
	idx_central = GetLaneIdxfromIdOSI(lane_id_of_vehicle); 
	
	// find left and right lane boundary ids of central lane	
	if (mobj_osi_internal.ln[idx_central]->mutable_classification()->left_lane_boundary_id_size() == 0 )
	{
		left_lb_id = -1;  
	}
	else
	{
		osi3::Identifier left_lane = mobj_osi_internal.ln[idx_central]->mutable_classification()->left_lane_boundary_id(0);
		left_lb_id = (int)left_lane.value(); 
	}

	if (mobj_osi_internal.ln[idx_central]->mutable_classification()->right_lane_boundary_id_size() == 0 )
	{
		right_lb_id = -1; 
	}
	else
	{
		osi3::Identifier right_lane = mobj_osi_internal.ln[idx_central]->mutable_classification()->right_lane_boundary_id(0);
		right_lb_id = (int)right_lane.value(); 
	}

	// find first left lane
	if (mobj_osi_internal.ln[idx_central]->mutable_classification()->left_adjacent_lane_id_size() == 0 )
	{ 
		far_left_lb_id = -1; 
	}
	else
	{

		osi3::Identifier Left_lane_id = mobj_osi_internal.ln[idx_central]->mutable_classification()->left_adjacent_lane_id(0);
		int left_lane_id = (int)Left_lane_id.value(); 	
		idx_left = GetLaneIdxfromIdOSI(left_lane_id); 
		if (IsCentralOSILane(idx_left))
		{
			// if it is central lane -> we look for its left one
			if (mobj_osi_internal.ln[idx_left]->mutable_classification()->left_adjacent_lane_id_size() == 0 )
			{
				far_right_lb_id = -1; 
			} 
			else
			{
				Left_lane_id = mobj_osi_internal.ln[idx_left]->mutable_classification()->left_adjacent_lane_id(0);
				left_lane_id = (int)Left_lane_id.value(); 	
				idx_left = GetLaneIdxfromIdOSI(left_lane_id);
			} 
		}
		// save left boundary of left lane as far left lane boundary of central lane
		if (mobj_osi_internal.ln[idx_left]->mutable_classification()->left_lane_boundary_id_size() == 0 )
		{
			far_right_lb_id = -1; 
		}
		else
		{
			osi3::Identifier Far_left_lb_id = mobj_osi_internal.ln[idx_left]->mutable_classification()->left_lane_boundary_id(0);
			far_left_lb_id = (int)Far_left_lb_id.value();
		}
		
	}
		

	// now find first right lane
	if (mobj_osi_internal.ln[idx_central]->mutable_classification()->right_adjacent_lane_id_size() == 0 )
	{
		far_right_lb_id = -1; 
	}
	else
	{
		osi3::Identifier Right_lane_id = mobj_osi_internal.ln[idx_central]->mutable_classification()->right_adjacent_lane_id(0);
		int right_lane_id = (int)Right_lane_id.value();   
		idx_right = GetLaneIdxfromIdOSI(right_lane_id);
		if (IsCentralOSILane(idx_right))
		{
			// if it is central lane -> we look for its right one 
			if (mobj_osi_internal.ln[idx_right]->mutable_classification()->right_adjacent_lane_id_size() == 0 )
			{
				far_right_lb_id = -1; 
			}
			else
			{
				Right_lane_id = mobj_osi_internal.ln[idx_right]->mutable_classification()->right_adjacent_lane_id(0);
				right_lane_id = (int)Right_lane_id.value(); 	 
				idx_right = GetLaneIdxfromIdOSI(right_lane_id); 
			}
		}
		// save right boundary of right lane as far right lane boundary of central lane 
		if (mobj_osi_internal.ln[idx_right]->mutable_classification()->right_lane_boundary_id_size() == 0 )
		{
			far_right_lb_id = -1; 
		}
		else
		{
			osi3::Identifier Far_right_lb_id = mobj_osi_internal.ln[idx_right]->mutable_classification()->right_lane_boundary_id(0);
			far_right_lb_id = (int)Far_right_lb_id.value();
		}
		 
	}
		
	// push all ids into output vector 
	final_lb_ids.push_back(far_left_lb_id);
	final_lb_ids.push_back(left_lb_id);
	final_lb_ids.push_back(right_lb_id); 
	final_lb_ids.push_back(far_right_lb_id);

	ids = final_lb_ids; 

	return;
}

void ScenarioGateway::updateObjectInfo(ObjectState* obj_state, double timestamp, double speed, double wheel_angle, double wheel_rot)
{
	if (!obj_state)
	{
		return;
	}

	obj_state->state_.speed = (float)speed;
	obj_state->state_.timeStamp = (float)timestamp;
	obj_state->state_.wheel_angle = (float)wheel_angle;
	obj_state->state_.wheel_rot = (float)wheel_rot;

	// Write status to file - for later replay
	if (data_file_.is_open())
	{
		data_file_.write((char*)(&obj_state->state_), sizeof(obj_state->state_));
	}
}

void ScenarioGateway::reportObject(int id, std::string name, int model_id, int control, OSCBoundingBox boundingbox,
	double timestamp, double speed, double wheel_angle, double wheel_rot,
	roadmanager::Position* pos)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
		obj_state = new ObjectState(id, name, model_id, control, boundingbox, timestamp, speed, wheel_angle, wheel_rot, pos);

		// Add object to collection
		objectState_.push_back(obj_state);
	}
	else
	{
		// Update status
		obj_state->state_.pos = *pos;
		updateObjectInfo(obj_state, timestamp, speed, wheel_angle, wheel_rot);
	}
}

void ScenarioGateway::reportObject(int id, std::string name, int model_id, int control, OSCBoundingBox boundingbox,
	double timestamp, double speed, double wheel_angle, double wheel_rot,
	double x, double y, double z, double h, double p, double r)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
		obj_state = new ObjectState(id, name, model_id, control, boundingbox, timestamp, speed, wheel_angle, wheel_rot, x, y, z, h, p, r);

		// Add object to collection
		objectState_.push_back(obj_state);
	}
	else
	{
		// Update status
		obj_state->state_.pos.SetInertiaPos(x, y, z, h, p, r);
		updateObjectInfo(obj_state, timestamp, speed, wheel_angle, wheel_rot);
	}
}

void ScenarioGateway::reportObject(int id, std::string name, int model_id, int control, OSCBoundingBox boundingbox,
	double timestamp, double speed, double wheel_angle, double wheel_rot,
	int roadId, int laneId, double laneOffset, double s)
{
	ObjectState* obj_state = getObjectStatePtrById(id);

	if (obj_state == 0)
	{
		// Create state and set permanent information
		LOG("Creating new object \"%s\" (id %d, timestamp %.2f)", name.c_str(), id, timestamp);
		obj_state = new ObjectState(id, name, model_id, control, boundingbox,timestamp, speed, wheel_angle, wheel_rot, roadId, laneId, laneOffset, s);

		// Add object to collection
		objectState_.push_back(obj_state);
	}
	else
	{
		// Update status
		obj_state->state_.pos.SetLanePos(roadId, laneId, s, laneOffset);
		updateObjectInfo(obj_state, timestamp, speed, wheel_angle, wheel_rot);
	}
}

void ScenarioGateway::removeObject(int id)
{
	for (size_t i = 0; i < objectState_.size(); i++) 
	{
		if (objectState_[i]->state_.id == id) 
		{
			objectState_.erase(objectState_.begin() + i);
		}
	}
}

void ScenarioGateway::removeObject(std::string name)
{
	for (size_t i = 0; i < objectState_.size(); i++) 
	{
		if (objectState_[i]->state_.name == name) 
		{
			objectState_.erase(objectState_.begin() + i);
		}
	}
}

int ScenarioGateway::RecordToFile(std::string filename, std::string odr_filename, std::string  model_filename)
{
	if (!filename.empty())
	{
		data_file_.open(filename, std::ofstream::binary);
		if (data_file_.fail())
		{
			LOG("Cannot open file: %s", filename.c_str());
			return -1;
		}
		ReplayHeader header;
		strncpy(header.odr_filename, FileNameOf(odr_filename).c_str(), REPLAY_FILENAME_SIZE);
		strncpy(header.model_filename, FileNameOf(model_filename).c_str(), REPLAY_FILENAME_SIZE);

		data_file_.write((char*)&header, sizeof(header));
	}

	return 0;
}

SumoController::SumoController(Entities* entities, ScenarioGateway* scenarioGateway)
{
	// initalize sumo with the configuration file
	entities_ = entities;
	scenarioGateway_ = scenarioGateway;
	std::vector<std::string> options;

	options.push_back("-c " + entities_->sumo_config_path);
	options.push_back("--xml-validation");
	options.push_back("never");
	
	libsumo::Simulation::load(options);
	sumo_used = true;
}

SumoController::SumoController()
{
	sumo_used = false;
}

void SumoController::InitalizeObjects()
{	
	// Adds all vehicles added in openscenario to the sumosimulation
	if (sumo_used)
	{
		for (size_t j = 0; j < entities_->object_.size(); j++)
		{
			if (entities_->object_[j]->control_ != Object::Control::HYBRID_GHOST) 
			{
				libsumo::Vehicle::add(entities_->object_[j]->name_,"");
			}
		}
		updatePositions();
	}
}

void SumoController::updatePositions()
{
	// Updates all positions for non-sumo controlled vehicles
	if (sumo_used)
	{
		for (size_t i = 0; i < entities_->object_.size(); i++)
		{
			if ((entities_->object_[i]->control_ != Object::Control::SUMO) && (entities_->object_[i]->control_ != Object::Control::HYBRID_GHOST))
			{
				libsumo::Vehicle::moveToXY(entities_->object_[i]->name_,"random",0,entities_->object_[i]->pos_.GetX()+entities_->sumo_x_offset,entities_->object_[i]->pos_.GetY()+entities_->sumo_y_offset,entities_->object_[i]->pos_.GetH(),0);
				libsumo::Vehicle::setSpeed(entities_->object_[i]->name_,entities_->object_[i]->speed_);
			}
		}
	}
}

void SumoController::step(double time)
{
	// stepping funciton for sumo, adds/removes vehicles (based on sumo), 
	// updates all positions of vehicles in the simulation that are controlled by sumo
	if (sumo_used)
	{
		// do sumo timestep
		libsumo::Simulation::step(time);

		// check if any new cars has been added by sumo and add them to entities
		if (libsumo::Simulation::getDepartedNumber() > 0) {
			std::vector<std::string> deplist = libsumo::Simulation::getDepartedIDList();
			for (size_t i = 0; i < deplist.size(); i++)
			{
				if (!entities_->nameExists(deplist[i]))
				{
					Vehicle *vehicle = new Vehicle();
					// copy the default vehicle stuff here (add bounding box and so on)
					LOG("Adding new vehicle: %s",deplist[i].c_str());
					vehicle->name_ = deplist[i];
					vehicle->control_ = Object::Control::SUMO;
					vehicle->model_filepath_ = entities_->sumo_vehicle->model_filepath_;
					entities_->addObject(vehicle);
				}
			}
		}

		// check if any cars have been removed by sumo and remove them from scenarioGateway and entities
		if (libsumo::Simulation::getArrivedNumber() > 0) {
			std::vector<std::string> arrivelist = libsumo::Simulation::getArrivedIDList();
			for (size_t i = 0; i < arrivelist.size();i++)
			{
				for (size_t j = 0; j < entities_->object_.size(); j++)
				{
					if (arrivelist[i] == entities_->object_[j]->name_) {
						LOG("Removing vehicle: %s",arrivelist[i].c_str());
						entities_->removeObject(arrivelist[i]);
						scenarioGateway_->removeObject(arrivelist[i]);
					}
				}
			}
		}

		// Update the position of all cars controlled by sumo
		for (size_t i = 0; i < entities_->object_.size(); i++)
		{
			if (entities_->object_[i]->control_ == Object::Control::SUMO)
			{
				std::string sumoid = entities_->object_[i]->name_;
				libsumo::TraCIPosition pos = libsumo::Vehicle::getPosition3D(sumoid);
				entities_->object_[i]->speed_ = libsumo::Vehicle::getSpeed(sumoid);
				entities_->object_[i]->pos_.SetInertiaPos(pos.x-entities_->sumo_x_offset,pos.y-entities_->sumo_y_offset,pos.z,-libsumo::Vehicle::getAngle(sumoid)*3.14159265359/180+ 3.14159265359/2,libsumo::Vehicle::getSlope(sumoid)*3.14159265359/180,0);
			}
        }
	}
}