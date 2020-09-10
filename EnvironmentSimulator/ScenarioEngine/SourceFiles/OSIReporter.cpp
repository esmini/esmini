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

#include "CommonMini.hpp"
#include "OSIReporter.hpp"
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

#define OSI_OUT_PORT 48198

static struct {
	osi3::SensorView *sv;
	std::vector<osi3::StationaryObject*> sobj;
	std::vector<osi3::MovingObject*> mobj;
	std::vector<osi3::Lane*> ln;
	std::vector<osi3::LaneBoundary*> lnb;
} obj_osi_internal;

typedef struct
{
	std::string sensor_view;
	unsigned int size;
} OSISensorView;

typedef struct
{
	std::string osi_lane_info;
	unsigned int size;
} OSIRoadLane;

typedef struct
{
	std::string osi_lane_boundary_info;
	unsigned int size;
} OSIRoadLaneBoundary;

using namespace scenarioengine;

static OSISensorView osiSensorView;
static OSIRoadLane osiRoadLane;
static OSIRoadLaneBoundary osiRoadLaneBoundary;
std::ofstream osi_file;

static struct sockaddr_in recvAddr;

// ScenarioGateway

OSIReporter::OSIReporter()
{
	sendSocket = 0;

	obj_osi_internal.sv = new osi3::SensorView();

	obj_osi_internal.sv->mutable_version()->set_version_major(3);
	obj_osi_internal.sv->mutable_version()->set_version_minor(0);
	obj_osi_internal.sv->mutable_version()->set_version_patch(0);

	obj_osi_internal.sv->mutable_timestamp()->set_seconds(0);
	obj_osi_internal.sv->mutable_timestamp()->set_seconds(0);
}


OSIReporter::~OSIReporter()
{
	if (obj_osi_internal.sv)
	{
		obj_osi_internal.sv->Clear();
		delete obj_osi_internal.sv;
	}

	obj_osi_internal.mobj.clear();
	obj_osi_internal.ln.clear();
	obj_osi_internal.lnb.clear();

	osiSensorView.size = 0;
	osiRoadLane.size=0;

	CloseSocket();
	if (osi_file.is_open())
	{
		osi_file.close();
	}
	
}

int OSIReporter::OpenSocket(std::string ipaddr)
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

int OSIReporter::CloseSocket()
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

void OSIReporter::ReportSensors(std::vector<ObjectSensor*> sensor)
{
	const int max_detections = 10;

	if (sensor.size() == 0)
	{
		return;
	}

	// Below is just some stub code hinting how ideal sensor data can be collected
	// Remains to implement OSI output :)

	printf("Sensor detections:\n");
	
	for (int i = 0; i < sensor.size(); i++)
	{
		for (int j = 0; j < sensor[i]->nObj_; j++)
		{
			printf("  sensor %d obj_id: %d pos: %.2f, %.2f \n", i, sensor[i]->hitList_[j].obj_->id_, sensor[i]->hitList_[j].x_, sensor[i]->hitList_[j].y_);
		}
	}
}

bool OSIReporter::OpenOSIFile()
{
	osi_file = std::ofstream("move_obj.osi", std::ios_base::binary);
	if (!osi_file.good())
	{
		LOG("Failed open osi file");
		return false; 
	}
	return true; 
}

bool OSIReporter::WriteOSIFile()
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

int OSIReporter::UpdateOSISensorView(std::vector<ObjectState*> objectState)
{
	double time_stamp = objectState[0]->state_.timeStamp;

	obj_osi_internal.sv->mutable_global_ground_truth()->mutable_timestamp()->set_seconds((int64_t)objectState[0]->state_.timeStamp);
	obj_osi_internal.sv->mutable_global_ground_truth()->mutable_timestamp()->set_nanos((uint32_t)(
		(objectState[0]->state_.timeStamp - (int64_t)objectState[0]->state_.timeStamp) * 1e9)
	);

	for (size_t i = 0; i < objectState.size(); i++)
	{
		if(objectState[i]->state_.obj_type==static_cast<int>(Object::Type::VEHICLE) || 
		objectState[i]->state_.obj_type==static_cast<int>(Object::Type::PEDESTRIAN))
		{
			UpdateOSIMovingObject(objectState[i]);
		}
		else if(objectState[i]->state_.obj_type==static_cast<int>(Object::Type::MISC_OBJECT))
		{
			UpdateOSIStationaryObject(objectState[i]);
		}
		else
		{
			LOG("Warning: Object type %d is not supported in OSIReporter, and hence no OSI update for this object", objectState[i]->state_.obj_type);
		}
	}
	
	//collect all information of lanes in the lane section where obj=0 is
	UpdateOSIRoadLane(objectState);

	UpdateOSILaneBoundary(objectState);

	obj_osi_internal.sv->SerializeToString(&osiSensorView.sensor_view);
	osiSensorView.size = (unsigned int)obj_osi_internal.sv->ByteSizeLong();

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

int OSIReporter::UpdateOSIStationaryObject(ObjectState* objectState)
{
	// Create OSI Stationary object
	osi3::StationaryObject* sobj = obj_osi_internal.sv->mutable_global_ground_truth()->add_stationary_object();
	sobj->mutable_id()->set_value(obj_osi_internal.sobj.size());
	
	// Set OSI Stationary Object Type and Classification
	if(objectState->state_.obj_category == static_cast<int>(Object::Type::MISC_OBJECT))
	{
		if(objectState->state_.obj_category == static_cast<int>(MiscObject::Category::NONE))
		{
			sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_UNKNOWN);
		} 
		else if (objectState->state_.obj_category == static_cast<int>(MiscObject::Category::POLE))
		{
			sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_POLE); 
		}
		else if (objectState->state_.obj_category == static_cast<int>(MiscObject::Category::TREE))
		{
			sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_TREE); 
		}
		else if (objectState->state_.obj_category == static_cast<int>(MiscObject::Category::VEGETATION))
		{
			sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_VEGETATION); 
		}
		else if (objectState->state_.obj_category == static_cast<int>(MiscObject::Category::BARRIER))
		{
			sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_BARRIER); 
		}
		else if (objectState->state_.obj_category == static_cast<int>(MiscObject::Category::BUILDING))
		{
			sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_BUILDING); 
		}
		else if (objectState->state_.obj_category == static_cast<int>(MiscObject::Category::OBSTACLE) ||
				objectState->state_.obj_category == static_cast<int>(MiscObject::Category::PARKINGSPACE) || 
				objectState->state_.obj_category == static_cast<int>(MiscObject::Category::RAILING) ||
				objectState->state_.obj_category == static_cast<int>(MiscObject::Category::PATCH) ||
				objectState->state_.obj_category == static_cast<int>(MiscObject::Category::TRAFFICISLAND) ||
				objectState->state_.obj_category == static_cast<int>(MiscObject::Category::CROSSWALK) ||
				objectState->state_.obj_category == static_cast<int>(MiscObject::Category::STREETLAMP) ||
				objectState->state_.obj_category == static_cast<int>(MiscObject::Category::GANTRY) ||
				objectState->state_.obj_category == static_cast<int>(MiscObject::Category::SOUNDBARRIER) ||
				objectState->state_.obj_category == static_cast<int>(MiscObject::Category::WIND) ||
				objectState->state_.obj_category == static_cast<int>(MiscObject::Category::ROADMARK))
		{
			sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_OTHER); 
		}
		else
		{
			sobj->mutable_classification()->set_type(osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_UNKNOWN); 
			LOG("OSIReporter::UpdateOSIStationaryObject -> Unsupported stationary object category");
		}
	}
	else
	{
		LOG("OSIReporter::UpdateOSIStationaryObject -> Unsupported stationary object type");
	}

	// Set OSI Stationary Object Boundingbox
	sobj->mutable_base()->mutable_dimension()->set_height(objectState->state_.boundingbox.dimensions_.height_);
	sobj->mutable_base()->mutable_dimension()->set_width(objectState->state_.boundingbox.dimensions_.width_);
	sobj->mutable_base()->mutable_dimension()->set_length(objectState->state_.boundingbox.dimensions_.length_);

	// Set OSI Stationary Object Position 
	sobj->mutable_base()->mutable_position()->set_x(objectState->state_.pos.GetX());
	sobj->mutable_base()->mutable_position()->set_y(objectState->state_.pos.GetY());
	sobj->mutable_base()->mutable_position()->set_z(objectState->state_.pos.GetZ());

	// Set OSI Stationary Object Orientation 
	sobj->mutable_base()->mutable_orientation()->set_roll(objectState->state_.pos.GetR());
	sobj->mutable_base()->mutable_orientation()->set_pitch(objectState->state_.pos.GetP());
	sobj->mutable_base()->mutable_orientation()->set_yaw(objectState->state_.pos.GetH());
	
	// Append OSI Stationary Object to OSI structure
	obj_osi_internal.sobj.push_back(sobj);
	return 0;
}

int OSIReporter::UpdateOSIMovingObject(ObjectState* objectState)
{
	// Create OSI Moving object
	osi3::MovingObject* mobj = obj_osi_internal.sv->mutable_global_ground_truth()->add_moving_object();
	mobj->mutable_id()->set_value(obj_osi_internal.mobj.size());
	
	// Set OSI Moving Object Type and Classification
	if(objectState->state_.obj_type == static_cast<int>(Object::Type::VEHICLE))
	{
		mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_VEHICLE);

		if(objectState->state_.obj_category == static_cast<int>(Vehicle::Category::CAR))
		{
			mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_MEDIUM_CAR); 
		} 
		else if (objectState->state_.obj_category == static_cast<int>(Vehicle::Category::BICYCLE))
		{
			mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_BICYCLE); 
		} 
		else
		{
			mobj->mutable_vehicle_classification()->set_type(osi3::MovingObject_VehicleClassification::TYPE_UNKNOWN); 
			LOG("OSIReporter::UpdateOSIMovingObject -> Unsupported moving object category");
		}
	}
	else if (objectState->state_.obj_type == static_cast<int>(Object::Type::PEDESTRIAN))
	{
		if (objectState->state_.obj_category!=static_cast<int>(Pedestrian::Category::PEDESTRIAN))
		{
			mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_PEDESTRIAN);
		}
		else if (objectState->state_.obj_category!=static_cast<int>(Pedestrian::Category::ANIMAL))
		{
			mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_ANIMAL);
		}
		else if (objectState->state_.obj_category!=static_cast<int>(Pedestrian::Category::WHEELCHAIR))
		{
			mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_OTHER);
		}
		else
		{
			mobj->set_type(osi3::MovingObject::Type::MovingObject_Type_TYPE_UNKNOWN);
			LOG("OSIReporter::UpdateOSIMovingObject -> Unsupported moving object category");
		}
	}
	else
	{
		LOG("OSIReporter::UpdateOSIMovingObject -> Unsupported moving object type");
	}

	// Set OSI Moving Object Control Type
	mobj->mutable_vehicle_attributes()->mutable_driver_id()->set_value((uint64_t)objectState->state_.control);

	// Set OSI Moving Object Boundingbox
	mobj->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->set_x((double)(objectState->state_.boundingbox.center_.x_));
	mobj->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->set_y((double)(objectState->state_.boundingbox.center_.y_));
	mobj->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->set_z((double)(objectState->state_.boundingbox.center_.z_));
	mobj->mutable_base()->mutable_dimension()->set_height(objectState->state_.boundingbox.dimensions_.height_);
	mobj->mutable_base()->mutable_dimension()->set_width(objectState->state_.boundingbox.dimensions_.width_);
	mobj->mutable_base()->mutable_dimension()->set_length(objectState->state_.boundingbox.dimensions_.length_);

	// Set OSI Moving Object Position 
	mobj->mutable_base()->mutable_position()->set_x(objectState->state_.pos.GetX());
	mobj->mutable_base()->mutable_position()->set_y(objectState->state_.pos.GetY());
	mobj->mutable_base()->mutable_position()->set_z(objectState->state_.pos.GetZ());

	// Set OSI Moving Object Orientation
	mobj->mutable_base()->mutable_orientation()->set_roll(objectState->state_.pos.GetR());
	mobj->mutable_base()->mutable_orientation()->set_pitch(objectState->state_.pos.GetP());
	mobj->mutable_base()->mutable_orientation()->set_yaw(objectState->state_.pos.GetH());
	mobj->mutable_base()->mutable_orientation_rate()->set_yaw(objectState->state_.pos.GetHRate());
	mobj->mutable_base()->mutable_orientation_acceleration()->set_yaw(objectState->state_.pos.GetHAcc());

	// Set OSI Moving Object Velocity 
	mobj->mutable_base()->mutable_velocity()->set_x(objectState->state_.pos.GetVelX());
	mobj->mutable_base()->mutable_velocity()->set_y(objectState->state_.pos.GetVelY());
	mobj->mutable_base()->mutable_velocity()->set_z(0);  // assume neglectable speed in z dimension

	// Set OSI Moving Object Acceleration 
	mobj->mutable_base()->mutable_acceleration()->set_x(objectState->state_.pos.GetAccX());
	mobj->mutable_base()->mutable_acceleration()->set_y(objectState->state_.pos.GetAccY());
	mobj->mutable_base()->mutable_acceleration()->set_z(0);  // assume neglectable speed in z dimension
	
	// Append OSI Moving Object to OSI structure
	obj_osi_internal.mobj.push_back(mobj);

	return 0;
}

int OSIReporter::UpdateOSILaneBoundary(std::vector<ObjectState*> objectState)
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
								for (int h=0; h < obj_osi_internal.lnb.size(); h++)
								{
									if (obj_osi_internal.lnb[h]->mutable_id()->value() == line_id)
									{
										osi_laneboundary = obj_osi_internal.lnb[h];
									}
								}
								if (!osi_laneboundary)
								{
									osi_laneboundary = obj_osi_internal.sv->mutable_global_ground_truth()->add_lane_boundary();

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

									obj_osi_internal.lnb.push_back(osi_laneboundary);

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
					for (int h=0; h < obj_osi_internal.lnb.size(); h++)
					{
						if (obj_osi_internal.lnb[h]->mutable_id()->value() == boundary_id)
						{
							osi_laneboundary = obj_osi_internal.lnb[h];
						}
					}
					if (!osi_laneboundary)
					{
						osi_laneboundary = obj_osi_internal.sv->mutable_global_ground_truth()->add_lane_boundary();

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

						obj_osi_internal.lnb.push_back(osi_laneboundary);
					}
				}
			}
		}
	}

	return 0;
}

int OSIReporter::UpdateOSIRoadLane(std::vector<ObjectState*> objectState)
{
	// Find ego vehicle 
	roadmanager::Position pos;
	for (size_t i = 0; i < objectState.size() ; i++)
	{
		if (objectState[i]->state_.control == 3) // external hybrid is host 
		{
			pos = objectState[i]->state_.pos;
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
				for (int jj=0; jj < obj_osi_internal.ln.size(); jj++)
				{
					if (obj_osi_internal.ln[jj]->mutable_id()->value() == lane_global_id)
					{
						osi_lane = obj_osi_internal.ln[jj];

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
					osi_lane = obj_osi_internal.sv->mutable_global_ground_truth()->add_lane();
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
					int n_osi_points = lane->GetOSIPoints()->GetNumOfOSIPoints();
					for (int jj = 0; jj < n_osi_points; jj++)
					{
						osi3::Vector3d* centerLine = osi_lane->mutable_classification()->add_centerline();
						centerLine->set_x(lane->GetOSIPoints()->GetXfromIdx(jj));
						centerLine->set_y(lane->GetOSIPoints()->GetYfromIdx(jj));
						centerLine->set_z(lane->GetOSIPoints()->GetZfromIdx(jj));
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

					obj_osi_internal.ln.push_back(osi_lane);
				}
			}
		}
	}

	return 0;
}

const char* OSIReporter::GetOSISensorView(int* size)
{
	*size = osiSensorView.size;
	return osiSensorView.sensor_view.data();
}

const char* OSIReporter::GetOSIRoadLane(std::vector<ObjectState*> objectState, int* size, int object_id)
{
	// Check if object_id exists
	if (object_id >= objectState.size())
	{
		LOG("Object %d not available, only %d registered", object_id, objectState.size());
		*size = 0; 
		return 0; 
	}

	// Find position of the object
	roadmanager::Position pos; 
	for (size_t i = 0; i < objectState.size() ; i++)
	{
		if (object_id == objectState[i]->state_.id)
		{
			pos = objectState[i]->state_.pos;
		}
	}

	// find the lane in the sensor view and save its index in the sensor view
	int lane_id_of_vehicle = pos.GetLaneGlobalId();
	int idx;
	for (int i = 0; i<obj_osi_internal.ln.size(); i++)
	{
		osi3::Identifier identifier = obj_osi_internal.ln[i]->id();
		int found_id = (int)identifier.value();
		if (found_id == lane_id_of_vehicle)
		{
			idx = i;
			break; 
		}
	}

	// serialize to string the single lane
	obj_osi_internal.ln[idx]->SerializeToString(&osiRoadLane.osi_lane_info);
	osiRoadLane.size = (unsigned int)obj_osi_internal.ln[idx]->ByteSizeLong();
	*size = osiRoadLane.size;
	return osiRoadLane.osi_lane_info.data();
}


const char* OSIReporter::GetOSIRoadLaneBoundary(int* size, int global_id)
{
	// find the lane bounday in the sensor view and save its index
	int idx = -1;
	for (int i = 0; i<obj_osi_internal.lnb.size(); i++)
	{
		osi3::Identifier identifier = obj_osi_internal.lnb[i]->id();
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
	obj_osi_internal.lnb[idx]->SerializeToString(&osiRoadLaneBoundary.osi_lane_boundary_info);
	osiRoadLaneBoundary.size = (unsigned int)obj_osi_internal.lnb[idx]->ByteSizeLong();
	*size = osiRoadLaneBoundary.size;
	return osiRoadLaneBoundary.osi_lane_boundary_info.data();
}

bool OSIReporter::IsCentralOSILane(int lane_idx)
{
	// to check if the lane is a central lane we check if the right and left lane boundary have the same global id. 
	osi3::Identifier Left_lb_id = obj_osi_internal.ln[lane_idx]->mutable_classification()->left_lane_boundary_id(0);
	int left_lb_id = (int)Left_lb_id.value();

	osi3::Identifier Right_lb_id = obj_osi_internal.ln[lane_idx]->mutable_classification()->right_lane_boundary_id(0);
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

int OSIReporter::GetLaneIdxfromIdOSI(int lane_id)
{
	int idx = -1; 
	for (int i = 0; i<obj_osi_internal.ln.size(); i++)
	{
		osi3::Identifier identifier = obj_osi_internal.ln[i]->id();
		int found_id = (int)identifier.value(); 
		if (found_id == lane_id)
		{
			idx = i; 
			break; 
		}
	}
	return idx; 
}

void OSIReporter::GetOSILaneBoundaryIds(std::vector<ObjectState*> objectState, std::vector<int> &ids, int object_id)
{
	int idx_central, idx_left, idx_right; 
	int left_lb_id, right_lb_id; 
	int far_left_lb_id, far_right_lb_id;  
	std::vector<int> final_lb_ids;   

	// Check if object_id exists
	if (object_id >= objectState.size())
	{
		LOG("Object %d not available, only %d registered", object_id, objectState.size());
		ids = {-1, -1, -1, -1}; 
		return; 
	}	

	// Find position of the object 
	roadmanager::Position pos;
	for (size_t i = 0; i < objectState.size(); i++)
	{
		if (object_id == objectState[i]->state_.id)
		{
			pos = objectState[i]->state_.pos;
		}		
	} 

	// find the lane in the sensor view and save its index
	int lane_id_of_vehicle = pos.GetLaneGlobalId();
	idx_central = GetLaneIdxfromIdOSI(lane_id_of_vehicle); 
	
	// find left and right lane boundary ids of central lane	
	if (obj_osi_internal.ln[idx_central]->mutable_classification()->left_lane_boundary_id_size() == 0 )
	{
		left_lb_id = -1;  
	}
	else
	{
		osi3::Identifier left_lane = obj_osi_internal.ln[idx_central]->mutable_classification()->left_lane_boundary_id(0);
		left_lb_id = (int)left_lane.value(); 
	}

	if (obj_osi_internal.ln[idx_central]->mutable_classification()->right_lane_boundary_id_size() == 0 )
	{
		right_lb_id = -1; 
	}
	else
	{
		osi3::Identifier right_lane = obj_osi_internal.ln[idx_central]->mutable_classification()->right_lane_boundary_id(0);
		right_lb_id = (int)right_lane.value(); 
	}

	// find first left lane
	if (obj_osi_internal.ln[idx_central]->mutable_classification()->left_adjacent_lane_id_size() == 0 )
	{ 
		far_left_lb_id = -1; 
	}
	else
	{

		osi3::Identifier Left_lane_id = obj_osi_internal.ln[idx_central]->mutable_classification()->left_adjacent_lane_id(0);
		int left_lane_id = (int)Left_lane_id.value(); 	
		idx_left = GetLaneIdxfromIdOSI(left_lane_id); 
		if (IsCentralOSILane(idx_left))
		{
			// if it is central lane -> we look for its left one
			if (obj_osi_internal.ln[idx_left]->mutable_classification()->left_adjacent_lane_id_size() == 0 )
			{
				far_right_lb_id = -1; 
			} 
			else
			{
				Left_lane_id = obj_osi_internal.ln[idx_left]->mutable_classification()->left_adjacent_lane_id(0);
				left_lane_id = (int)Left_lane_id.value(); 	
				idx_left = GetLaneIdxfromIdOSI(left_lane_id);
			} 
		}
		// save left boundary of left lane as far left lane boundary of central lane
		if (obj_osi_internal.ln[idx_left]->mutable_classification()->left_lane_boundary_id_size() == 0 )
		{
			far_right_lb_id = -1; 
		}
		else
		{
			osi3::Identifier Far_left_lb_id = obj_osi_internal.ln[idx_left]->mutable_classification()->left_lane_boundary_id(0);
			far_left_lb_id = (int)Far_left_lb_id.value();
		}
		
	}
		

	// now find first right lane
	if (obj_osi_internal.ln[idx_central]->mutable_classification()->right_adjacent_lane_id_size() == 0 )
	{
		far_right_lb_id = -1; 
	}
	else
	{
		osi3::Identifier Right_lane_id = obj_osi_internal.ln[idx_central]->mutable_classification()->right_adjacent_lane_id(0);
		int right_lane_id = (int)Right_lane_id.value();   
		idx_right = GetLaneIdxfromIdOSI(right_lane_id);
		if (IsCentralOSILane(idx_right))
		{
			// if it is central lane -> we look for its right one 
			if (obj_osi_internal.ln[idx_right]->mutable_classification()->right_adjacent_lane_id_size() == 0 )
			{
				far_right_lb_id = -1; 
			}
			else
			{
				Right_lane_id = obj_osi_internal.ln[idx_right]->mutable_classification()->right_adjacent_lane_id(0);
				right_lane_id = (int)Right_lane_id.value(); 	 
				idx_right = GetLaneIdxfromIdOSI(right_lane_id); 
			}
		}
		// save right boundary of right lane as far right lane boundary of central lane 
		if (obj_osi_internal.ln[idx_right]->mutable_classification()->right_lane_boundary_id_size() == 0 )
		{
			far_right_lb_id = -1; 
		}
		else
		{
			osi3::Identifier Far_right_lb_id = obj_osi_internal.ln[idx_right]->mutable_classification()->right_lane_boundary_id(0);
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
