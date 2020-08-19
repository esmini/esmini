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

/*
 * This application is an example of how to integrate a simple Ego vehicle into the environment simulator
 * The vehicle is implemented in a separate module (vehicle.cpp)
 * Communication is by means of function calls
 *
 * Ego means that the vehicle is controlled externally, e.g. interactively by a human-in-the-loop
 * or by an AD controller or by an external test framework providing stimuli for a vehicle simulator
 *
 * This application support three types of vehicle controls:
 * 1. Internal: Scenario engine executes according to OpenSCENARIO description (default)
 * 2. External: The vehicle is controlled by the user.
 * 3. Hyybrid: Automatic driver model following a ghost vehicle performing the scenario as i 1.
 */


#define _USE_MATH_DEFINES
#include <math.h>

#include "playerbase.hpp"
#include "vehicle.hpp"
#include "CommonMini.cpp"


static vehicle::Vehicle *ego;
static int ego_id = -1;

#define MIN_TIME_STEP 0.01
#define MAX_TIME_STEP 0.1

typedef struct
{
	scenarioengine::Object *obj;
	vehicle::Vehicle *dyn_model;
#ifdef _SCENARIO_VIEWER
	viewer::CarModel *gfx_model;
#endif

	double speed_target_speed;
	double steering_target_heading;
} ExternVehicle;

static std::vector<ExternVehicle> extern_vehicle;

int SetupExternVehicles(ScenarioPlayer *player)
{

	for (size_t i = 0; i < player->scenarioEngine->entities.object_.size(); i++)
	{
		Object *obj = player->scenarioEngine->entities.object_[i];

		if (obj->GetControl() == Object::Control::EXTERNAL ||
			obj->GetControl() == Object::Control::HYBRID_EXTERNAL)
		{
			ExternVehicle vh;

			if (ego_id != -1)
			{
				LOG("Only one Ego vehicle (external control) supported. Already registered id %d. Ignoring this (%d)", ego_id, i);
			}
			else
			{
				LOG("Registering Ego id %d", i);
				ego_id = (int)extern_vehicle.size();
			}

			vh.steering_target_heading = 0;
			vh.speed_target_speed = 0;

#ifdef _SCENARIO_VIEWER
			vh.gfx_model = player->viewer_->cars_[i];
			if (obj->GetControl() == Object::Control::HYBRID_EXTERNAL)
			{
				player->viewer_->SensorSetPivotPos(vh.gfx_model->speed_sensor_, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ());
				player->viewer_->SensorSetTargetPos(vh.gfx_model->speed_sensor_, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ());

				player->viewer_->SensorSetPivotPos(vh.gfx_model->steering_sensor_, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ());
				player->viewer_->SensorSetTargetPos(vh.gfx_model->steering_sensor_, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ());
			}
			vh.dyn_model = new vehicle::Vehicle(obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetH(), vh.gfx_model->size_x);
#else
			vh.dyn_model = new vehicle::Vehicle(obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetH(), 5.0);
#endif
			vh.obj = obj;

			extern_vehicle.push_back(vh);
		}
	}

	return 0;
}


void UpdateExternVehicles(double deltaTimeStep, ScenarioPlayer *player)
{
	if (player->scenarioEngine->getSimulationTime() <= 0)
	{
		return;
	}
	for (size_t i = 0; i < extern_vehicle.size(); i++)
	{
		ExternVehicle *vh = &extern_vehicle[i];
		Object *obj = vh->obj;

		double speed_limit = obj->pos_.GetSpeedLimit(); /// -----> HERE
		if (speed_limit < SMALL_NUMBER)
		{
			// no speed limit defined, set something with regards to number of lanes
			if (obj->pos_.GetRoadById(obj->pos_.GetTrackId())->GetNumberOfDrivingLanesSide(
				obj->pos_.GetS(), SIGN(obj->pos_.GetLaneId())) > 1)
			{
				speed_limit = 110 / 3.6;
			}
			else
			{
				speed_limit = 60 / 3.6;
			}
		}
		vh->dyn_model->SetMaxSpeed(speed_limit);

		if (obj->GetControl() == Object::Control::EXTERNAL)
		{
			if (i == ego_id)
			{
				vehicle::THROTTLE accelerate = vehicle::THROTTLE_NONE;
				vehicle::STEERING steer = vehicle::STEERING_NONE;
#ifdef _SCENARIO_VIEWER

				if (player->viewer_->getKeyUp())
				{
					accelerate = vehicle::THROTTLE_ACCELERATE;
				}
				else if (player->viewer_->getKeyDown())
				{
					accelerate = vehicle::THROTTLE_BRAKE;
				}

				if (player->viewer_->getKeyLeft())
				{
					steer = vehicle::STEERING_LEFT;
				}
				else if (player->viewer_->getKeyRight())
				{
					steer = vehicle::STEERING_RIGHT;
				}
#endif
				// Update vehicle motion
				vh->dyn_model->DrivingControlBinary(deltaTimeStep, accelerate, steer);
			}
		}
		else if (obj->GetControl() == Object::Control::HYBRID_EXTERNAL)
		{
			// Set steering target point at a distance ahead proportional to the speed
			double speed_target_distance = MAX(7, vh->obj->speed_ * 2.0);
			double steering_target_distance = MAX(5, 0.25 * speed_target_distance);

			// find out what direction is forward, according to vehicle relative road heading
			if (GetAbsAngleDifference(vh->obj->pos_.GetH(), vh->obj->pos_.GetHRoadInDrivingDirection()) > M_PI_2)
			{
				speed_target_distance *= -1;
			}

			roadmanager::RoadProbeInfo data;

			// Speed - common speed target for these control modes
			vh->obj->pos_.GetProbeInfo(speed_target_distance, &data, roadmanager::Position::LOOKAHEADMODE_AT_ROAD_CENTER);

#ifdef _SCENARIO_VIEWER
			player->viewer_->SensorSetTargetPos(vh->gfx_model->speed_sensor_, data.road_lane_info.pos[0], data.road_lane_info.pos[1], data.road_lane_info.pos[2]);
#endif

			// Steering - Find out a steering target along ghost vehicle trail
			double s_out;
			int index_out;
			ObjectTrailState state;
			state.speed_ = 0;

			if (vh->obj->ghost_->trail_.FindPointAhead(vh->obj->trail_follow_index_, vh->obj->trail_follow_s_, steering_target_distance, state, index_out, s_out) != 0)
			{
				state.x_ = (float)vh->obj->pos_.GetX();
				state.y_ = (float)vh->obj->pos_.GetY();
				state.z_ = (float)vh->obj->pos_.GetX();
				state.speed_ = 0;
			}
			roadmanager::Position pos(state.x_, state.y_, 0, 0, 0, 0);
			vh->obj->pos_.GetProbeInfo(&pos, &data);
			vh->steering_target_heading = data.relative_h;

#ifdef _SCENARIO_VIEWER
			player->viewer_->SensorSetTargetPos(vh->gfx_model->steering_sensor_, data.road_lane_info.pos[0], data.road_lane_info.pos[1], data.road_lane_info.pos[2]);
#endif

			// Let steering target heading influence speed target - slowing down when turning
			vh->speed_target_speed = state.speed_ * (1 - vh->steering_target_heading / M_PI_2);

			vh->dyn_model->DrivingControlTarget(deltaTimeStep, vh->steering_target_heading, vh->speed_target_speed);
		}

		// Set OpenDRIVE position
		obj->pos_.XYZH2TrackPos(vh->dyn_model->posX_, vh->dyn_model->posY_, vh->dyn_model->posZ_, vh->dyn_model->heading_);

		// Fetch Z and Pitch from OpenDRIVE position
		vh->dyn_model->posZ_ = obj->pos_.GetZ();
		vh->dyn_model->pitch_ = obj->pos_.GetP();

		// Report updated state to scenario gateway
		std::string name = vh->obj->GetControl() == Object::Control::EXTERNAL ? "External_" : "Hybrid_external_" + i;
		player->scenarioGateway->reportObject(i, name, 0, 1, vh->obj->boundingbox_,player->scenarioEngine->getSimulationTime(),
			vh->dyn_model->speed_, vh->dyn_model->wheelAngle_, vh->dyn_model->wheelRotation_,
			vh->dyn_model->posX_, vh->dyn_model->posY_, vh->dyn_model->posZ_,
			vh->dyn_model->heading_, vh->dyn_model->pitch_, 0);

#if 0
		// Check distance
		roadmanager::Position distant_pos;
		distant_pos.SetLanePos(227, -1, 40, 0);
		roadmanager::PositionDiff diff;
		bool found = vh->obj->pos_.Delta(distant_pos, diff);
#endif
	}
}

void DeleteExternVehicles()
{
	for (size_t i = 0; i < extern_vehicle.size(); i++)
	{
		delete(extern_vehicle[i].dyn_model);
	}

	extern_vehicle.clear();
}

int main(int argc, char *argv[])
{
	ScenarioPlayer *player;
	__int64 time_stamp = 0;

	try
	{
		player = new ScenarioPlayer(argc, argv);
	}
	catch (const std::exception& e)
	{
		LOG(e.what());
		return -1;
	}

	SetupExternVehicles(player);

	//Vehicle Logger code
	VehicleLogger& egoLog = VehicleLogger::InstVehicleLog(player->scenarioEngine->getScenarioFilename(), player->scenarioEngine->entities.object_.size());
	//Counter for indexing each log entry
	int indexcounter = 0;

	while (!player->IsQuitRequested())
	{
		double dt;
		if (player->GetFixedTimestep() > 0.0)
		{
			dt = player->GetFixedTimestep();
		}
		else
		{
			dt = SE_getSimTimeStep(time_stamp, player->minStepSize, player->maxStepSize);
		}

		UpdateExternVehicles(dt, player);
		player->Frame(dt);



		//Flag for signalling end of data line, all vehicles reported
		bool isendline = false;

		//For each vehicle (entitity) stored in the ScenarioPlayer
		for (size_t i = 0; i < player->scenarioEngine->entities.object_.size(); i++)
		{
			//Create a pointer to the object at position i in the entities vector
			Object *obj = player->scenarioEngine->entities.object_[i];

			//Create a Position object for extracting this vehicles XYZ coordinates
			roadmanager::Position pos = obj->pos_;

			//Extract the String name of the object and store in a compatable const char array
			const char *name_ = &(*obj->name_.c_str());

			if ((i + 1) == player->scenarioEngine->entities.object_.size())
			{
				isendline = true;
			}

			//Log the extracted data of a normal INTERNAL ego vehicle, or additonal scenario vehicles
			egoLog.LogVehicleData(isendline, indexcounter, player->scenarioEngine->getSimulationTime(), name_,
				obj->id_, obj->speed_, obj->wheel_angle_, obj->wheel_rot_, pos.GetX(), pos.GetY(), pos.GetZ(),
				pos.GetS(), pos.GetT(), pos.GetH(), pos.GetHRelative(), pos.GetHRelativeDrivingDirection(),
				pos.GetP(), pos.GetCurvature());

			indexcounter += 1;
		}
	}

	DeleteExternVehicles();

	delete player;

	return 0;
}
