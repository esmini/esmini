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

#include <iostream>

#include "vehicle.hpp"
#include "viewer.hpp"
#include "ScenarioEngine.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#define _USE_MATH_DEFINES
#include <math.h>


static SE_Thread thread;
static SE_Mutex mutex;
using namespace scenarioengine;

static int COLOR_GREEN[3] = { 0x40, 0xA0, 0x50 };
static int COLOR_DARK_GRAY[3] = { 0x80, 0x80, 0x80 };
static int COLOR_GRAY[3] = { 0xBB, 0xBB, 0xBB };
static int COLOR_YELLOW[3] = { 0x90, 0x80, 0x50 };
static int COLOR_RED[3] = { 0x90, 0x30, 0x30 };

static const double maxStepSize = 0.1;
static const double minStepSize = 0.01;
static vehicle::Vehicle *ego;
static double simTime = 0;
static int ego_id = -1;

roadmanager::OpenDrive *odrManager = 0;

typedef enum {
	VIEWER_NOT_STARTED,
	VIEWER_RUNNING,
	VIEWER_QUIT
} ViewerState;

std::string RequestControlMode2Str(ScenarioEngine::RequestControlMode mode)
{
	if (mode == ScenarioEngine::RequestControlMode::CONTROL_BY_OSC) return "by OSC";
	else if (mode == ScenarioEngine::RequestControlMode::CONTROL_INTERNAL) return "Internal";
	else if (mode == ScenarioEngine::RequestControlMode::CONTROL_EXTERNAL) return "External";
	else if (mode == ScenarioEngine::RequestControlMode::CONTROL_HYBRID) return "Hybrid";
	else return "Unknown";
}


static ViewerState viewer_state = ViewerState::VIEWER_NOT_STARTED; 
static ScenarioEngine *scenarioEngine;
static viewer::Viewer *scenarioViewer;

typedef struct
{
	scenarioengine::Object *obj;
	viewer::CarModel *gfx_model;
	vehicle::Vehicle *dyn_model;

	double speed_target_pos[3];
	double speed_target_distance;
	double speed_target_speed;

	double steering_target_pos[3];
	double steering_target_distance;
	double steering_target_heading;

	double trail_pos[3];
} ScenarioVehicle;

static std::vector<ScenarioVehicle> scenarioVehicle;

int SetupVehicles()
{
	for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
	{
		ScenarioVehicle vh;

		vh.obj = scenarioEngine->entities.object_[i];
		vh.steering_target_pos[0] = vh.obj->pos_.GetX();
		vh.steering_target_pos[1] = vh.obj->pos_.GetY();
		vh.steering_target_pos[2] = vh.obj->pos_.GetZ();
		vh.steering_target_distance = 0;
		vh.steering_target_heading = 0;

		vh.speed_target_pos[0] = vh.obj->pos_.GetX();
		vh.speed_target_pos[1] = vh.obj->pos_.GetY();
		vh.speed_target_pos[2] = vh.obj->pos_.GetZ();
		vh.speed_target_distance = 0;
		vh.speed_target_speed = 0;

		//  Create vehicles for visualization
		bool transparent;
		osg::Vec3 trail_color;
		if (vh.obj->control_ == Object::Control::HYBRID_GHOST)
		{
			transparent = true;
			trail_color[0] = ((double)COLOR_DARK_GRAY[0]) / 0xFF;
			trail_color[1] = ((double)COLOR_DARK_GRAY[1]) / 0xFF;
			trail_color[2] = ((double)COLOR_DARK_GRAY[2]) / 0xFF;
		}
		else if (vh.obj->control_ == Object::Control::HYBRID_EXTERNAL || vh.obj->control_ == Object::Control::EXTERNAL)
		{
			transparent = false;
			trail_color[0] = ((double)COLOR_YELLOW[0]) / 0xFF;
			trail_color[1] = ((double)COLOR_YELLOW[1]) / 0xFF;
			trail_color[2] = ((double)COLOR_YELLOW[2]) / 0xFF;
		}
		else
		{
			transparent = false;
			trail_color[0] = ((double)COLOR_RED[0]) / 0xFF;
			trail_color[1] = ((double)COLOR_RED[1]) / 0xFF;
			trail_color[2] = ((double)COLOR_RED[2]) / 0xFF;
		}
		if (scenarioViewer->AddCar(vh.obj->model_filepath_, transparent, trail_color) == 0)
		{
			delete scenarioViewer;
			viewer_state = ViewerState::VIEWER_QUIT;
			return -1;
		}

		vh.gfx_model = scenarioViewer->cars_[i];

		if (vh.obj->GetControl() == Object::Control::EXTERNAL)
		{
			if (ego_id != -1)
			{
				LOG("Only one Ego vehicle (external control) supported. Already registered id %d", ego_id);
				return -1;
			}
			else
			{
				LOG("Registering Ego id %d", i);
				ego_id = i;
			}
		}

		if (vh.obj->GetControl() == Object::Control::EXTERNAL ||
			vh.obj->GetControl() == Object::Control::HYBRID_EXTERNAL)
		{
			roadmanager::Position *pos = &vh.obj->pos_;
			vh.dyn_model = new vehicle::Vehicle(pos->GetX(), pos->GetY(), pos->GetH(), vh.gfx_model->size_x);
		}
		else
		{
			vh.dyn_model = 0;
		}

		if (vh.obj->GetControl() == Object::Control::HYBRID_EXTERNAL)
		{
			vh.gfx_model->steering_sensor_ = scenarioViewer->CreateSensor(COLOR_GREEN, true, true, 0.4, 3);
			if (odrManager->GetNumOfRoads() > 0)
			{
				vh.gfx_model->speed_sensor_ = scenarioViewer->CreateSensor(COLOR_GRAY, true, true, 0.4, 1);
				vh.gfx_model->trail_sensor_ = scenarioViewer->CreateSensor(COLOR_RED, true, false, 0.4, 3);
			}
		}
		else if (vh.obj->GetControl() == Object::Control::EXTERNAL)
		{
			if (odrManager->GetNumOfRoads() > 0)
			{
				scenarioViewer->CreateRoadSensors(vh.gfx_model);
			}
		}

		scenarioVehicle.push_back(vh);
	}

	return 0;
}

void UpdateExternalVehicle(int id, double deltaTimeStep, viewer::Viewer *viewer)
{
	ScenarioVehicle *vh = &scenarioVehicle[id];

	double speed_limit = vh->obj->pos_.GetSpeedLimit();
	if (speed_limit < SMALL_NUMBER)
	{
		// no speed limit defined, set something with regards to number of lanes
		if(vh->obj->pos_.GetRoadById(vh->obj->pos_.GetTrackId())->GetNumberOfDrivingLanesSide(vh->obj->pos_.GetS(), SIGN(vh->obj->pos_.GetLaneId())) > 1)
		{
			speed_limit = 110 / 3.6;
		}
		else
		{
			speed_limit = 60 / 3.6;
		}
	}
	vh->dyn_model->SetMaxSpeed(speed_limit);
	
	if (vh->obj->GetControl() == Object::Control::EXTERNAL)
	{
		vehicle::THROTTLE accelerate = vehicle::THROTTLE_NONE;
		if (viewer->getKeyUp())
		{
			accelerate = vehicle::THROTTLE_ACCELERATE;
		}
		else if (viewer->getKeyDown())
		{
			accelerate = vehicle::THROTTLE_BRAKE;
		}

		vehicle::STEERING steer = vehicle::STEERING_NONE;
		if (viewer->getKeyLeft())
		{
			steer = vehicle::STEERING_LEFT;
		}
		else if (viewer->getKeyRight())
		{
			steer = vehicle::STEERING_RIGHT;
		}

		// Update vehicle motion
		vh->dyn_model->DrivingControlBinary(deltaTimeStep, accelerate, steer);
	}
	else if (vh->obj->GetControl() == Object::Control::HYBRID_EXTERNAL)
	{
		vh->dyn_model->DrivingControlTarget(deltaTimeStep, vh->steering_target_heading, vh->speed_target_speed);
	}
	
	// Set OpenDRIVE position
	vh->obj->pos_.XYZH2TrackPos(vh->dyn_model->posX_, vh->dyn_model->posY_, vh->dyn_model->posZ_,
		vh->dyn_model->heading_);

	// Fetch Z and Pitch from OpenDRIVE position
	vh->dyn_model->posZ_ = vh->obj->pos_.GetZ();
	vh->dyn_model->pitch_ = vh->obj->pos_.GetP();
}

static void viewer_thread(void *args)
{
	osg::ArgumentParser *parser = (osg::ArgumentParser*)args;

	// Create viewer
	scenarioViewer = new viewer::Viewer(
		roadmanager::Position::GetOpenDrive(), 
		scenarioEngine->getSceneGraphFilename().c_str(), 
		scenarioEngine->getScenarioFilename().c_str(), 
		*parser, 
		scenarioEngine->entities.object_[0]->GetControl() == Object::Control::INTERNAL ? false : true);

	SetupVehicles();

	std::string info_text_str;
	parser->read("--info_text", info_text_str);
	if (info_text_str == "off")
	{
		scenarioViewer->ShowInfoText(false);
	}

	double last_dot_time = 0;

	while (!scenarioViewer->osgViewer_->done())
	{
		mutex.Lock();

		bool add_dot = false;
		if (simTime - last_dot_time > 0.2)
		{
			add_dot = true;
			last_dot_time = simTime;
		}

		// Visualize scenario cars
		for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
		{
			ScenarioVehicle *vh = &scenarioVehicle[i];

			vh->gfx_model->SetPosition(vh->obj->pos_.GetX(), vh->obj->pos_.GetY(), vh->obj->pos_.GetZ());
			vh->gfx_model->SetRotation(vh->obj->pos_.GetH(), vh->obj->pos_.GetP(), vh->obj->pos_.GetR());

			if (vh->obj->GetControl() == Object::Control::EXTERNAL ||
				vh->obj->GetControl() == Object::Control::HYBRID_EXTERNAL)
			{

				vh->gfx_model->UpdateWheels(vh->dyn_model->wheelAngle_, vh->dyn_model->wheelRotation_);

				if (vh->obj->GetControl() == Object::Control::HYBRID_EXTERNAL)
				{
					scenarioViewer->UpdateSensor(vh->gfx_model->speed_sensor_, &vh->obj->pos_, vh->speed_target_pos);
					scenarioViewer->UpdateSensor(vh->gfx_model->steering_sensor_, &vh->obj->pos_, vh->steering_target_pos);

					double pos[3];
					if (vh->obj->ghost_->trail_.FindClosestPoint(vh->obj->pos_.GetX(), vh->obj->pos_.GetY(), pos[0], pos[1],
						vh->obj->trail_follow_s_, vh->obj->trail_follow_index_, vh->obj->trail_follow_index_) == 0)
					{
						pos[2] = vh->obj->pos_.GetZ();
					}
					else
					{
						// Failed find point along trail, copy entity position
						pos[0] = vh->obj->pos_.GetX();
						pos[1] = vh->obj->pos_.GetY();
						pos[2] = vh->obj->pos_.GetZ();
					}
					scenarioViewer->UpdateSensor(vh->gfx_model->trail_sensor_, &vh->obj->pos_, pos);
				}
				else if (odrManager->GetNumOfRoads() > 0 && vh->obj->GetControl() == Object::Control::EXTERNAL)
				{
					scenarioViewer->UpdateRoadSensors(vh->gfx_model->road_sensor_, vh->gfx_model->lane_sensor_, &vh->obj->pos_);
				}
			}
			
			if (add_dot)
			{
				vh->gfx_model->trail_->AddDot(simTime, vh->obj->pos_.GetX(), vh->obj->pos_.GetY(), vh->obj->pos_.GetZ(), vh->obj->pos_.GetH());
			}
		}		

		mutex.Unlock();

		// Update info text 
		static char str_buf[128];
		snprintf(str_buf, sizeof(str_buf), "%.2fs %.2fkm/h", scenarioEngine->getSimulationTime(), 
			3.6 * scenarioVehicle[scenarioViewer->currentCarInFocus_].obj->speed_);
		scenarioViewer->SetInfoText(str_buf);

		scenarioViewer->osgViewer_->frame();

		viewer_state = ViewerState::VIEWER_RUNNING;
	}

	delete scenarioViewer;

	viewer_state = ViewerState::VIEWER_QUIT;
}

void log_callback(const char *str)
{
	printf("%s\n", str);
}

int main(int argc, char** argv)
{
	double deltaSimTime;

	// Use logger callback
	Logger::Inst().SetCallback(log_callback);

	ScenarioGateway *scenarioGateway;
	roadmanager::Position *lane_pos = new roadmanager::Position();
	roadmanager::Position *track_pos = new roadmanager::Position();

	// use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);	

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName());
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options]\n");
	arguments.getApplicationUsage()->addCommandLineOption("--osc <filename>", "OpenSCENARIO filename");
	arguments.getApplicationUsage()->addCommandLineOption("--control <mode>", "Ego control (\"osc\", \"internal\", \"external\", \"hybrid\"");
	arguments.getApplicationUsage()->addCommandLineOption("--record <file.dat>", "Record position data into a file for later replay");
	arguments.getApplicationUsage()->addCommandLineOption("--info_text <mode>", "Show info text HUD (\"on\" (default), \"off\") (toggle during simulation by press 't') ");

	if (arguments.argc() < 2)
	{
		arguments.getApplicationUsage()->write(std::cout, 1, 120, true);
		return -1;
	}

	std::string oscFilename;
	arguments.read("--osc", oscFilename);

	std::string record_filename;
	arguments.read("--record", record_filename);

	std::string control_str;
	arguments.read("--control", control_str);

	ScenarioEngine::RequestControlMode control;
	if (control_str == "osc" || control_str == "") control = ScenarioEngine::RequestControlMode::CONTROL_BY_OSC;
	else if(control_str == "internal") control = ScenarioEngine::RequestControlMode::CONTROL_INTERNAL;
	else if (control_str == "external") control = ScenarioEngine::RequestControlMode::CONTROL_EXTERNAL;
	else if (control_str == "hybrid") control = ScenarioEngine::RequestControlMode::CONTROL_HYBRID;
	else
	{
		LOG("Unrecognized external control mode: %s", control_str.c_str());
		control = ScenarioEngine::RequestControlMode::CONTROL_BY_OSC;
	}

	// Create scenario engine
	try
	{
		scenarioEngine = new ScenarioEngine(oscFilename, 2.0, control);
		odrManager = scenarioEngine->getRoadManager();
	}
	catch (const std::exception& e)
	{
		LOG(e.what());
		return -1;
	}

	// Fetch ScenarioGateway
	scenarioGateway = scenarioEngine->getScenarioGateway();

	// Create a data file for later replay?
	if (!record_filename.empty())
	{
		LOG("Recording data to file %s", record_filename);
		scenarioGateway->RecordToFile(record_filename, scenarioEngine->getOdrFilename(), scenarioEngine->getSceneGraphFilename());
	}

	// Step scenario engine - zero time - just to reach init state
	// Report all vehicles initially - to communicate initial position for external vehicles as well
	scenarioEngine->step(0.0, true);

	// Launch viewer in a separate thread
	thread.Start(viewer_thread, &arguments);

	// Wait for the viewer to launch
	while (viewer_state == ViewerState::VIEWER_NOT_STARTED) SE_sleep(100);

	try
	{
		__int64 now, lastTimeStamp = 0;

		while (viewer_state == ViewerState::VIEWER_RUNNING)
		{
			// Get milliseconds since Jan 1 1970
			now = SE_getSystemTime();
			deltaSimTime = (now - lastTimeStamp) / 1000.0;  // step size in seconds
			lastTimeStamp = now;
			double adjust = 0;

			if (deltaSimTime > maxStepSize) // limit step size
			{
				adjust = -(deltaSimTime - maxStepSize);
			}
			else if (deltaSimTime < minStepSize)  // avoid CPU rush, sleep for a while
			{
				adjust = minStepSize - deltaSimTime;
				SE_sleep(adjust * 1000);
				lastTimeStamp += adjust * 1000;
			}

			deltaSimTime += adjust;

			// ScenarioEngine
			mutex.Lock();

			for (size_t i = 0; i < scenarioVehicle.size(); i++)
			{
				ScenarioVehicle *vh = &scenarioVehicle[i];
				if (vh->obj->GetControl() == Object::Control::HYBRID_EXTERNAL)
				{
					ObjectTrailState *trailState = 0;

					// Set steering target point at a distance ahead proportional to the speed
					trailState = vh->obj->ghost_->trail_.GetStateByIndex(vh->obj->trail_follow_index_);
					if (trailState != 0)
					{
						vh->speed_target_distance = MAX(7, trailState->speed_ * 2.0);
					}
					else
					{
						vh->speed_target_distance = 7;
					}
					vh->steering_target_distance = 0.5 * vh->speed_target_distance;

					// find out what direction is forward, according to vehicle relative road heading 
					if (GetAbsAngleDifference(vh->obj->pos_.GetH(), vh->obj->pos_.GetHRoadInDrivingDirection()) > M_PI_2)
					{
						vh->steering_target_distance *= -1;
						vh->speed_target_distance *= -1;
					}

					roadmanager::SteeringTargetInfo data;

					// Speed - common speed target for these control modes
					vh->obj->pos_.GetSteeringTargetInfo(vh->speed_target_distance, &data, true);
					memcpy(vh->speed_target_pos, data.global_pos, sizeof(vh->speed_target_pos));

					// Steering - Find out a steering target along ghost vehicle trail
					double x, y, z, s_out, speed;
					int index_out;

					if (vh->obj->ghost_->trail_.FindPointAhead(vh->obj->trail_follow_index_, vh->obj->trail_follow_s_, vh->steering_target_distance,
						x, y, z, speed, index_out, s_out) != 0)
					{
						x = vh->obj->pos_.GetX();
						y = vh->obj->pos_.GetY();
						z = vh->obj->pos_.GetX();
						speed = 0;
					}
					roadmanager::Position pos(x, y, 0, 0, 0, 0);
					vh->obj->pos_.CalcSteeringTarget(&pos, &data);
					memcpy(vh->steering_target_pos, data.global_pos, sizeof(vh->steering_target_pos));
					vh->steering_target_heading = data.angle;
					
					// Let steering target heading influence speed target - slowing down when turning
					vh->speed_target_speed = speed * (1 - vh->steering_target_heading / M_PI_2);
					
					//LOG("ahead [%d]: speed: %.2f ts %.2f dist: %.2f (%.2f, %.2f)", vh->obj->trail_follow_index_, trailState ? trailState->speed_ : -1, vh->speed_target_speed, vh->steering_target_distance, x, y);
				}

				if (vh->obj->GetControl() == Object::Control::HYBRID_EXTERNAL || vh->obj->GetControl() == Object::Control::EXTERNAL)
				{
					// Update vehicle dynamics/driver model
					UpdateExternalVehicle(i, deltaSimTime, scenarioViewer);

					// Report updated Ego state to scenario gateway
					std::string name = vh->obj->GetControl() == Object::Control::EXTERNAL ? "External_" : "Hybrid_external_" + i;
					scenarioGateway->reportObject(ObjectState(i, name, 0, 1, simTime,
						vh->dyn_model->posX_, vh->dyn_model->posY_, vh->dyn_model->posZ_,
						vh->dyn_model->heading_, vh->dyn_model->pitch_, 0,
						vh->dyn_model->speed_, vh->dyn_model->wheelAngle_, vh->dyn_model->wheelRotation_));
				}


				// Save position to object trail
				Object *obj = scenarioEngine->entities.object_[i];
				obj->trail_.AddState(simTime, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ(), obj->speed_);
			}

			scenarioEngine->step(deltaSimTime);

			simTime += deltaSimTime;

			//LOG("%d %d %.2f h: %.5f rh %.5f rh %.5f",
			    //      scenarioEngine->entities.object_[0]->pos_.GetTrackId(),
			    //      scenarioEngine->entities.object_[0]->pos_.GetLaneId(),
			    //      scenarioEngine->entities.object_[0]->pos_.GetS(),
			    //      scenarioEngine->entities.object_[0]->pos_.GetH(),
			    //      scenarioEngine->entities.object_[0]->pos_.GetHRoad(),
			    //      scenarioEngine->entities.object_[0]->pos_.GetHRelative());

			mutex.Unlock();
		}
	}
	catch (std::logic_error &e)
	{
		printf("%s\n", e.what());
		return 2;
	}
	catch (std::runtime_error &e)
	{
		printf("%s\n", e.what());
		return 3;
	}

	delete(scenarioEngine);
	for (size_t i = 0; i < scenarioVehicle.size(); i++)
	{
		if (scenarioVehicle[i].dyn_model)
		{
			delete (scenarioVehicle[i].dyn_model);
		}
	}
	scenarioVehicle.clear();
	delete track_pos;
	delete lane_pos;

	return 0;
}
