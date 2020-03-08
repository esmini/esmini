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
#include "IdealSensor.hpp"
#define _USE_MATH_DEFINES
#include <math.h>


static SE_Thread thread;
static SE_Mutex mutex;
using namespace scenarioengine;

static int COLOR_GREEN[3] = { 0x40, 0xA0, 0x50 };
static int COLOR_DARK_GRAY[3] = { 0x80, 0x80, 0x80 };
static int COLOR_GRAY[3] = { 0xBB, 0xBB, 0xBB };
static int COLOR_YELLOW[3] = { 0xC0, 0xB0, 0x70 };
static int COLOR_RED[3] = { 0xA0, 0x40, 0x40 };

static const double maxStepSize = 0.1;
static const double minStepSize = 0.01;
static vehicle::Vehicle *ego;
static double simTime = 0;
static int ego_id = -1;
static bool drawTrails = false;
static std::vector<ObjectSensor*> sensor;
static std::vector<viewer::SensorViewFrustum*> sensorFrustum;
static int camera_mode = 0;

#define TRAIL_DT 0.5
#define GHOST_HEADSTART 2.0

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
			return -1;
		}
		vh.gfx_model = scenarioViewer->cars_[i];

		if (vh.obj->GetControl() == Object::Control::EXTERNAL)
		{
			if (ego_id != -1)
			{
				LOG("Only one Ego vehicle (external control) supported. Already registered id %d. Ignoring this (%d)", ego_id, i);
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

			// Add sensor
			sensor.push_back(new ObjectSensor(&scenarioEngine->entities, scenarioEngine->entities.object_[i], 4, 0, 0.5, 5, 50, 50 * M_PI / 180.0, 10));
			sensorFrustum.push_back(new viewer::SensorViewFrustum(sensor.back(), vh.gfx_model->txNode_));
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

	if (SetupVehicles() != 0)
	{
		delete scenarioViewer;
		viewer_state = ViewerState::VIEWER_QUIT;
		return;
	}

	std::string info_text_str;
	parser->read("--info_text", info_text_str);
	if (info_text_str == "off")
	{
		scenarioViewer->ShowInfoText(false);
	}

	std::string trails_str;
	parser->read("--trails", trails_str);
	if (trails_str == "off")
	{
		scenarioViewer->ShowTrail(false);
	}

	std::string sensors_str;
	parser->read("--sensors", sensors_str);
	if (sensors_str == "on")
	{
		scenarioViewer->ShowObjectSensors(true);
	}

	std::string camera_str;
	parser->read("--camera_mode", camera_str);
	if (camera_str == "orbit")
	{
		camera_mode = osgGA::RubberbandManipulator::RB_MODE_ORBIT;
	} 
	else if (camera_str == "fixed")
	{
		camera_mode = osgGA::RubberbandManipulator::RB_MODE_FIXED;
	}
	else if (camera_str == "flex")
	{
		camera_mode = osgGA::RubberbandManipulator::RB_MODE_RUBBER_BAND;
	}
	else if (camera_str == "flex-orbit")
	{
		camera_mode = osgGA::RubberbandManipulator::RB_MODE_RUBBER_BAND_ORBIT;
	}
	else if (camera_str != "")
	{
		LOG("Unsupported camera mode: %s - using default (orbit)", camera_str.c_str());
	}

	scenarioViewer->SetCameraMode(camera_mode);

	double last_dot_time = 0;

	while (!scenarioViewer->osgViewer_->done())
	{
		mutex.Lock();

		bool add_dot = false;
		if (simTime - last_dot_time > TRAIL_DT)
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

			for (size_t i = 0; i < sensorFrustum.size(); i++)
			{
				sensorFrustum[i]->Update();
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
	arguments.getApplicationUsage()->addCommandLineOption("--info_text <mode>", "Show info text HUD (\"on\" (default), \"off\") (toggle during simulation by press 'i') ");
	arguments.getApplicationUsage()->addCommandLineOption("--trails <mode>", "Show trails (\"on\" (default), \"off\") (toggle during simulation by press 't') ");
	arguments.getApplicationUsage()->addCommandLineOption("--sensors <mode>", "Show sensor frustums (\"on\", \"off\" (default)) (toggle during simulation by press 'r') ");
	arguments.getApplicationUsage()->addCommandLineOption("--camera_mode <mode>", "Initial camera mode (\"orbit\" (default), \"fixed\", \"flex\", \"flex-orbit\") (toggle during simulation by press 'c') ");
	arguments.getApplicationUsage()->addCommandLineOption("--aa_mode <mode>", "Anti-alias mode=number of multisamples (subsamples, 0=off, 4=default)");

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
		scenarioEngine = new ScenarioEngine(oscFilename, GHOST_HEADSTART, control);
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
					// Set steering target point at a distance ahead proportional to the speed
					vh->speed_target_distance = MAX(7, vh->obj->speed_ * 2.0);
					vh->steering_target_distance = MAX(5, 0.25 * vh->speed_target_distance);

					// find out what direction is forward, according to vehicle relative road heading 
					if (GetAbsAngleDifference(vh->obj->pos_.GetH(), vh->obj->pos_.GetHRoadInDrivingDirection()) > M_PI_2)
					{
						vh->speed_target_distance *= -1;
					}

					roadmanager::SteeringTargetInfo data;

					// Speed - common speed target for these control modes
					vh->obj->pos_.GetSteeringTargetInfo(vh->speed_target_distance, &data, roadmanager::Position::LOOKAHEADMODE_AT_ROAD_CENTER);
					memcpy(vh->speed_target_pos, data.global_pos, sizeof(vh->speed_target_pos));

					// Steering - Find out a steering target along ghost vehicle trail
					double s_out;
					int index_out;
					ObjectTrailState state;
					state.speed_ = 0;

					if (vh->obj->ghost_->trail_.FindPointAhead(vh->obj->trail_follow_index_, vh->obj->trail_follow_s_, vh->steering_target_distance, state, index_out, s_out) != 0)
					{
						state.x_ = vh->obj->pos_.GetX();
						state.y_ = vh->obj->pos_.GetY();
						state.z_ = vh->obj->pos_.GetX();
						state.speed_ = 0;
					}
					roadmanager::Position pos(state.x_, state.y_, 0, 0, 0, 0);
					vh->obj->pos_.CalcSteeringTarget(&pos, &data);
					memcpy(vh->steering_target_pos, data.global_pos, sizeof(vh->steering_target_pos));
					vh->steering_target_heading = data.angle;
					
					// Let steering target heading influence speed target - slowing down when turning
					vh->speed_target_speed = state.speed_ * (1 - vh->steering_target_heading / M_PI_2);
				}

				if (scenarioEngine->getSimulationTime() >= 0 && (vh->obj->GetControl() == Object::Control::HYBRID_EXTERNAL || vh->obj->GetControl() == Object::Control::EXTERNAL))
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
			}

			for (size_t i = 0; i < sensor.size(); i++)
			{
				sensor[i]->Update();
				//LOG("sensor identified %d objects", sensor[i]->nObj_);
			}

			scenarioEngine->step(deltaSimTime);

			simTime += deltaSimTime;

#if 0
			LOG("%d %d %.2f (%.2f, %.2f) h: %.5f road_h %.5f h_relative_road %.5f",
			    scenarioEngine->entities.object_[0]->pos_.GetTrackId(),
			    scenarioEngine->entities.object_[0]->pos_.GetLaneId(),
				scenarioEngine->entities.object_[0]->pos_.GetS(),
				scenarioEngine->entities.object_[0]->pos_.GetX(),
				scenarioEngine->entities.object_[0]->pos_.GetY(),
			    scenarioEngine->entities.object_[0]->pos_.GetH(),
			    scenarioEngine->entities.object_[0]->pos_.GetHRoad(),
			    scenarioEngine->entities.object_[0]->pos_.GetHRelative());
#endif

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
