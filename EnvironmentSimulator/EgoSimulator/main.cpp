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

#define EGO_ID 0	// need to match appearing order in the OpenSCENARIO file
#define SIGN(x) (x < 0 ? -1 : 1)
#define MAX(x, y) (y > x ? y : x)
#define MIN(x, y) (y < x ? y : x)

static const double maxStepSize = 0.1;
static const double minStepSize = 0.01;
static vehicle::Vehicle *ego;
static double simTime = 0;
static double steering_heading;
static double steering_target_pos[3];
static double steering_target_hwt;
static double steer_tgt_distance;

static roadmanager::Position *ego_pos;

typedef enum {
	VIEWER_NOT_STARTED,
	VIEWER_RUNNING,
	VIEWER_QUIT
} ViewerState;

typedef enum
{
	CONTROL_BY_OSC,
	CONTROL_INTERNAL,
	CONTROL_EXTERNAL,
	CONTROL_HYBRID
} RequestControlMode;

std::string RequestControlMode2Str(RequestControlMode mode)
{
	if (mode == RequestControlMode::CONTROL_BY_OSC) return "by OSC";
	else if (mode == RequestControlMode::CONTROL_INTERNAL) return "Internal";
	else if (mode == RequestControlMode::CONTROL_EXTERNAL) return "External";
	else if (mode == RequestControlMode::CONTROL_HYBRID) return "Hybrid";
	else return "Unknown";
}


static ViewerState viewer_state = ViewerState::VIEWER_NOT_STARTED; 
static ScenarioEngine *scenarioEngine;
static viewer::Viewer *scenarioViewer;

typedef struct
{
	int road_id_init;
	int lane_id_init;
	roadmanager::Position *pos;
	viewer::CarModel *graphics_model;
	vehicle::Vehicle *vehicle;
} EgoCar;

static EgoCar *egoCar;

typedef struct
{
	int id;
	viewer::CarModel *carModel;
	roadmanager::Position pos;
} ScenarioCar;

static std::vector<ScenarioCar> scenarioCar;

int SetupEgo(roadmanager::OpenDrive *odrManager, roadmanager::Position init_pos)
{
	egoCar = new EgoCar;
	egoCar->road_id_init = init_pos.GetTrackId();
	egoCar->lane_id_init = init_pos.GetLaneId();
	egoCar->pos = new roadmanager::Position(init_pos);

	return 0;
}

void UpdateEgo(double deltaTimeStep, viewer::Viewer *viewer)
{
	double speed_limit = egoCar->pos->GetSpeedLimit();
	if (speed_limit < SMALL_NUMBER)
	{
		// no speed limit defined, set something with regards to number of lanes
		if(egoCar->pos->GetRoadById(egoCar->pos->GetTrackId())->GetNumberOfDrivingLanesSide(egoCar->pos->GetS(), SIGN(egoCar->pos->GetLaneId())) > 1)
		{
			speed_limit = 110 / 3.6;
		}
		else
		{
			speed_limit = 60 / 3.6;
		}
	}
	egoCar->vehicle->SetMaxSpeed(speed_limit);
	
	if (scenarioEngine->GetControl() == Object::Control::EXTERNAL)
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
		egoCar->vehicle->DrivingControlBinary(deltaTimeStep, accelerate, steer);
	}
	else if (scenarioEngine->GetControl() == Object::Control::HYBRID_EXTERNAL)
	{
		egoCar->vehicle->DrivingControlTarget(deltaTimeStep, steering_heading, steering_target_hwt);
	}
	
	// Set OpenDRIVE position
	egoCar->pos->XYZH2TrackPos(egoCar->vehicle->posX_, egoCar->vehicle->posY_, egoCar->vehicle->posZ_, egoCar->vehicle->heading_);

	// Fetch Z and Pitch from OpenDRIVE position
	egoCar->vehicle->posZ_ = egoCar->pos->GetZ();
	egoCar->vehicle->pitch_ = egoCar->pos->GetP();

}

static void viewer_thread(void *args)
{
	osg::ArgumentParser *parser = (osg::ArgumentParser*)args;
	int firstScenarioVehicle = 
		(scenarioEngine->GetControl() == Object::Control::EXTERNAL || 
		 scenarioEngine->GetControl() == Object::Control::HYBRID_EXTERNAL) ? 1 : 0;

	// Create viewer
	scenarioViewer = new viewer::Viewer(
		roadmanager::Position::GetOpenDrive(), 
		scenarioEngine->getSceneGraphFilename().c_str(), 
		scenarioEngine->getScenarioFilename().c_str(), 
		*parser, 
		true);

	std::string info_text_str;
	parser->read("--info_text", info_text_str);
	if (info_text_str == "off")
	{
		scenarioViewer->ShowInfoText(false);
	}

	// Create Ego vehicle, 
	if (scenarioEngine->GetControl() == Object::Control::EXTERNAL ||
		scenarioEngine->GetControl() == Object::Control::HYBRID_EXTERNAL)
	{
		if ((egoCar->graphics_model = scenarioViewer->AddCar(scenarioEngine->entities.object_[0]->model_filepath_)) == 0)
		{
			delete scenarioViewer;
			viewer_state = ViewerState::VIEWER_QUIT;
			return;
		}
		egoCar->vehicle = new vehicle::Vehicle(egoCar->pos->GetX(), egoCar->pos->GetY(), egoCar->pos->GetH(), egoCar->graphics_model->size_x);
	}

	//  Create cars for visualization
	for (size_t i = firstScenarioVehicle; i < scenarioEngine->entities.object_.size(); i++)
	{
		bool transparent = scenarioEngine->entities.object_[i]->control_ == Object::Control::HYBRID_GHOST ? true : false;

		if (scenarioViewer->AddCar(scenarioEngine->entities.object_[i]->model_filepath_, transparent) == 0)
		{
			delete scenarioViewer;
			viewer_state = ViewerState::VIEWER_QUIT;
			return;
		}
	}

	while (!scenarioViewer->osgViewer_->done())
	{
		mutex.Lock();

		// Visualize scenario cars
		for (size_t i = firstScenarioVehicle; i < scenarioEngine->entities.object_.size(); i++)
		{
			viewer::CarModel *car = scenarioViewer->cars_[i];
			roadmanager::Position pos = scenarioEngine->entities.object_[i]->pos_;

			car->SetPosition(pos.GetX(), pos.GetY(), pos.GetZ());
			car->SetRotation(pos.GetH(), pos.GetP(), pos.GetR());
		}


		if (ego_pos && ego_pos->GetOpenDrive()->GetNumOfRoads() > 0)
		{
			// Update road and vehicle debug lines 
			scenarioViewer->UpdateVehicleLineAndPoints(ego_pos);
			scenarioViewer->UpdateDriverModelPoint(ego_pos, steering_target_pos);
		}

		if (scenarioEngine->GetControl() == Object::Control::EXTERNAL ||
			scenarioEngine->GetControl() == Object::Control::HYBRID_EXTERNAL)
		{
			// Visualize Ego car separatelly, if external control set
			// update 3D model transform
			egoCar->graphics_model->SetPosition(egoCar->vehicle->posX_, egoCar->vehicle->posY_, egoCar->vehicle->posZ_);
			egoCar->graphics_model->SetRotation(egoCar->vehicle->heading_, egoCar->vehicle->pitch_, 0.0);
			egoCar->graphics_model->UpdateWheels(egoCar->vehicle->wheelAngle_, egoCar->vehicle->wheelRotation_);
		}

		mutex.Unlock();

		// Update info text 
		static char str_buf[128];
		snprintf(str_buf, sizeof(str_buf), "%.2fs %.2fkm/h", scenarioEngine->getSimulationTime(), 
			3.6 * scenarioEngine->entities.object_[scenarioViewer->currentCarInFocus_]->speed_);
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
	roadmanager::OpenDrive *odrManager;
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

	RequestControlMode control;
	if (control_str == "osc" || control_str == "") control = RequestControlMode::CONTROL_BY_OSC;
	else if(control_str == "internal") control = RequestControlMode::CONTROL_INTERNAL;
	else if (control_str == "external") control = RequestControlMode::CONTROL_EXTERNAL;
	else if (control_str == "hybrid") control = RequestControlMode::CONTROL_HYBRID;
	else
	{
		LOG("Unrecognized external control mode: %s", control_str.c_str());
		control = RequestControlMode::CONTROL_BY_OSC;
	}

	// Create scenario engine
	try
	{
		scenarioEngine = new ScenarioEngine(oscFilename);
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

	if (scenarioEngine->GetControl() == Object::Control::EXTERNAL ||
		scenarioEngine->GetControl() == Object::Control::HYBRID_EXTERNAL)
	{
		// Setup Ego with initial position from the gateway
		SetupEgo(odrManager, scenarioGateway->getObjectStatePtrByIdx(0)->state_.pos);
	}

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


			// Set steering target point at a distance ahead proportional to the speed
			roadmanager::Position *pos = 0;

			if (scenarioEngine->GetControl() == Object::Control::EXTERNAL ||
				scenarioEngine->GetControl() == Object::Control::HYBRID_EXTERNAL)
			{
				ego_pos = egoCar->pos;
				steer_tgt_distance = MAX(5, egoCar->vehicle->speed_);
			}
			else if (scenarioEngine->entities.object_.size() > 0)
			{
				ego_pos = &scenarioEngine->entities.object_[0]->pos_;
				steer_tgt_distance = MAX(5, scenarioEngine->entities.object_[0]->speed_);
			}

			// find out what direction is forward, according to vehicle relative road heading 
			if (GetAbsAngleDifference(ego_pos->GetH(), ego_pos->GetHRoadInDrivingDirection()) > M_PI_2)
			{
				steer_tgt_distance *= -1;
			}

			// Find and visualize steering target point	
			roadmanager::SteeringTargetInfo data;
			if (scenarioEngine->GetControl() == Object::Control::EXTERNAL)
			{
				ego_pos->GetSteeringTargetInfo(steer_tgt_distance, &data, false);
				steering_target_hwt = 0;
				steering_heading = data.angle;
			}
			else if (scenarioEngine->GetControl() == Object::Control::HYBRID_EXTERNAL)
			{
				ego_pos->GetSteeringTargetInfo(&scenarioEngine->entities.object_[0]->ghost_->pos_, &data);
				if (fabs(egoCar->vehicle->speed_) > SMALL_NUMBER)
				{
					steering_target_hwt = GetLengthOfVector3D(data.local_pos[0], data.local_pos[1], data.local_pos[2]) / egoCar->vehicle->speed_;
				}
				else
				{
					steering_target_hwt = LARGE_NUMBER;
				}
				steering_heading = data.angle;
			}
			memcpy_s(steering_target_pos, sizeof(steering_target_pos), data.global_pos, sizeof(data.global_pos));

			if (scenarioEngine->GetControl() == Object::Control::EXTERNAL ||
				scenarioEngine->GetControl() == Object::Control::HYBRID_EXTERNAL)
			{
				// Update vehicle dynamics/driver model
				UpdateEgo(deltaSimTime, scenarioViewer);

				// Report updated Ego state to scenario gateway
				scenarioGateway->reportObject(ObjectState(EGO_ID, std::string("Ego"), 0, 1, simTime,
					egoCar->vehicle->posX_, egoCar->vehicle->posY_, egoCar->vehicle->posZ_,
					egoCar->vehicle->heading_, egoCar->vehicle->pitch_, 0,
					egoCar->vehicle->speed_, 0.0));
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
	delete(egoCar);
	delete track_pos;
	delete lane_pos;

	return 0;
}
