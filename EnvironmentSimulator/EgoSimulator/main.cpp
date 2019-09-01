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

typedef enum {
	VIEWER_NOT_STARTED,
	VIEWER_RUNNING,
	VIEWER_QUIT
} ViewerState;

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
	egoCar->vehicle->Update(deltaTimeStep, accelerate, steer);

	// Set OpenDRIVE position
	egoCar->pos->XYZH2TrackPos(egoCar->vehicle->posX_, egoCar->vehicle->posY_, egoCar->vehicle->posZ_, egoCar->vehicle->heading_);

	// Fetch Z and Pitch from OpenDRIVE position
	egoCar->vehicle->posZ_ = egoCar->pos->GetZ();
	egoCar->vehicle->pitch_ = egoCar->pos->GetP();
	
	// Find out pitch of road in driving direction
	if (egoCar->pos->GetHRelative() > M_PI / 2 && egoCar->pos->GetHRelative() < 3 * M_PI / 2)
	{
		egoCar->vehicle->pitch_ *= -1;
	}
}

static void viewer_thread(void *args)
{
	osg::ArgumentParser *parser = (osg::ArgumentParser*)args;
	int firstScenarioVehicle = scenarioEngine->GetExtControl() == true ? 1 : 0;

	// Create viewer
	scenarioViewer = new viewer::Viewer(roadmanager::Position::GetOpenDrive(), scenarioEngine->getSceneGraphFilename().c_str(), *parser, true);

	// Create Ego vehicle, 
	if (scenarioEngine->GetExtControl())
	{
		egoCar->graphics_model = scenarioViewer->AddCar(carModelsFiles_[0]);
		egoCar->vehicle = new vehicle::Vehicle(egoCar->pos->GetX(), egoCar->pos->GetY(), egoCar->pos->GetH(), egoCar->graphics_model->size_x);
	}

	//  Create cars for visualization
	for (size_t i = firstScenarioVehicle; i < scenarioEngine->entities.object_.size(); i++)
	{
		if (scenarioViewer->AddCar(scenarioEngine->entities.object_[i]->model_filepath_) == 0)
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
			car->SetRotation(pos.GetH(), pos.GetR(), pos.GetP());
		}

		// Visualize Ego car separatelly, if external control set
		if (scenarioEngine->GetExtControl())
		{
			// update 3D model transform
			egoCar->graphics_model->SetPosition(egoCar->vehicle->posX_, egoCar->vehicle->posY_, egoCar->vehicle->posZ_);
			egoCar->graphics_model->SetRotation(egoCar->vehicle->heading_, egoCar->vehicle->pitch_, 0.0);
			egoCar->graphics_model->UpdateWheels(egoCar->vehicle->wheelAngle_, egoCar->vehicle->wheelRotation_);

			// Update road and vehicle debug lines 
			scenarioViewer->UpdateVehicleLineAndPoints(egoCar->pos);

			// Visualize steering target point
			scenarioViewer->UpdateDriverModelPoint(egoCar->pos, MAX(5, egoCar->vehicle->speed_));
		}
		else if(scenarioEngine->entities.object_.size() > 0)
		{
			// Update road and vehicle debug lines 
			scenarioViewer->UpdateVehicleLineAndPoints(&scenarioEngine->entities.object_[0]->pos_);

			// Visualize steering target point
			scenarioViewer->UpdateDriverModelPoint(&scenarioEngine->entities.object_[0]->pos_, MAX(5, scenarioEngine->entities.object_[0]->speed_));
		}
		

		mutex.Unlock();

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

	double simTime = 0;

	// use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);	

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName());
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options]\n");
	arguments.getApplicationUsage()->addCommandLineOption("--osc <filename>", "OpenSCENARIO filename");
	arguments.getApplicationUsage()->addCommandLineOption("--ext_control <mode>", "Ego control (\"osc\", \"off\", \"on\")");
	arguments.getApplicationUsage()->addCommandLineOption("--record <file.dat>", "Record position data into a file for later replay");

	if (arguments.argc() < 2)
	{
		arguments.getApplicationUsage()->write(std::cout, 1, 120, true);
		return -1;
	}

	std::string oscFilename;
	arguments.read("--osc", oscFilename);

	std::string record_filename;
	arguments.read("--record", record_filename);

	std::string ext_control_str;
	arguments.read("--ext_control", ext_control_str);

	ExternalControlMode ext_control;
	if (ext_control_str == "osc" || ext_control_str == "") ext_control = ExternalControlMode::EXT_CONTROL_BY_OSC;
	else if(ext_control_str == "off") ext_control = ExternalControlMode::EXT_CONTROL_OFF;
	else if (ext_control_str == "on") ext_control = ExternalControlMode::EXT_CONTROL_ON;
	else
	{
		LOG("Unrecognized external control mode: %s", ext_control_str.c_str());
		ext_control = ExternalControlMode::EXT_CONTROL_BY_OSC;
	}


	// Create scenario engine
	try
	{
		scenarioEngine = new ScenarioEngine(oscFilename, simTime, ext_control);
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

	if (scenarioEngine->GetExtControl())
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

			if (scenarioEngine->GetExtControl())
			{
				// Update vehicle dynamics/driver model
				UpdateEgo(deltaSimTime, scenarioViewer);

				// Report updated Ego state to scenario gateway
				scenarioGateway->reportObject(ObjectState(EGO_ID, std::string("Ego"), 0, 1, simTime,
					egoCar->vehicle->posX_, egoCar->vehicle->posY_, egoCar->vehicle->posZ_,
					egoCar->vehicle->heading_, egoCar->vehicle->pitch_, 0,
					egoCar->vehicle->speed_));
			}

			scenarioEngine->step(deltaSimTime);

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
