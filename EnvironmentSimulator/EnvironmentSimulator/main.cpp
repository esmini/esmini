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
  * This application is an example of how to use the ScenarioEngine and Viewer modules to play and view scenarios.
  *
  * Instead of linking with the ScenarioEngineDLL it links all needed library functions statically into the 
  * all-inclusive executable.
  *
  * The Viewer is driven in a separate thread to enable camera movement even if simulation is paused.
  *
  * A simpler solution is to move the Viewer handling to the main loop, i.e:
  *   Creation of Viewer and Vehicles visual models should be done after scenario engine initialization
  *   Vehicle position update and Viewer->frame() call part of the main loop.
  */

#include <iostream>
#include <string>
#include <random>

#include "ScenarioEngine.hpp"
#include "viewer.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "Server.hpp"

using namespace scenarioengine;

enum ViewerState { VIEWER_NOT_STARTED, VIEWER_RUNNING, VIEWER_DONE, VIEWER_FAILED };

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

static const double maxStepSize = 0.1;
static const double minStepSize = 0.01;
static const bool freerun = true;
static int viewer_state = VIEWER_NOT_STARTED;
static double simTime = 0;
static int COLOR_DARK_GRAY[3] = { 0x80, 0x80, 0x80 };
static int COLOR_YELLOW[3] = { 0x85, 0x75, 0x40 };
static int COLOR_RED[3] = { 0x90, 0x30, 0x30 };


static ScenarioEngine *scenarioEngine;

static SE_Thread thread;
static SE_Mutex mutex;

static viewer::Viewer *viewer_;
static bool quit_request = false;

#define TRAIL_DT 0.5

void scenario_frame()
{
	static __int64 lastTimeStamp = 0;
	double deltaSimTime;
	__int64 now;

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

	// Time operations
	simTime = simTime + deltaSimTime;

	// ScenarioEngine

	mutex.Lock();

	scenarioEngine->step(deltaSimTime);
	//LOG("%d %d %.2f h: %.5f road_h %.5f h_relative_road %.5f",
	//    scenarioEngine->entities.object_[0]->pos_.GetTrackId(),
	//    scenarioEngine->entities.object_[0]->pos_.GetLaneId(),
	//    scenarioEngine->entities.object_[0]->pos_.GetS(),
	//    scenarioEngine->entities.object_[0]->pos_.GetH(),
	//    scenarioEngine->entities.object_[0]->pos_.GetHRoad(),
	//    scenarioEngine->entities.object_[0]->pos_.GetHRelative());

	mutex.Unlock();
}

void viewer_frame()
{
	static double last_dot_time = 0;

	bool add_dot = false;
	if (simTime - last_dot_time > TRAIL_DT)
	{
		add_dot = true;
		last_dot_time = simTime;
	}

	mutex.Lock();

	// Visualize cars
	for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
	{
		viewer::CarModel *car = viewer_->cars_[i];
		Object *obj = scenarioEngine->entities.object_[i];
		roadmanager::Position pos = scenarioEngine->entities.object_[i]->pos_;

		car->SetPosition(pos.GetX(), pos.GetY(), pos.GetZ());
		car->SetRotation(pos.GetH(), pos.GetP(), pos.GetR());
		car->UpdateWheels(obj->wheel_angle_, obj->wheel_rot_);

		if (add_dot)
		{
			car->trail_->AddDot(simTime, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ(), obj->pos_.GetH());
		}
	}

	// Update info text 
	static char str_buf[128];
	snprintf(str_buf, sizeof(str_buf), "%.2fs %.2fkm/h", simTime, 3.6 * scenarioEngine->entities.object_[viewer_->currentCarInFocus_]->speed_);
	viewer_->SetInfoText(str_buf);

	mutex.Unlock();

	viewer_->osgViewer_->frame();
}

void scenario_thread(void *args)
{
	while (!quit_request)
	{
		scenario_frame();
	}
}

void log_callback(const char *str)
{
	printf("%s\n", str);
}

int main(int argc, char *argv[])
{	
	// use an ArgumentParser object to manage the program arguments.
	osg::ArgumentParser arguments(&argc, argv);

	arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName());
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options]\n");
	arguments.getApplicationUsage()->addCommandLineOption("--osc <filename>", "OpenSCENARIO filename");
	arguments.getApplicationUsage()->addCommandLineOption("--control <mode>", "Ego control (\"osc\", \"internal\", \"external\", \"hybrid\"");
	arguments.getApplicationUsage()->addCommandLineOption("--info_text <mode>", "Show info text HUD (\"on\" (default), \"off\") (toggle during simulation by press 't') ");
	arguments.getApplicationUsage()->addCommandLineOption("--trails <mode>", "Show trails (\"on\" (default), \"off\") (toggle during simulation by press 't') ");

	if (arguments.argc() < 2)
	{
		arguments.getApplicationUsage()->write(std::cout, 1, 80, true);
		return -1;
	}

	std::string oscFilename;
	arguments.read("--osc", oscFilename);

	std::string control_str;
	arguments.read("--control", control_str);

	RequestControlMode control;
	if (control_str == "osc" || control_str == "") control = RequestControlMode::CONTROL_BY_OSC;
	else if (control_str == "internal") control = RequestControlMode::CONTROL_INTERNAL;
	else if (control_str == "external") control = RequestControlMode::CONTROL_EXTERNAL;
	else if (control_str == "hybrid") control = RequestControlMode::CONTROL_HYBRID;
	else
	{
		LOG("Unrecognized external control mode: %s", control_str.c_str());
		control = RequestControlMode::CONTROL_BY_OSC;
	}

	std::string record_filename;
	arguments.read("--record", record_filename);

	// Use logger callback
	Logger::Inst().SetCallback(log_callback);

	// Create scenario engine
	try 
	{ 
		scenarioEngine = new ScenarioEngine(oscFilename, 1.0, (ScenarioEngine::RequestControlMode)control);
	}
	catch (std::logic_error &e)
	{
		printf("%s\n", e.what());
		return -1;
	}

	// ScenarioGateway
	ScenarioGateway *scenarioGateway = scenarioEngine->getScenarioGateway();

	// Create a data file for later replay?
	if (!record_filename.empty())
	{
		LOG("Recording data to file %s", record_filename.c_str());
		scenarioGateway->RecordToFile(record_filename, scenarioEngine->getOdrFilename(), scenarioEngine->getSceneGraphFilename());
	}

	// Step scenario engine - zero time - just to reach init state	
	// Report all vehicles initially - to communicate initial position for external vehicles as well
	scenarioEngine->step(0.0, true);

	// Launch viewer in a separate thread
	osg::ArgumentParser *parser = (osg::ArgumentParser*)&arguments;

	// Create viewer
	viewer_ = new viewer::Viewer(
		roadmanager::Position::GetOpenDrive(),
		scenarioEngine->getSceneGraphFilename().c_str(),
		scenarioEngine->getScenarioFilename().c_str(),
		*parser);

	std::string info_text_str;
	parser->read("--info_text", info_text_str);
	if (info_text_str == "off")
	{
		viewer_->ShowInfoText(false);
	}

	std::string trail_str;
	parser->read("--trails", trail_str);
	if (trail_str == "off")
	{
		viewer_->ShowTrail(false);
	}

	//  Create cars for visualization
	for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
	{
		//  Create vehicles for visualization
		bool transparent;
		osg::Vec3 trail_color;
		Object *obj = scenarioEngine->entities.object_[i];
		if (obj->control_ == Object::Control::HYBRID_GHOST)
		{
			transparent = true;
			trail_color[0] = ((double)COLOR_DARK_GRAY[0]) / 0xFF;
			trail_color[1] = ((double)COLOR_DARK_GRAY[1]) / 0xFF;
			trail_color[2] = ((double)COLOR_DARK_GRAY[2]) / 0xFF;
		}
		else if (obj->control_ == Object::Control::HYBRID_EXTERNAL || obj->control_ == Object::Control::EXTERNAL)
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

		if (viewer_->AddCar(obj->model_filepath_, transparent, trail_color) == 0)
		{
			delete viewer_;
			viewer_ = 0;
			return -1;
		}
	}

	if(scenarioEngine->entities.object_[0]->GetControl() == Object::Control::EXTERNAL ||
		scenarioEngine->entities.object_[0]->GetControl() == Object::Control::HYBRID_EXTERNAL)
	{ 
		// Launch UDP server to receive external Ego state
		StartServer(scenarioEngine);
	}

	// Launch scenario engine in a separate thread
	thread.Start(scenario_thread, 0);

	while (!viewer_->osgViewer_->done())
	{
		viewer_frame();
	}
	quit_request = true;
	delete viewer_;

	if (scenarioEngine->entities.object_[0]->GetControl() == Object::Control::EXTERNAL ||
		scenarioEngine->entities.object_[0]->GetControl() == Object::Control::HYBRID_EXTERNAL)
	{
		StopServer();
	}

	delete scenarioEngine;

	return 0;
}


