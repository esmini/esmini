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


static ScenarioEngine *scenarioEngine;

static SE_Thread thread;
static SE_Mutex mutex;

void viewer_thread(void *args)
{
	osg::ArgumentParser *parser = (osg::ArgumentParser*)args;

	// Create viewer
	viewer::Viewer *viewer = new viewer::Viewer(
		roadmanager::Position::GetOpenDrive(), 
		scenarioEngine->getSceneGraphFilename().c_str(), 
		scenarioEngine->getScenarioFilename().c_str(), 
		*parser);

	std::string info_text_str;
	parser->read("--info_text", info_text_str);
	if (info_text_str == "off")
	{
		viewer->ShowInfoText(false);
	}

	//  Create cars for visualization
	for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
	{
		if ((viewer->AddCar(scenarioEngine->entities.object_[i]->model_filepath_)) == 0)
		{
			delete viewer;
			viewer_state = VIEWER_FAILED;
			return;
		}
	}

	while (!viewer->osgViewer_->done())
	{

		mutex.Lock();

		// Visualize cars
		for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
		{
			viewer::CarModel *car = viewer->cars_[i];
			roadmanager::Position pos = scenarioEngine->entities.object_[i]->pos_;

			car->SetPosition(pos.GetX(), pos.GetY(), pos.GetZ());
			car->SetRotation(pos.GetH(), pos.GetP(), pos.GetR());
			car->UpdateWheels(scenarioEngine->entities.object_[i]->wheel_angle, 0);
		}

		// Update info text 
		static char str_buf[128];
		snprintf(str_buf, sizeof(str_buf), "%.2fs %.2fkm/h", simTime, 3.6 * scenarioEngine->entities.object_[viewer->currentCarInFocus_]->speed_);
		viewer->SetInfoText(str_buf);

		mutex.Unlock();

		viewer->osgViewer_->frame();
		
		viewer_state = VIEWER_RUNNING;
	}

	delete viewer;

	viewer_state = VIEWER_DONE;
}

void log_callback(const char *str)
{
	printf("%s\n", str);
}

int main(int argc, char *argv[])
{	
	double deltaSimTime;

	// Simulation constants
	double endTime = 100;
	double simulationTime = 0;
	double timeStep = 1;

	// use an ArgumentParser object to manage the program arguments.
	osg::ArgumentParser arguments(&argc, argv);

	arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName());
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options]\n");
	arguments.getApplicationUsage()->addCommandLineOption("--osc <filename>", "OpenSCENARIO filename");
	arguments.getApplicationUsage()->addCommandLineOption("--ext_control <mode>", "Ego control (\"osc\", \"off\", \"on\")");
	arguments.getApplicationUsage()->addCommandLineOption("--info_text <mode>", "Show info text HUD (\"on\" (default), \"off\") (toggle during simulation by press 't') ");

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
		scenarioEngine = new ScenarioEngine(oscFilename);
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
	thread.Start(viewer_thread, &arguments);
	
	// Wait for the viewer to launch
	while (viewer_state == VIEWER_NOT_STARTED) SE_sleep(100);
	if (viewer_state == VIEWER_FAILED)
	{
		return -1;
	}

	if(scenarioEngine->GetControl() == Object::Control::EXTERNAL ||
		scenarioEngine->GetControl() == Object::Control::HYBRID_EXTERNAL)
	{ 
		// Launch UDP server to receive external Ego state
		StartServer(scenarioEngine);
	}

	__int64 now, lastTimeStamp = 0;

	while (viewer_state == VIEWER_RUNNING)
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

		// Time operations
		simTime = simTime + deltaSimTime;

		// ScenarioEngine
		mutex.Lock();
	
		scenarioEngine->step(deltaSimTime);

		mutex.Unlock();
	}

	if (scenarioEngine->GetControl() == Object::Control::EXTERNAL ||
		scenarioEngine->GetControl() == Object::Control::HYBRID_EXTERNAL)
	{
		StopServer();
	}

	delete scenarioEngine;

	return 0;
}


