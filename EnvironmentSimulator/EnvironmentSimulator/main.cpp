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

static const double maxStepSize = 0.1;
static const double minStepSize = 0.01;
static const bool freerun = true;
static int viewer_state = VIEWER_NOT_STARTED;


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

	if (arguments.argc() < 2)
	{
		arguments.getApplicationUsage()->write(std::cout, 1, 80, true);
		return -1;
	}

	std::string oscFilename;
	arguments.read("--osc", oscFilename);

	std::string ext_control_str;
	arguments.read("--ext_control", ext_control_str);

	ExternalControlMode ext_control;
	if (ext_control_str == "osc" || ext_control_str == "") ext_control = ExternalControlMode::EXT_CONTROL_BY_OSC;
	else if (ext_control_str == "off") ext_control = ExternalControlMode::EXT_CONTROL_OFF;
	else if (ext_control_str == "on") ext_control = ExternalControlMode::EXT_CONTROL_ON;
	else
	{
		LOG("Unrecognized external control mode: %s", ext_control_str.c_str());
		ext_control = ExternalControlMode::EXT_CONTROL_BY_OSC;
	}

	std::string record_filename;
	arguments.read("--record", record_filename);

	// Use logger callback
	Logger::Inst().SetCallback(log_callback);

	// Create scenario engine
	try 
	{ 
		scenarioEngine = new ScenarioEngine(oscFilename, simulationTime, ext_control);
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

	if(scenarioEngine->GetExtControl())
	{ 
		// Launch UDP server to receive external Ego state
		StartServer(scenarioEngine);
	}

	__int64 now, lastTimeStamp = 0;
	double simTime = 0;

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

	if (scenarioEngine->GetExtControl())
	{
		StopServer();
	}

	delete scenarioEngine;

	return 0;
}


