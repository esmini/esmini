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

#include <iostream>
#include <string>
#include <random>

#include "ScenarioEngine.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "Server.hpp"
#include "IdealSensor.hpp"
#ifdef _SCENARIO_VIEWER
  #include "viewer.hpp"
#endif

using namespace scenarioengine;

class ScenarioPlayer
{
	typedef enum
	{
		CONTROL_BY_OSC,
		CONTROL_INTERNAL,
		CONTROL_EXTERNAL,
		CONTROL_HYBRID
	} RequestControlMode;

public:
	ScenarioPlayer(int &argc, char *argv[]);
	~ScenarioPlayer();
	bool IsQuitRequested() { return quit_request; }
	void Frame();  // let player calculate actual time step
	void Frame(double timestep_s);
	void ScenarioFrame(double timestep_s);
	void AddObjectSensor(int object_index, double pos_x, double pos_y, double pos_z, double near, double far, double fovH, int maxObj);
	void SetFixedTimestep(double timestep) { fixed_timestep_ = timestep; }
	double GetFixedTimestep() { return fixed_timestep_; }

	ScenarioEngine *scenarioEngine;
	ScenarioGateway *scenarioGateway;
#ifdef _SCENARIO_VIEWER
	viewer::Viewer *viewer_;
	std::vector<viewer::SensorViewFrustum*> sensorFrustum;
#endif
	roadmanager::OpenDrive *odr_manager;
	std::vector<ObjectSensor*> sensor;
	const double maxStepSize;
	const double minStepSize;

private:
	std::string RequestControlMode2Str(RequestControlMode mode);
	int Init(int &rgc, char *argv[]);
#ifdef _SCENARIO_VIEWER
	void ViewerFrame();
#endif

	double trail_dt;
	SE_Thread thread;
	SE_Mutex mutex;
	bool quit_request;
	bool threads;
	bool headless;
	bool launch_server;
	double fixed_timestep_;
};