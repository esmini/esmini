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
public:
	typedef enum
	{
		CONTROL_BY_OSC,
		CONTROL_INTERNAL,
		CONTROL_EXTERNAL,
		CONTROL_HYBRID
	} RequestControlMode;

	typedef enum
	{
		VIEWER_STATE_NOT_STARTED,
		VIEWER_STATE_STARTED,
		VIEWER_STATE_DONE
	} ViewerState;

	ScenarioPlayer(int &argc, char *argv[]);
	~ScenarioPlayer();
	bool IsQuitRequested() { return quit_request; }
	void Frame();  // let player calculate actual time step
	void Frame(double timestep_s);
	void ScenarioFrame(double timestep_s);
	void ShowObjectSensors(bool mode);
	void AddObjectSensor(int object_index, double pos_x, double pos_y, double pos_z, double heading, 
		double near, double far, double fovH, int maxObj);
	void SetFixedTimestep(double timestep) { fixed_timestep_ = timestep; }
	double GetFixedTimestep() { return fixed_timestep_; }
	int GetOSIFreq() { return osi_freq_; }
	
	CSV_Logger *CSV_Log;
	ScenarioEngine *scenarioEngine;
	ScenarioGateway *scenarioGateway;
#ifdef _SCENARIO_VIEWER
	viewer::Viewer *viewer_;
	std::vector<viewer::SensorViewFrustum*> sensorFrustum;
	ViewerState viewerState_;
	int InitViewer();
	void CloseViewer();
	void ViewerFrame();
#endif
	roadmanager::OpenDrive *odr_manager;
	std::vector<ObjectSensor*> sensor;
	const double maxStepSize;
	const double minStepSize;
	SE_Options opt;

private:
	std::string RequestControlMode2Str(RequestControlMode mode);
	int Init();

	double trail_dt;
	SE_Thread thread;
	SE_Mutex mutex;
	bool quit_request;
	bool threads;
	bool headless;
	bool launch_server;
	double fixed_timestep_;
	bool osi_file; 
	int osi_freq_; 
	std::string osi_receiver_addr;
	int& argc_;
	char** argv_;
};