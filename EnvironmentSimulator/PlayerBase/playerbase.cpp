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
#include "playerbase.hpp"
#ifdef _SCENARIO_VIEWER
	#include "viewer.hpp"
#endif

using namespace scenarioengine;

#define GHOST_HEADSTART 5.0

void log_callback(const char *str)
{
	printf("%s\n", str);
}

std::string ScenarioPlayer::RequestControlMode2Str(RequestControlMode mode)
{
	if (mode == RequestControlMode::CONTROL_BY_OSC) return "by OSC";
	else if (mode == RequestControlMode::CONTROL_INTERNAL) return "Internal";
	else if (mode == RequestControlMode::CONTROL_EXTERNAL) return "External";
	else if (mode == RequestControlMode::CONTROL_HYBRID) return "Hybrid";
	else return "Unknown";
}

ScenarioPlayer::ScenarioPlayer(int &argc, char *argv[]) : 
	maxStepSize(0.1), minStepSize(0.01)
{
	quit_request = false;
	threads = false;
	headless = false;
	launch_server = false;
	fixed_timestep_ = -1.0;
#ifdef _SCENARIO_VIEWER
	trail_dt = TRAIL_DOTS_DT;
#else
	trail_dt = 0;
#endif

	if (Init(argc, argv) != 0)
	{
		throw std::invalid_argument("Failed to initialize scenario player");
	}
}

ScenarioPlayer::~ScenarioPlayer()
{
	if (launch_server && (scenarioEngine->entities.object_[0]->GetControl() == Object::Control::EXTERNAL ||
		scenarioEngine->entities.object_[0]->GetControl() == Object::Control::HYBRID_EXTERNAL))
	{
		StopServer();
	}

	if (!headless)
	{
#ifdef _SCENARIO_VIEWER
		delete viewer_;
#endif
	}
	delete scenarioEngine;
}

void ScenarioPlayer::Frame(double timestep_s)
{
	if (!threads)
	{
		ScenarioFrame(timestep_s);
	}
	if (!headless)
	{
#ifdef _SCENARIO_VIEWER
		ViewerFrame();
#endif
	}
}

void ScenarioPlayer::Frame()
{
	static __int64 time_stamp = 0;
	double dt;
	if ((dt = GetFixedTimestep()) < 0.0)
	{
		Frame(SE_getSimTimeStep(time_stamp, minStepSize, maxStepSize));
	}
	else
	{
		Frame(dt);
	}
}

void ScenarioPlayer::ScenarioFrame(double timestep_s)
{
	for (size_t i = 0; i < sensor.size(); i++)
	{
		sensor[i]->Update();
		//LOG("sensor identified %d objects", sensor[i]->nObj_);
	}

	mutex.Lock();

	scenarioEngine->step(timestep_s);
	//LOG("%d %d %.2f h: %.5f road_h %.5f h_relative_road %.5f",
	//    scenarioEngine->entities.object_[0]->pos_.GetTrackId(),
	//    scenarioEngine->entities.object_[0]->pos_.GetLaneId(),
	//    scenarioEngine->entities.object_[0]->pos_.GetS(),
	//    scenarioEngine->entities.object_[0]->pos_.GetH(),
	//    scenarioEngine->entities.object_[0]->pos_.GetHRoad(),
	//    scenarioEngine->entities.object_[0]->pos_.GetHRelative());

	mutex.Unlock();
}

#ifdef _SCENARIO_VIEWER
void ScenarioPlayer::ViewerFrame()
{
	static double last_dot_time = scenarioEngine->getSimulationTime();

	bool add_dot = false;
	if (scenarioEngine->getSimulationTime() - last_dot_time > trail_dt)
	{
		add_dot = true;
		last_dot_time = scenarioEngine->getSimulationTime();
	}

	mutex.Lock();

	// Visualize cars
	for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
	{
		viewer::CarModel *car = viewer_->cars_[i];
		Object *obj = scenarioEngine->entities.object_[i];
		roadmanager::Position pos = obj->pos_;

		car->SetPosition(pos.GetX(), pos.GetY(), pos.GetZ());
		car->SetRotation(pos.GetH(), pos.GetP(), pos.GetR());
		car->UpdateWheels(obj->wheel_angle_, obj->wheel_rot_);

		if (obj->GetControl() == Object::Control::EXTERNAL ||
			obj->GetControl() == Object::Control::HYBRID_EXTERNAL)
		{

			if (obj->GetControl() == Object::Control::HYBRID_EXTERNAL)
			{
				viewer_->SensorSetPivotPos(car->speed_sensor_, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ());
				viewer_->UpdateSensor(car->speed_sensor_);
				viewer_->SensorSetPivotPos(car->steering_sensor_, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ());
				viewer_->UpdateSensor(car->steering_sensor_);

				double closest_pos[3];
				if (obj->ghost_->trail_.FindClosestPoint(pos.GetX(), pos.GetY(), closest_pos[0], closest_pos[1],
					obj->trail_follow_s_, obj->trail_follow_index_, obj->trail_follow_index_) == 0)
				{
					closest_pos[2] = obj->pos_.GetZ();
				}
				else
				{
					// Failed find point along trail, copy entity position
					closest_pos[0] = obj->pos_.GetX();
					closest_pos[1] = obj->pos_.GetY();
					closest_pos[2] = obj->pos_.GetZ();
				}
				viewer_->SensorSetPivotPos(car->trail_sensor_, closest_pos[0], closest_pos[1], closest_pos[2]);
				viewer_->SensorSetTargetPos(car->trail_sensor_, pos.GetX(), pos.GetY(), pos.GetZ());
				viewer_->UpdateSensor(car->trail_sensor_);
			}
			else if (odr_manager->GetNumOfRoads() > 0 && obj->GetControl() == Object::Control::EXTERNAL)
			{
				viewer_->UpdateRoadSensors(car->road_sensor_, car->lane_sensor_, &pos);
			}
		}

		if (add_dot)
		{
			car->trail_->AddDot(scenarioEngine->getSimulationTime(), pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetH());
		}
	}

	for (size_t i = 0; i < sensorFrustum.size(); i++)
	{
		sensorFrustum[i]->Update();
	}

	// Update info text 
	static char str_buf[128];
	snprintf(str_buf, sizeof(str_buf), "%.2fs %.2fkm/h", scenarioEngine->getSimulationTime(), 
		3.6 * scenarioEngine->entities.object_[viewer_->currentCarInFocus_]->speed_);
	viewer_->SetInfoText(str_buf);

	mutex.Unlock();

	viewer_->osgViewer_->frame();
	if (viewer_->osgViewer_->done())
	{
		quit_request = true;
	}
}
#endif

void scenario_thread(void *args)
{
	ScenarioPlayer *player = (ScenarioPlayer*)args;
	__int64 time_stamp = 0;

	while (!player->IsQuitRequested())
	{
		double dt;
		if ((dt = player->GetFixedTimestep()) < 0.0)
		{
			player->ScenarioFrame(SE_getSimTimeStep(time_stamp, player->minStepSize, player->maxStepSize));
		}
		else
		{
			player->ScenarioFrame(dt);
		}
	}
}

void ScenarioPlayer::AddObjectSensor(int object_index, double pos_x, double pos_y, double pos_z, double near, double far, double fovH, int maxObj)
{
	sensor.push_back(new ObjectSensor(&scenarioEngine->entities, scenarioEngine->entities.object_[object_index], 
		pos_x, pos_y, pos_z, near, far, fovH, maxObj));
 	if (!headless)
	{
#ifdef _SCENARIO_VIEWER
		sensorFrustum.push_back(new viewer::SensorViewFrustum(sensor.back(), viewer_->cars_[object_index]->txNode_));
#endif
	}
}

int ScenarioPlayer::Init(int &argc, char *argv[])
{
	std::string arg_str;

	// Use logger callback
	Logger::Inst().SetCallback(log_callback);

	// use an ArgumentParser object to manage the program arguments.
	SE_Options opt;
	opt.AddOption("osc", "OpenSCENARIO filename", "filename");
	opt.AddOption("control", "Ego control (\"osc\", \"internal\", \"external\", \"hybrid\"", "mode");
	opt.AddOption("record", "Record position data into a file for later replay", "filename");
	opt.AddOption("info_text", "Show info text HUD (\"on\" (default), \"off\") (toggle during simulation by press 't') ", "mode");
	opt.AddOption("trails", "Show trails (\"on\" (default), \"off\") (toggle during simulation by press 't') ", "mode");
	opt.AddOption("sensors", "Show sensor frustums (\"on\", \"off\" (default)) (toggle during simulation by press 'r') ", "mode");
	opt.AddOption("camera_mode", "Initial camera mode (\"orbit\" (default), \"fixed\", \"flex\", \"flex-orbit\") (toggle during simulation by press 'c') ", "mode");
	opt.AddOption("aa_mode", "Anti-alias mode=number of multisamples (subsamples, 0=off, 4=default)", "mode");
	opt.AddOption("threads", "Run viewer and scenario in separate threads");
	opt.AddOption("headless", "Run without viewer");
	opt.AddOption("server", "Launch server to receive state of external Ego simulator");
	opt.AddOption("fixed_timestep", "Run simulation decoupled from realtime, with specified timesteps", "timestep");

	if (argc < 3)
	{
		opt.PrintUsage();
		return -1;
	}

	opt.ParseArgs(&argc, argv);

	RequestControlMode control = RequestControlMode::CONTROL_BY_OSC;
	if ((arg_str = opt.GetOptionArg("control")) != "")
	{
		if (arg_str == "osc" || arg_str == "") control = RequestControlMode::CONTROL_BY_OSC;
		else if (arg_str == "internal") control = RequestControlMode::CONTROL_INTERNAL;
		else if (arg_str == "external") control = RequestControlMode::CONTROL_EXTERNAL;
		else if (arg_str == "hybrid") control = RequestControlMode::CONTROL_HYBRID;
		else LOG("Unrecognized external control mode: %s - defaulting to ", arg_str.c_str());
	}

	if (opt.GetOptionSet("threads"))
	{
		threads = true;
		LOG("Run viewer in separate thread");
	}

	if (opt.GetOptionSet("headless"))
	{
		headless = true;
		LOG("Run without viewer");
	}

	if (opt.GetOptionSet("server"))
	{
		launch_server = true;
		LOG("Launch server to receive state of external Ego simulator");
	}

	if ((arg_str = opt.GetOptionArg("fixed_timestep")) != "")
	{
		SetFixedTimestep(atof(arg_str.c_str()));
		LOG("Run simulation decoupled from realtime, with fixed timestep: %.2f", GetFixedTimestep());
	}

	// Create scenario engine
	try
	{
		arg_str = opt.GetOptionArg("osc");
		scenarioEngine = new ScenarioEngine(arg_str, GHOST_HEADSTART, (ScenarioEngine::RequestControlMode)control);
	}
	catch (std::logic_error &e)
	{
		printf("%s\n", e.what());
		return -1;
	}

	// Fetch scenario gateway and OpenDRIVE manager objects
	scenarioGateway = scenarioEngine->getScenarioGateway();
	odr_manager = scenarioEngine->getRoadManager();
	
	// Create a data file for later replay?
	if ((arg_str = opt.GetOptionArg("record")) != "")
	{
		LOG("Recording data to file %s", arg_str.c_str());
		scenarioGateway->RecordToFile(arg_str, scenarioEngine->getOdrFilename(), scenarioEngine->getSceneGraphFilename());
	}

	// Step scenario engine - zero time - just to reach and report init state of all vehicles
	scenarioEngine->step(0.0, true);


	if (!headless)
	{

#ifdef _SCENARIO_VIEWER

		// Create viewer
		osg::ArgumentParser arguments(&argc, argv);
		viewer_ = new viewer::Viewer(
			roadmanager::Position::GetOpenDrive(),
			scenarioEngine->getSceneGraphFilename().c_str(),
			scenarioEngine->getScenarioFilename().c_str(),
			arguments);

		if (opt.GetOptionArg("info_text") == "off")
		{
			viewer_->ShowInfoText(false);
		}

		if (opt.GetOptionArg("trails") == "off")
		{
			viewer_->ShowTrail(false);
		}

		if (opt.GetOptionArg("sensors") == "on")
		{
			viewer_->ShowObjectSensors(true);
		}

		if ((arg_str = opt.GetOptionArg("camera_mode")) != "")
		{
			if (arg_str == "orbit")
			{
				viewer_->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_ORBIT);
			}
			else if (arg_str == "fixed")
			{
				viewer_->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_FIXED);
			}
			else if (arg_str == "flex")
			{
				viewer_->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_RUBBER_BAND);
			}
			else if (arg_str == "flex-orbit")
			{
				viewer_->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_RUBBER_BAND_ORBIT);
			}
			else 
			{
				LOG("Unsupported camera mode: %s - using default (orbit)", arg_str.c_str());
			}
		}

		//  Create cars for visualization
		for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
		{
			//  Create vehicles for visualization
			osg::Vec3 trail_color;
			Object *obj = scenarioEngine->entities.object_[i];

			if (obj->control_ == Object::Control::HYBRID_GHOST)
			{
				trail_color.set(color_gray[0], color_gray[1], color_gray[2]);
			}
			else if (obj->control_ == Object::Control::HYBRID_EXTERNAL || obj->control_ == Object::Control::EXTERNAL)
			{
				trail_color.set(color_yellow[0], color_yellow[1], color_yellow[2]);
			}
			else
			{
				trail_color.set(color_red[0], color_red[1], color_red[2]);
			}

			bool transparent = obj->control_ == Object::Control::HYBRID_GHOST ? true : false;
			bool road_sensor = obj->control_ == Object::Control::HYBRID_GHOST || obj->control_ == Object::Control::EXTERNAL ? true : false;
			if (viewer_->AddCar(obj->model_filepath_, transparent, trail_color, road_sensor) == 0)
			{
				delete viewer_;
				viewer_ = 0;
				return -1;
			}

			if (obj->GetControl() == Object::Control::HYBRID_EXTERNAL)
			{
				viewer_->cars_.back()->steering_sensor_ = viewer_->CreateSensor(color_green, true, true, 0.4, 3);
				if (odr_manager->GetNumOfRoads() > 0)
				{
					viewer_->cars_.back()->speed_sensor_ = viewer_->CreateSensor(color_gray, true, true, 0.4, 1);
					viewer_->cars_.back()->trail_sensor_ = viewer_->CreateSensor(color_red, true, false, 0.4, 3);
				}
			}
		}

		// Trig first viewer frame, it typically takes extra long due to initial loading of gfx content
		ViewerFrame();
#endif
	}

	if (argc > 1)
	{
		opt.PrintArgs(argc, argv, "Unrecognized arguments:");
		opt.PrintUsage();
		return -1;
	}

	// Add sensors
	for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
	{
		Object *obj = scenarioEngine->entities.object_[i];

		if (obj->GetControl() == Object::Control::EXTERNAL ||
			obj->GetControl() == Object::Control::HYBRID_EXTERNAL)
		{
			AddObjectSensor((int)i, 4, 0, 0.5, 5, 50, 50 * M_PI / 180.0, 10);
		}
	}

	if (launch_server && (scenarioEngine->entities.object_[0]->GetControl() == Object::Control::EXTERNAL ||
		scenarioEngine->entities.object_[0]->GetControl() == Object::Control::HYBRID_EXTERNAL))
	{
		// Launch UDP server to receive external Ego state
		StartServer(scenarioEngine);
	}

	if (threads)
	{
		// Launch scenario engine in a separate thread
		thread.Start(scenario_thread, (void*)this);
	}

	return 0;
}
