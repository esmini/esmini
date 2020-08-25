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

#define GHOST_HEADSTART 2.5

static int osi_counter = 0; 

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
	maxStepSize(0.1), minStepSize(0.01), argc_(argc), argv_(argv)
{
	quit_request = false;
	threads = false;
	headless = false;
	launch_server = false;
	fixed_timestep_ = -1.0;
	viewer_ = 0;
	osi_receiver_addr = "";
	osi_file = false; 
	osi_freq_ = 1;
	CSV_Log = NULL;

#ifdef _SCENARIO_VIEWER
	viewerState_ = ViewerState::VIEWER_STATE_NOT_STARTED;
	trail_dt = TRAIL_DOTS_DT;
#else
	trail_dt = 0;
#endif

	if (Init() != 0)
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
		if (threads)
		{
			viewer_->SetQuitRequest(true);
			thread.Wait();
		}
		else
		{
			CloseViewer();
		}
#endif
	}
	delete scenarioEngine; 
}

void ScenarioPlayer::Frame(double timestep_s)
{
	static bool messageShown = false;

	ScenarioFrame(timestep_s);
	
	if (!headless)
	{
#ifdef _SCENARIO_VIEWER
		if (!threads)
		{
			ViewerFrame();
		}
#endif
	}

	if (scenarioEngine->getSimulationTime() > 3600 && !messageShown)
	{
		LOG("Info: Simulation time > 1 hour. Put a stopTrigger for automatic ending");
		messageShown = true;
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

	// Update OSI info
	if (osi_file || scenarioGateway->GetSocket())
	{
		osi_counter++; 
		if (osi_counter % osi_freq_ == 0 )
		{
			scenarioEngine->getScenarioGateway()->UpdateOSISensorView();
			if (osi_file)
			{
				scenarioEngine->getScenarioGateway()->WriteOSIFile();
			}
		}	
	}

	//LOG("%d %d %.2f h: %.5f road_h %.5f h_relative_road %.5f",
	//    scenarioEngine->entities.object_[0]->pos_.GetTrackId(),
	//    scenarioEngine->entities.object_[0]->pos_.GetLaneId(),
	//    scenarioEngine->entities.object_[0]->pos_.GetS(),
	//    scenarioEngine->entities.object_[0]->pos_.GetH(),
	//    scenarioEngine->entities.object_[0]->pos_.GetHRoad(),
	//    scenarioEngine->entities.object_[0]->pos_.GetHRelative());

	mutex.Unlock();
	
	if (scenarioEngine->GetQuitFlag())
	{
		quit_request = true;
	}
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


	// Add or remove cars (for sumo)
	
	if (scenarioEngine->entities.object_.size() > viewer_->cars_.size())
	{
		// add cars missing
		for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
		{
			bool toadd = true;
			for (size_t j = 0; j < viewer_->cars_.size(); j++)
			{
				if (scenarioEngine->entities.object_[i]->name_ == viewer_->cars_[j]->name_)
				{
					toadd = false;
				}
			}
			if (toadd)
			{
				// add car
				osg::Vec3 trail_color;
				trail_color.set(color_blue[0], color_blue[1], color_blue[2]);
				viewer_->AddCar(scenarioEngine->entities.object_[i]->model_filepath_, false,trail_color, false ,scenarioEngine->entities.object_[i]->name_);
			}
		}
	} 
	else if (scenarioEngine->entities.object_.size() < viewer_->cars_.size())
	{
		// remove obsolete cars
		for (size_t j = 0; j < viewer_->cars_.size(); j++)
		{
			bool toremove = true;
			for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
			{
				if (scenarioEngine->entities.object_[i]->name_ == viewer_->cars_[j]->name_)
				{
					toremove = false;
				}
				
			}
			if (toremove)
			{
				// remove car
				viewer_->RemoveCar(viewer_->cars_[j]->name_);
			}
		}
	}
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
		LOG("Quit requested from viewer - probably ESC button pressed");
		viewerState_ = ViewerState::VIEWER_STATE_DONE;
		quit_request = true;
	}

}

void ScenarioPlayer::CloseViewer()
{
	delete viewer_;
	viewerState_ = ScenarioPlayer::ViewerState::VIEWER_STATE_DONE;
}

int ScenarioPlayer::InitViewer()
{
	std::string arg_str;

	// Create viewer
	osg::ArgumentParser arguments(&argc_, argv_);
	viewer_ = new viewer::Viewer(
		roadmanager::Position::GetOpenDrive(),
		scenarioEngine->getSceneGraphFilename().c_str(),
		scenarioEngine->getScenarioFilename().c_str(),
		arguments, &opt);

	if (opt.GetOptionArg("info_text") == "off")
	{
		viewer_->ShowInfoText(false);
	}

	if (opt.GetOptionArg("trails") == "off")
	{
		viewer_->ShowTrail(false);
	}

	if (opt.GetOptionArg("road_features") == "off")
	{
		viewer_->ShowRoadFeatures(false);
	}

	if (opt.GetOptionArg("osi_features") == "on")
	{
		viewer_->ShowOSIFeatures(true);
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
		else if (arg_str == "top")
		{
			viewer_->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_TOP);
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
		Object* obj = scenarioEngine->entities.object_[i];

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
		if (viewer_->AddCar(obj->model_filepath_, transparent, trail_color, road_sensor, obj->name_) == 0)
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

	viewerState_ = ViewerState::VIEWER_STATE_STARTED;

	return 0;
}

void viewer_thread(void *args)
{
	ScenarioPlayer *player = (ScenarioPlayer*)args;
	__int64 time_stamp = 0;

	if (player->InitViewer() != 0)
	{
		LOG("Failed to initialize the Viewer!");
		return;
	}

	while (!player->viewer_->GetQuitRequest() && !player->viewer_->osgViewer_->done())
	{
		player->ViewerFrame();
	}

	player->CloseViewer();
}

#endif

void ScenarioPlayer::AddObjectSensor(int object_index, double x, double y, double z, double h, double near, double far, double fovH, int maxObj)
{
	sensor.push_back(new ObjectSensor(&scenarioEngine->entities, scenarioEngine->entities.object_[object_index], x, y, z, h, near, far, fovH, maxObj));
 	if (!headless)
	{
#ifdef _SCENARIO_VIEWER
		mutex.Lock();
		sensorFrustum.push_back(new viewer::SensorViewFrustum(sensor.back(), viewer_->cars_[object_index]->txNode_));
		mutex.Unlock();
#endif
	}
}

void ScenarioPlayer::ShowObjectSensors(bool mode)
{
	// Switch on sensor visualization as defult when sensors are added
	if (viewer_)
	{
		mutex.Lock();
		viewer_->ShowObjectSensors(mode);
		mutex.Unlock();
	}
}

int ScenarioPlayer::Init()
{
	std::string arg_str;

	// Use logger callback
	Logger::Inst().SetCallback(log_callback);

	// use an ArgumentParser object to manage the program arguments.
	opt.AddOption("osc", "OpenSCENARIO filename", "filename");
	opt.AddOption("control", "Ego control (\"osc\", \"internal\", \"external\", \"hybrid\"", "mode");
	opt.AddOption("record", "Record position data into a file for later replay", "filename");
	opt.AddOption("csv_logger", "Log data for each vehicle in ASCII csv format", "csv_filename");
	opt.AddOption("info_text", "Show info text HUD (\"on\" (default), \"off\") (toggle during simulation by press 'i') ", "mode");
	opt.AddOption("trails", "Show trails (\"on\" (default), \"off\") (toggle during simulation by press 'j') ", "mode");
	opt.AddOption("road_features", "Show road features (\"on\" (default), \"off\") (toggle during simulation by press 'o') ", "mode");
	opt.AddOption("osi_features", "Show OSI road features (\"on\", \"off\" (default)) (toggle during simulation by press 'u') ", "mode");
	opt.AddOption("sensors", "Show sensor frustums (\"on\", \"off\" (default)) (toggle during simulation by press 'r') ", "mode");
	opt.AddOption("camera_mode", "Initial camera mode (\"orbit\" (default), \"fixed\", \"flex\", \"flex-orbit\", \"top\") (toggle during simulation by press 'k') ", "mode");
	opt.AddOption("aa_mode", "Anti-alias mode=number of multisamples (subsamples, 0=off, 4=default)", "mode");
	opt.AddOption("threads", "Run viewer in a separate thread, parallel to scenario engine");
	opt.AddOption("headless", "Run without viewer");
	opt.AddOption("server", "Launch server to receive state of external Ego simulator");
	opt.AddOption("fixed_timestep", "Run simulation decoupled from realtime, with specified timesteps", "timestep");
	opt.AddOption("osi_receiver_ip", "IP address where to send OSI UDP packages", "IP address");
	opt.AddOption("ghost_headstart", "Launch Ego ghost at specified headstart time", "time");
	opt.AddOption("osi_file", "save osi messages in file (\"on\", \"off\" (default))", "mode");
	opt.AddOption("osi_freq", "relative frequence for writing the .osi file e.g. --osi_freq=2 -> we write every two simulation steps", "frequence");

	if (argc_ < 3)
	{
		opt.PrintUsage();
		return -1;
	}

	opt.ParseArgs(&argc_, argv_);

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
#ifdef __APPLE__
		LOG("Separate viewer thread requested. Unfortunately only supported on Windows and Linux.");
		LOG("See https://www.mail-archive.com/osg-users@lists.openscenegraph.org/msg72698.html for an explanation.");
		return -1;
#else
		threads = true;
		LOG("Run viewer in separate thread");
#endif
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
	
	double ghost_headstart = GHOST_HEADSTART;
	if ((arg_str = opt.GetOptionArg("ghost_headstart")) != "")
	{
		ghost_headstart = atof(arg_str.c_str());
		LOG("Any ghosts will be launched with headstart %.2f seconds", ghost_headstart);
	}
	else
	{
		LOG("Any ghosts will be launched with headstart %.2f seconds (default)", ghost_headstart);
	}

	// Create scenario engine
	try
	{
		if ((arg_str = opt.GetOptionArg("osc")) == "")
		{
			LOG("Missing OpenSCENARIO filename argument");
			opt.PrintUsage();
			return -1;
		}
		scenarioEngine = new ScenarioEngine(arg_str, ghost_headstart, (ScenarioEngine::RequestControlMode)control);
	}
	catch (std::logic_error &e)
	{
		printf("%s\n", e.what());
		return -1;
	}

	// Fetch scenario gateway and OpenDRIVE manager objects
	scenarioGateway = scenarioEngine->getScenarioGateway();
	odr_manager = scenarioEngine->getRoadManager();

	if (opt.GetOptionSet("osi_receiver_ip"))
	{
		scenarioGateway->OpenSocket(opt.GetOptionArg("osi_receiver_ip"));
	}
	
	if (opt.GetOptionArg("osi_file") ==  "on")
	{
		osi_file = true;
		if (scenarioGateway->OpenOSIFile() == false)
		{
			osi_file = false;
		}
	}
	
	if ((arg_str = opt.GetOptionArg("osi_freq")) != "")
	{
		if (osi_file == false)
		{
			LOG("Specify osi frequence without --osi_file on is not possible"); 
			return -1; 
		}
		osi_freq_ = atoi(arg_str.c_str());
		LOG("Run simulation decoupled from realtime, with fixed timestep: %.2f", GetFixedTimestep());
	}	

	// Initialize CSV logger for recording vehicle data
	if (opt.GetOptionSet("csv_logger"))
	{
		CSV_Log = &CSV_Logger::InstVehicleLog(scenarioEngine->getScenarioFilename(),
			scenarioEngine->entities.object_.size(), opt.GetOptionArg("csv_logger"));
		LOG("Log all vehicle data in csv file");
	}

	// Create a data file for later replay?
	if ((arg_str = opt.GetOptionArg("record")) != "")
	{
		LOG("Recording data to file %s", arg_str.c_str());
		scenarioGateway->RecordToFile(arg_str, scenarioEngine->getOdrFilename(), scenarioEngine->getSceneGraphFilename());
	}

	// Step scenario engine - zero time - just to reach and report init state of all vehicles
	scenarioEngine->step(0.0, true);

	// Update OSI info
	if (osi_file || scenarioGateway->GetSocket())
	{
		scenarioEngine->getScenarioGateway()->UpdateOSISensorView();
		if (osi_file)
		{
			scenarioEngine->getScenarioGateway()->WriteOSIFile();
		}
	}

	if (!headless)
	{

#ifdef _SCENARIO_VIEWER

		if (threads)
		{
			// Launch Viewer in a separate thread
			thread.Start(viewer_thread, (void*)this);

			// Wait for viewer to initialize
			for (int i = 0; i < 60 && viewerState_ == VIEWER_STATE_NOT_STARTED; i++)
			{
				SE_sleep(100);
			}

			if (viewerState_ == ViewerState::VIEWER_STATE_NOT_STARTED)
			{
				LOG("Viewer still not ready. Start scenario anyway. Viewer will launch when ready.");
			}
			else if (viewerState_ == ViewerState::VIEWER_STATE_DONE)
			{
				LOG("Viewer already signaled done - something went wrong");
				return -1;
			}
		}
		else
		{
			InitViewer();
		}

#endif
	}

	if (argc_ > 1)
	{
		opt.PrintArgs(argc_, argv_, "Unrecognized arguments:");
		opt.PrintUsage();
	}

	if (launch_server && (scenarioEngine->entities.object_[0]->GetControl() == Object::Control::EXTERNAL ||
		scenarioEngine->entities.object_[0]->GetControl() == Object::Control::HYBRID_EXTERNAL))
	{
		// Launch UDP server to receive external Ego state
		StartServer(scenarioEngine);
	}

	return 0;
}
