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

static void log_callback(const char *str)
{
	printf("%s\n", str);
}

ScenarioPlayer::ScenarioPlayer(int &argc, char *argv[]) :
	maxStepSize(0.1), minStepSize(0.01), argc_(argc), argv_(argv)
{
	quit_request = false;
	threads = false;
	headless = false;
	launch_server = false;
	fixed_timestep_ = -1.0;
	osi_receiver_addr = "";
	osi_freq_ = 1;
	CSV_Log = NULL;
	osiReporter = NULL;
	viewer_ = 0;
	disable_controllers_ = false;

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
	if (launch_server)
	{
		StopServer();
	}

#if _SCENARIO_VIEWER
	if (!headless)
	{
		if (viewer_)
		{
			if (threads)
			{
				viewer_->SetQuitRequest(true);
				thread.Wait();
			}
			else
			{
				CloseViewer();
			}
		}
	}
#endif
	Logger::Inst().SetTimePtr(0);
	delete scenarioEngine; 

	if (osiReporter)
	{
		delete osiReporter;
	}
}

void ScenarioPlayer::SetOSIFileStatus(bool is_on, const char* filename)
{
	if (osiReporter)
	{
		if (is_on)
		{
			osiReporter->OpenOSIFile(filename);
		}
		else
		{
			osiReporter->CloseOSIFile();
		}
	}
}

void ScenarioPlayer::Frame(double timestep_s)
{
	static bool messageShown = false;

	ScenarioFrame(timestep_s);
	
	if (!headless && viewer_)
	{
#if _SCENARIO_VIEWER
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
	mutex.Lock();

	scenarioEngine->step(timestep_s);

	// Check for any callbacks to be made
	for (size_t i = 0; i < callback.size(); i++)
	{
		ObjectState *os = scenarioGateway->getObjectStatePtrById(callback[i].id);
		if (os)
		{
			ObjectStateStruct state;
			state = os->getStruct();
			callback[i].func(&state, callback[i].data);
		}
	}

	scenarioEngine->prepareOSIGroundTruth(timestep_s);

	scenarioGateway->WriteStatesToFile();

	mutex.Unlock();

	for (size_t i = 0; i < sensor.size(); i++)
	{
		sensor[i]->Update();
	}

	osiReporter->ReportSensors(sensor);

	// Update OSI info
	if (osiReporter->IsFileOpen() || osiReporter->GetSocket())
	{
		if (osi_counter % osi_freq_ == 0)
		{
			osiReporter->UpdateOSIGroundTruth(scenarioGateway->objectState_);
		}
		// Update counter after modulo-check since first frame should always be reported
		osi_counter++;
	}

	//LOG("%d %d %.2f h: %.5f road_h %.5f h_relative_road %.5f",
	//    scenarioEngine->entities.object_[0]->pos_.GetTrackId(),
	//    scenarioEngine->entities.object_[0]->pos_.GetLaneId(),
	//    scenarioEngine->entities.object_[0]->pos_.GetS(),
	//    scenarioEngine->entities.object_[0]->pos_.GetH(),
	//    scenarioEngine->entities.object_[0]->pos_.GetHRoad(),
	//    scenarioEngine->entities.object_[0]->pos_.GetHRelative());


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

	if (scenarioEngine->entities.object_.size() > viewer_->entities_.size())
	{
		// add cars missing
		for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
		{
			bool toadd = true;
			if (scenarioEngine->entities.object_[i]->type_ == Object::Type::VEHICLE)
			{
				for (size_t j = 0; j < viewer_->entities_.size(); j++)
				{
					if (scenarioEngine->entities.object_[i]->name_ == viewer_->entities_[j]->name_)
					{
						toadd = false;
					}
				}
				if (toadd)
				{
					// add car
					osg::Vec3 trail_color;
					trail_color.set(color_blue[0], color_blue[1], color_blue[2]);
					viewer_->AddEntityModel(scenarioEngine->entities.object_[i]->model_filepath_, trail_color, 
						viewer::EntityModel::EntityType::ENTITY_TYPE_VEHICLE, false, 
						scenarioEngine->entities.object_[i]->name_, &scenarioEngine->entities.object_[i]->boundingbox_);
				}
			}
		}
	}
	else if (scenarioEngine->entities.object_.size() < viewer_->entities_.size())
	{
		// remove obsolete cars
		for (size_t j = 0; j < viewer_->entities_.size(); j++)
		{
			if (viewer_->entities_[j]->entity_type_ == viewer::EntityModel::EntityType::ENTITY_TYPE_VEHICLE)
			{
				bool toremove = true;
				for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
				{
					if (scenarioEngine->entities.object_[i]->name_ == viewer_->entities_[j]->name_)
					{
						toremove = false;
					}

				}
				if (toremove)
				{
					// remove car
					viewer_->RemoveCar(viewer_->entities_[j]->name_);
				}
			}
		}
	}
	// Visualize entities
	for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
	{
		viewer::EntityModel *entity = viewer_->entities_[i];
		Object *obj = scenarioEngine->entities.object_[i];
		roadmanager::Position pos = obj->pos_;

		entity->SetPosition(pos.GetX(), pos.GetY(), pos.GetZ());
		entity->SetRotation(pos.GetH(), pos.GetP(), pos.GetR());

		if (entity->GetType() == viewer::EntityModel::EntityType::ENTITY_TYPE_VEHICLE)
		{
			viewer::CarModel* car = (viewer::CarModel*)entity;
			car->UpdateWheels(obj->wheel_angle_, obj->wheel_rot_);

			if (obj->GetGhost())
			{
				if (car->steering_sensor_)
				{
					viewer_->SensorSetPivotPos(car->steering_sensor_, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ());
					viewer_->SensorSetTargetPos(car->steering_sensor_, obj->sensor_pos_[0], obj->sensor_pos_[1], obj->sensor_pos_[2]);
					viewer_->UpdateSensor(car->steering_sensor_);
				}
				if (car->trail_sensor_)
				{
					viewer_->SensorSetPivotPos(car->trail_sensor_, obj->trail_closest_pos_[0], obj->trail_closest_pos_[1], obj->trail_closest_pos_[2]);
					viewer_->SensorSetTargetPos(car->trail_sensor_, pos.GetX(), pos.GetY(), pos.GetZ());
					viewer_->UpdateSensor(car->trail_sensor_);
				}
			}

			if (odr_manager->GetNumOfRoads() > 0 && car->road_sensor_)
			{
				viewer_->UpdateRoadSensors(car->road_sensor_, car->lane_sensor_, &pos);
			}
		}

		if (add_dot)
		{
			entity->trail_->AddDot(pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetH());
		}
	}

	for (size_t i = 0; i < sensorFrustum.size(); i++)
	{
		sensorFrustum[i]->Update();
	}

	// Update info text
	static char str_buf[128];
	if (viewer_->currentCarInFocus_ >= 0 && viewer_->currentCarInFocus_ < viewer_->entities_.size())
	{
		Object* obj = scenarioEngine->entities.object_[viewer_->currentCarInFocus_];
		snprintf(str_buf, sizeof(str_buf), "%.2fs entity[%d]: %s %.2fkm/h %.2fm (%d, %d, %.2f, %.2f) / (%.2f, %.2f %.2f)", scenarioEngine->getSimulationTime(),
			viewer_->currentCarInFocus_, obj->name_.c_str(), 3.6 * obj->speed_, obj->odometer_,
			obj->pos_.GetTrackId(), obj->pos_.GetLaneId(), fabs(obj->pos_.GetOffset()) < SMALL_NUMBER ? 0 : obj->pos_.GetOffset(), 
			obj->pos_.GetS(), obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetH());
	}
	else
	{
		snprintf(str_buf, sizeof(str_buf), "No object in focus...");
	}
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
		exe_path_.c_str(),
		arguments, &opt);

	if (viewer_->osgViewer_ == 0)
	{
		viewerState_ = ViewerState::VIEWER_STATE_FAILED;
		return -1;
	}

	if (opt.GetOptionArg("info_text") == "off")
	{
		viewer_->ShowInfoText(false);
	}

	if (opt.GetOptionSet("trails"))
	{
		viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_TRAILS);
	}

	if (opt.GetOptionArg("road_features") == "on")
	{
		viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_ODR_FEATURES);
	}
	else if (opt.GetOptionArg("road_features") == "off")
	{
		viewer_->ClearNodeMaskBits(viewer::NodeMask::NODE_MASK_ODR_FEATURES);
	}

	if (opt.GetOptionSet("osi_lines"))
	{
		viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_OSI_LINES);
	}

	if (opt.GetOptionSet("osi_points"))
	{
		viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_OSI_POINTS);
	}

	if (opt.GetOptionSet("sensors"))
	{
		viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_OBJECT_SENSORS);
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

	if (opt.GetOptionSet("bounding_boxes"))
	{
		viewer_->ClearNodeMaskBits(viewer::NodeMask::NODE_MASK_ENTITY_MODEL);
		viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_ENTITY_BB);
	}

	//  Create visual models
	for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
	{
		osg::Vec3 trail_color;
		Object* obj = scenarioEngine->entities.object_[i];

		// Create trajectory/trails for all entities
		if (obj->IsGhost())
		{
			trail_color.set(color_gray[0], color_gray[1], color_gray[2]);
		}
		else if (obj->GetAssignedControllerType() == Controller::Type::CONTROLLER_TYPE_EXTERNAL)
		{
			trail_color.set(color_yellow[0], color_yellow[1], color_yellow[2]);
		}
		else
		{
			trail_color.set(color_red[0], color_red[1], color_red[2]);
		}

		//  Create vehicles for visualization
		bool road_sensor = false;
		if (obj->GetGhost() ||
			obj->GetAssignedControllerType() == Controller::Type::CONTROLLER_TYPE_INTERACTIVE ||
			obj->GetAssignedControllerType() == Controller::Type::CONTROLLER_TYPE_EXTERNAL)
		{
			road_sensor = true;
		}

		if (viewer_->AddEntityModel(obj->model_filepath_, trail_color,
			obj->type_ == Object::Type::VEHICLE ? viewer::EntityModel::EntityType::ENTITY_TYPE_VEHICLE : viewer::EntityModel::EntityType::ENTITY_TYPE_OTHER,
			road_sensor, obj->name_, &obj->boundingbox_) == 0)
		{
			delete viewer_;
			viewer_ = 0;
			return -1;
		}

		// Connect callback for setting transparency
		viewer::VisibilityCallback* cb = new viewer::VisibilityCallback(viewer_->entities_.back()->txNode_, obj, viewer_->entities_.back());
		viewer_->entities_.back()->txNode_->setUpdateCallback(cb);

		if (viewer_->entities_.back()->GetType() == viewer::EntityModel::ENTITY_TYPE_VEHICLE)
		{
			viewer::CarModel* model = (viewer::CarModel*)viewer_->entities_.back();

			// Add a sensor to show when query road info ahead
			model->steering_sensor_ = viewer_->CreateSensor(color_green, true, true, 0.4, 3);
			viewer_->SensorSetPivotPos(model->steering_sensor_, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ());
			viewer_->SensorSetTargetPos(model->steering_sensor_, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ());
			if (obj->ghost_)
			{
				// Show steering sensor when following a ghost
				model->steering_sensor_->Show();
			}
			else
			{
				// Otherwise hide it (as default)
				model->steering_sensor_->Hide();
			}

			// If following a ghost vehicle, add visual representation of speed and steering sensors
			if (scenarioEngine->entities.object_[i]->GetGhost())
			{
				if (odr_manager->GetNumOfRoads() > 0)
				{
					model->trail_sensor_ = viewer_->CreateSensor(color_red, true, false, 0.4, 3);
				}

			}
			else if (scenarioEngine->entities.object_[i]->IsGhost())
			{
				scenarioEngine->entities.object_[i]->SetVisibilityMask(scenarioEngine->entities.object_[i]->visibilityMask_ &= ~(Object::Visibility::SENSORS));
			}
		}
	}

	// Choose vehicle to look at initially (switch with 'Tab')
	viewer_->SetVehicleInFocus(0);
	for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
	{
		Object* obj = scenarioEngine->entities.object_[i];
		
		if (obj->GetAssignedControllerType() == Controller::Type::CONTROLLER_TYPE_INTERACTIVE ||
			obj->GetAssignedControllerType() == Controller::Type::CONTROLLER_TYPE_EXTERNAL ||
			obj->GetAssignedControllerType() == Controller::Type::CONTROLLER_TYPE_FOLLOW_GHOST)
		{
			if (viewer_->GetEntityInFocus() == 0)
			{
				// Focus on first vehicle of specified types
				viewer_->SetVehicleInFocus(i);
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

	if (player->InitViewer() != 0)
	{
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
#if _SCENARIO_VIEWER
		if (viewer_)
		{
			mutex.Lock();
			sensorFrustum.push_back(new viewer::SensorViewFrustum(sensor.back(), viewer_->entities_[object_index]->txNode_));
			mutex.Unlock();
		}
#endif
	}
}

void ScenarioPlayer::ShowObjectSensors(bool mode)
{
	// Switch on sensor visualization as defult when sensors are added
#if _SCENARIO_VIEWER
	if (viewer_)
	{
		mutex.Lock();
		if (mode == false)
		{
			viewer_->ClearNodeMaskBits(viewer::NodeMask::NODE_MASK_OBJECT_SENSORS);
		}
		else
		{
			viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_OBJECT_SENSORS);
		}
		mutex.Unlock();
	}
#endif
}

int ScenarioPlayer::Init()
{
	std::string arg_str;

	// Use logger callback
	if (!(Logger::Inst().IsCallbackSet()))
	{
		Logger::Inst().SetCallback(log_callback);
	}

	// use an ArgumentParser object to manage the program arguments.
	opt.AddOption("osc", "OpenSCENARIO filename - if path includes spaces, enclose with \"\" ", "filename");
	opt.AddOption("osc_str", "OpenSCENARIO XML string", "string");
	opt.AddOption("disable_controllers", "Disable controllers");
	opt.AddOption("record", "Record position data into a file for later replay", "filename");
	opt.AddOption("csv_logger", "Log data for each vehicle in ASCII csv format", "csv_filename");
	opt.AddOption("info_text", "Show info text HUD (\"on\" (default), \"off\") (toggle during simulation by press 'i') ", "mode");
	opt.AddOption("trails", "Show trails (toggle during simulation by press 'j')");
	opt.AddOption("road_features", "Show OpenDRIVE road features (\"on\", \"off\"  (default)) (toggle during simulation by press 'o') ", "mode");
	opt.AddOption("bounding_boxes", "Show entities as bounding boxes (toggle modes on key ',') ");
	opt.AddOption("osi_lines", "Show OSI road lines (toggle during simulation by press 'u') ");
	opt.AddOption("osi_points", "Show OSI road pointss (toggle during simulation by press 'y') ");
	opt.AddOption("sensors", "Show sensor frustums (toggle during simulation by press 'r') ");
	opt.AddOption("camera_mode", "Initial camera mode (\"orbit\" (default), \"fixed\", \"flex\", \"flex-orbit\", \"top\") (toggle during simulation by press 'k') ", "mode");
	opt.AddOption("aa_mode", "Anti-alias mode=number of multisamples (subsamples, 0=off, 4=default)", "mode");
	opt.AddOption("threads", "Run viewer in a separate thread, parallel to scenario engine");
	opt.AddOption("headless", "Run without viewer");
	opt.AddOption("server", "Launch server to receive state of external Ego simulator");
	opt.AddOption("fixed_timestep", "Run simulation decoupled from realtime, with specified timesteps", "timestep");
	opt.AddOption("osi_receiver_ip", "IP address where to send OSI UDP packages", "IP address");
	opt.AddOption("osi_file", "save osi messages in file (\"on\", \"off\" (default))", "mode");
	opt.AddOption("osi_freq", "relative frequence for writing the .osi file e.g. --osi_freq=2 -> we write every two simulation steps", "frequence");
	opt.AddOption("path", "Search path prefix for assets, e.g. OpenDRIVE files (will be concatenated with filepath)", "path");

	if (argc_ < 3)
	{
		opt.PrintUsage();
		return -1;
	}

	exe_path_ = argv_[0];
	opt.ParseArgs(&argc_, argv_);

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

	if ((arg_str = opt.GetOptionArg("path")) != "")
	{
		SE_Env::Inst().AddPath(arg_str);
		LOG("Added path %s", arg_str.c_str());
	}

	if (opt.GetOptionSet("disable_controllers"))
	{
		disable_controllers_ = true;
		LOG("Disable entity controllers");
	}

	// Create scenario engine
	try
	{
		if ((arg_str = opt.GetOptionArg("osc")) != "")
		{
			scenarioEngine = new ScenarioEngine(arg_str, disable_controllers_);
			Logger::Inst().SetTimePtr(scenarioEngine->GetSimulationTimePtr());
		}
		else if ((arg_str = opt.GetOptionArg("osc_str")) != "")
		{
			// parse XML string as document
			pugi::xml_document doc;
			std::string xml_str(arg_str);
			if (!doc.load_buffer(xml_str.c_str(), xml_str.length()))
			{
				return -1;
			}
			scenarioEngine = new ScenarioEngine(doc, disable_controllers_);
			Logger::Inst().SetTimePtr(scenarioEngine->GetSimulationTimePtr());
		}
		else
		{
			LOG("Missing OpenSCENARIO filename argument or XML string");
			opt.PrintUsage();
			return -1;
		}
		
		
	}
	catch (std::logic_error &e)
	{
		printf("%s\n", e.what());
		return -1;
	}

	// Fetch scenario gateway and OpenDRIVE manager objects
	scenarioGateway = scenarioEngine->getScenarioGateway();
	odr_manager = scenarioEngine->getRoadManager();
	osiReporter = new OSIReporter();

	if (opt.GetOptionSet("osi_receiver_ip"))
	{
		osiReporter->OpenSocket(opt.GetOptionArg("osi_receiver_ip"));
	}

	if (opt.GetOptionArg("osi_file") ==  "on")
	{
		osiReporter->OpenOSIFile(0);
	}

	if ((arg_str = opt.GetOptionArg("osi_freq")) != "")
	{
		if (!osiReporter->IsFileOpen())
		{
			LOG("Specifying osi frequency without --osi_file on is not possible");
			return -1; 
		}
		osi_freq_ = atoi(arg_str.c_str());
		LOG("Run simulation decoupled from realtime, with fixed timestep: %.2f", GetFixedTimestep());
	}

	// Initialize CSV logger for recording vehicle data
	if (opt.GetOptionSet("csv_logger"))
	{
		CSV_Log = &CSV_Logger::InstVehicleLog(scenarioEngine->getScenarioFilename(),
			(int)scenarioEngine->entities.object_.size(), opt.GetOptionArg("csv_logger"));
		LOG("Log all vehicle data in csv file");
	}

	// Create a data file for later replay?
	if ((arg_str = opt.GetOptionArg("record")) != "")
	{
		LOG("Recording data to file %s", arg_str.c_str());
		scenarioGateway->RecordToFile(arg_str, scenarioEngine->getOdrFilename(), scenarioEngine->getSceneGraphFilename());
	}

	// Step scenario engine - zero time - just to reach and report init state of all vehicles
	ScenarioFrame(0.0);
	
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
			else if (viewerState_ == ViewerState::VIEWER_STATE_FAILED)
			{
				LOG("Viewer initialization failed");
				return -1;
			}
		}
		else
		{
			InitViewer();
		}

		// Decorate window border with application name and arguments
		viewer_->SetWindowTitleFromArgs(opt.GetOriginalArgs());

		viewer_->RegisterKeyEventCallback(ReportKeyEvent, this);
#endif
	}

	if (argc_ > 1)
	{
		opt.PrintArgs(argc_, argv_, "Unrecognized arguments:");
		opt.PrintUsage();
	}

	if (launch_server)
	{
		// Launch UDP server to receive external Ego state
		StartServer(scenarioEngine);
	}

	return 0;
}

void ScenarioPlayer::RegisterObjCallback(int id, ObjCallbackFunc func, void *data)
{
	ObjCallback cb;
	cb.id = id;
	cb.func = func;
	cb.data = data;
	callback.push_back(cb);
}

void ScenarioPlayer::UpdateCSV_Log()
{
	//Flag for signalling end of data line, all vehicles reported
	bool isendline = false;

	//For each vehicle (entitity) stored in the ScenarioPlayer
	for (size_t i = 0; i < scenarioEngine->entities.object_.size(); i++)
	{
		//Create a pointer to the object at position i in the entities vector
		Object* obj = scenarioEngine->entities.object_[i];

		//Create a Position object for extracting this vehicles XYZ coordinates
		roadmanager::Position pos = obj->pos_;

		//Extract the String name of the object and store in a compatable const char array
		const char* name_ = &(*obj->name_.c_str());

		if ((i + 1) == scenarioEngine->entities.object_.size())
		{
			isendline = true;
		}

		//Log the extracted data of ego vehicle and additonal scenario vehicles
		CSV_Log->LogVehicleData(isendline, scenarioEngine->getSimulationTime(), name_,
			obj->id_, obj->speed_, obj->wheel_angle_, obj->wheel_rot_,
			pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetS(), pos.GetT(), pos.GetH(),
			pos.GetHRelative(), pos.GetHRelativeDrivingDirection(),
			pos.GetP(), pos.GetCurvature());
	}
}

int ScenarioPlayer::SetParameterValue(const char* name, const void* value)
{
	return scenarioEngine->scenarioReader->parameters.setParameterValue(name, value);
}

int ScenarioPlayer::GetParameterValue(const char* name, void* value)
{
	return scenarioEngine->scenarioReader->parameters.getParameterValue(name, value);
}

#ifdef _SCENARIO_VIEWER
	void ReportKeyEvent(viewer::KeyEvent* keyEvent, void* data)
	{
		ScenarioPlayer* player = (ScenarioPlayer*)data;
		for (size_t i = 0; i < player->scenarioEngine->GetScenarioReader()->controller_.size(); i++)
		{
			player->scenarioEngine->GetScenarioReader()->controller_[i]->ReportKeyEvent(keyEvent->key_, keyEvent->down_);
		}
	}
#endif

