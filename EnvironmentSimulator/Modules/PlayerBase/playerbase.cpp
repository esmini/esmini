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
#include "helpText.hpp"
#ifdef _USE_OSG
	#include "viewer.hpp"
#endif

using namespace scenarioengine;

#define GHOST_HEADSTART 2.5
#define TRAIL_Z_OFFSET 0.02

#ifdef _USE_OSG
void RegisterImageCallback(viewer::ImageCallbackFunc func, void* data)
{
	imageCallback.func = func;
	imageCallback.data = data;
}
#endif

static void log_callback(const char *str)
{
	printf("%s\n", str);
}

ScenarioPlayer::ScenarioPlayer(int &argc, char *argv[]) :
	maxStepSize(0.1), minStepSize(0.01), argc_(argc), argv_(argv), state_(PlayerState::PLAYER_STATE_PLAYING)
{
	quit_request = false;
	threads = false;
	launch_server = false;
	fixed_timestep_ = -1.0;
	osi_receiver_addr = "";
	osi_freq_ = 1;
	CSV_Log = NULL;
	osiReporter = NULL;
	viewer_ = 0;
	disable_controllers_ = false;
	frame_counter_ = 0;

#ifdef _USE_OSG
	viewerState_ = ViewerState::VIEWER_STATE_NOT_STARTED;
	OSISensorDetection = nullptr;
#endif

	int retval = Init();
	if (retval == -1)
	{
		throw std::invalid_argument("Failed to initialize scenario player");
	}
	else if (retval == -2)
	{
		throw std::invalid_argument("Skipped initialize scenario player");
	}
}

ScenarioPlayer::~ScenarioPlayer()
{
	if (launch_server)
	{
		StopServer();
	}

#ifdef _USE_OSG
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
#endif  // _USE_OSG
	Logger::Inst().SetTimePtr(0);
	delete scenarioEngine;

#ifdef _USE_OSI
	if (osiReporter)
	{
		delete osiReporter;
	}
#endif  // _USE_OSI
}

void ScenarioPlayer::SetOSIFileStatus(bool is_on, const char* filename)
{
#ifdef _USE_OSI
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
#endif // USE_OSI
}
void ScenarioPlayer::Draw()
{
	if (viewer_)
	{
#ifdef _USE_OSG
		if (!threads)
		{
			if (!viewer_->GetQuitRequest())
			{
				ViewerFrame();
			}
			else
			{
				SetQuitRequest(true);
				CloseViewer();
			}
		}
#endif
	}
}

void ScenarioPlayer::Frame(double timestep_s)
{
	static bool messageShown = false;
	double dt = timestep_s;
	__int64 time_stamp = 0;
	int retval = 0;

	if (GetState() != PlayerState::PLAYER_STATE_PAUSE)
	{
		if (ScenarioFrame(timestep_s, true) == 0)
		{
			while (retval == 0 && scenarioEngine->GetGhostMode() != GhostMode::NORMAL)
			{
				// Main simulation is paused during ghost headstart or restart

				Draw();
				retval = ScenarioFrame(dt, false);

				if (GetFixedTimestep() > SMALL_NUMBER)
				{
					dt = GetFixedTimestep();
				}
				else
				{
					dt = SE_getSimTimeStep(time_stamp, minStepSize, maxStepSize);
				}
			}

			if (retval == 0)
			{
				ScenarioPostFrame();
			}
		}


		if (GetState() == PlayerState::PLAYER_STATE_STEP)
		{
			SetState(PlayerState::PLAYER_STATE_PAUSE);
		}
	}

	Draw();

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

int ScenarioPlayer::ScenarioFrame(double timestep_s, bool keyframe)
{
	int retval = 0;
	mutex.Lock();

	if ((retval = scenarioEngine->step(timestep_s)) == 0)
	{
		if (keyframe)
		{
			// Check for any callbacks to be made
			for (size_t i = 0; i < objCallback.size(); i++)
			{
				ObjectState* os = scenarioGateway->getObjectStatePtrById(objCallback[i].id);
				if (os)
				{
					ObjectStateStruct state;
					state = os->getStruct();
					objCallback[i].func(&state, objCallback[i].data);
				}
			}
		}

		scenarioEngine->prepareGroundTruth(timestep_s);

		scenarioGateway->WriteStatesToFile();

		if (CSV_Log)
		{
			UpdateCSV_Log();
		}

		if (keyframe) frame_counter_++;
	}

	mutex.Unlock();

	if (scenarioEngine->GetQuitFlag())
	{
		quit_request = true;
	}

	return retval;
}

void ScenarioPlayer::ScenarioPostFrame()
{
	mutex.Lock();


	for (size_t i = 0; i < sensor.size(); i++)
	{
		sensor[i]->Update();
	}
#ifdef _USE_OSI
	if (NEAR_NUMBERS(scenarioEngine->getSimulationTime(), scenarioEngine->GetTrueTime()))
	{
		osiReporter->ReportSensors(sensor);

		// Update OSI info
		if (osiReporter->IsFileOpen() || osiReporter->GetSocket())
		{
			if (osiReporter->GetCounter() % osi_freq_ == 0)
			{
				osiReporter->UpdateOSIGroundTruth(scenarioGateway->objectState_);
				if (osiReporter->GetCounter() == 0)
				{
					// Clear the static data now when it has been reported once
					osiReporter->ClearOSIGroundTruth();
				}
			}
		}
	}
#endif  // USE_OSI

	//LOG("%d %d %.2f h: %.5f road_h %.5f h_relative_road %.5f",
	//    scenarioEngine->entities_.object_[0]->pos_.GetTrackId(),
	//    scenarioEngine->entities_.object_[0]->pos_.GetLaneId(),
	//    scenarioEngine->entities_.object_[0]->pos_.GetS(),
	//    scenarioEngine->entities_.object_[0]->pos_.GetH(),
	//    scenarioEngine->entities_.object_[0]->pos_.GetHRoad(),
	//    scenarioEngine->entities_.object_[0]->pos_.GetHRelative());

	mutex.Unlock();

}

#ifdef _USE_OSG
void ScenarioPlayer::ViewerFrame()
{
	static double last_dot_time = scenarioEngine->getSimulationTime();

	mutex.Lock();

	// remove deleted cars
	osg::Vec4 trail_color;
	trail_color.set(color_blue[0], color_blue[1], color_blue[2], 1.0);
	for (size_t i = 0; i < viewer_->entities_.size() && i < scenarioEngine->entities_.object_.size(); i++)
	{
		if (scenarioEngine->entities_.object_[i]->name_ != viewer_->entities_[i]->name_ ||
			scenarioEngine->entities_.object_[i]->model3d_ != viewer_->entities_[i]->filename_)
		{
			// Object has most probably been deleted from the entity list
			viewer_->RemoveCar(i);
			i--;  // test same object again against next in viewer list
		}
	}

	// Add missing cars
	while (viewer_->entities_.size() < scenarioEngine->entities_.object_.size())
	{
		Object* obj = scenarioEngine->entities_.object_[viewer_->entities_.size()];
		viewer_->AddEntityModel(viewer_->CreateEntityModel(obj->model3d_, trail_color,
			viewer::EntityModel::EntityType::VEHICLE, false,
			obj->name_, &obj->boundingbox_, obj->scaleMode_));
		viewer_->entities_.back()->routewaypoints_->SetWayPoints(obj->pos_.GetRoute());
	}

	// remove obsolete cars
	while (viewer_->entities_.size() > scenarioEngine->entities_.object_.size())
	{
		if (viewer_->entities_.back()->trajectory_->activeRMTrajectory_)
		{
			viewer_->entities_.back()->trajectory_->Disable();
		}
		viewer_->RemoveCar(viewer_->entities_.size() - 1);
	}

	// Visualize entities
	for (size_t i = 0; i < scenarioEngine->entities_.object_.size(); i++)
	{
		viewer::EntityModel *entity = viewer_->entities_[i];
		Object* obj = scenarioEngine->entities_.object_[i];

		entity->SetPosition(obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ());
		entity->SetRotation(obj->pos_.GetH(), obj->pos_.GetP(), obj->pos_.GetR());

		if (obj->pos_.GetTrajectory() && obj->pos_.GetTrajectory() != entity->trajectory_->activeRMTrajectory_)
		{
			entity->trajectory_->SetActiveRMTrajectory(obj->pos_.GetTrajectory());
		}
		else if (entity->trajectory_->activeRMTrajectory_ && !obj->pos_.GetTrajectory())
		{
			// Trajectory has been deactivated on the entity, disable visualization
			entity->trajectory_->Disable();
		}

		if (obj->CheckDirtyBits(Object::DirtyBit::ROUTE))
		{
			entity->routewaypoints_->SetWayPoints(obj->pos_.GetRoute());
			obj->ClearDirtyBits(Object::DirtyBit::ROUTE);
		}
		else if (entity->routewaypoints_->group_->getNumChildren() && obj->pos_.GetRoute() == nullptr)
		{
			entity->routewaypoints_->SetWayPoints(nullptr);
		}


		if (entity->GetType() == viewer::EntityModel::EntityType::VEHICLE)
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
					viewer_->SensorSetPivotPos(car->trail_sensor_, obj->trail_closest_pos_.x, obj->trail_closest_pos_.y, obj->trail_closest_pos_.z);
					viewer_->SensorSetTargetPos(car->trail_sensor_, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ());
					viewer_->UpdateSensor(car->trail_sensor_);
				}
			}

			if (odr_manager->GetNumOfRoads() > 0 && car->road_sensor_)
			{
				car->ShowRouteSensor(obj->pos_.GetRoute() ? true : false);
				viewer_->UpdateRoadSensors(car->road_sensor_, car->route_sensor_, car->lane_sensor_, &obj->pos_);
			}
		}

		if (entity->trail_->pline_vertex_data_->size() > obj->trail_.GetNumberOfVertices())
		{
			// Reset the trail, probably there has been a ghost restart
			entity->trail_->Reset();
			for (size_t j = 0; j < obj->trail_.GetNumberOfVertices(); j++)
			{
				entity->trail_->AddPoint(osg::Vec3(obj->trail_.vertex_[j].x, obj->trail_.vertex_[j].y, obj->trail_.vertex_[j].z + (obj->GetId() + 1) * TRAIL_Z_OFFSET));
			}
		}

		if (obj->trail_.GetNumberOfVertices() > entity->trail_->pline_vertex_data_->size())
		{
			entity->trail_->AddPoint(osg::Vec3(obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ() + (obj->GetId() + 1) * TRAIL_Z_OFFSET));
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
		Object* obj = scenarioEngine->entities_.object_[viewer_->currentCarInFocus_];
		snprintf(str_buf, sizeof(str_buf), "%.2fs entity[%d]: %s (%d) %.2fkm/h %.2fm (%d, %d, %.2f, %.2f) / (%.2f, %.2f %.2f)", scenarioEngine->getSimulationTime(),
			viewer_->currentCarInFocus_, obj->name_.c_str(), obj->GetId(), 3.6 * obj->speed_, obj->odometer_,
			obj->pos_.GetTrackId(), obj->pos_.GetLaneId(), fabs(obj->pos_.GetOffset()) < SMALL_NUMBER ? 0 : obj->pos_.GetOffset(),
			obj->pos_.GetS(), obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetH());
	}
	else
	{
		snprintf(str_buf, sizeof(str_buf), "%.2fs No entity in focus...", scenarioEngine->getSimulationTime());
	}
	viewer_->SetInfoText(str_buf);

	mutex.Unlock();

	viewer_->Frame();
}

int ScenarioPlayer::SaveImagesToRAM(bool state)
{
	if (viewer_)
	{
		viewer_->imageMutex.Lock();
		viewer_->SaveImagesToRAM(state);
		viewer_->imageMutex.Unlock();
		return 0;
	}

	return -1;
}

int ScenarioPlayer::SaveImagesToFile(int nrOfFrames)
{
	if (viewer_)
	{
		viewer_->imageMutex.Lock();
		viewer_->SaveImagesToFile(nrOfFrames);
		viewer_->imageMutex.Unlock();
		return 0;
	}

	return -1;
}

OffScreenImage* ScenarioPlayer::FetchCapturedImagePtr()
{
	static OffScreenImage img;

	if (viewer_ && !viewer_->GetDisableOffScreen())
	{
		if (viewer_->GetSaveImagesToRAM() == false)
		{
			LOG("FetchCapturedImagePtr Error: Activate save images to RAM (SaveImagesToRAM(true)) in order to fetch images\n");
			return nullptr;
		}

		viewer_->renderSemaphore.Wait();  // Wait until rendering is done

		viewer_->imageMutex.Lock();

		OffScreenImage* tmpImg = &viewer_->capturedImage_;

		if (tmpImg != nullptr)
		{
			// Check whether image data has to be allocated due to first time or changed window size
			if (img.height * img.width != tmpImg->height * tmpImg->width)
			{
				if (img.data != nullptr)
				{
					free(img.data);
				}
				if (tmpImg->height * tmpImg->width > 0)
				{
					img.data = (unsigned char*)malloc(tmpImg->pixelSize * tmpImg->height * tmpImg->width * sizeof(unsigned char));
				}
			}
			if (img.data != nullptr)
			{
				img.height = tmpImg->height;
				img.width = tmpImg->width;
				img.pixelSize = tmpImg->pixelSize;
				img.pixelFormat = tmpImg->pixelFormat;
				memcpy(img.data, tmpImg->data, tmpImg->pixelSize * tmpImg->height * tmpImg->width * sizeof(unsigned char));
			}
		}

		viewer_->imageMutex.Unlock();

		return &img;
	}

	return nullptr;
}

void ScenarioPlayer::AddCustomCamera(double x, double y, double z, double h, double p)
{
	if (viewer_)
	{
		viewer_->AddCustomCamera(x, y, z, h, p);
		viewer_->SetCameraMode(-1);  // activate last camera which is the one just added
	}
}

void ScenarioPlayer::CloseViewer()
{
	if (viewer_ != nullptr)
	{
		delete viewer_;
		viewer_ = nullptr;
	}
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

	viewer_->osgViewer_->setKeyEventSetsDone(0);  // Disable default Escape key event handler, take over control

	if (opt.GetOptionArg("info_text") == "off")
	{
		viewer_->ClearNodeMaskBits(viewer::NodeMask::NODE_MASK_INFO);
	}

	viewer_->RegisterImageCallback(imageCallback.func, imageCallback.data);

	if (opt.GetOptionSet("capture_screen"))
	{
		LOG("Activate continuous screen capture");
		viewer_->SaveImagesToFile(-1);
	}

	if ((arg_str = opt.GetOptionArg("trail_mode")) != "")
	{
		int mask = strtoi(arg_str);
		if (mask < 0 || mask > 3)
		{
			LOG_AND_QUIT("Invalid trail_mode %d. Valid range is 0-3", mask);
		}
		viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_TRAIL_LINES |
			viewer::NodeMask::NODE_MASK_TRAIL_DOTS, mask * viewer::NodeMask::NODE_MASK_TRAIL_LINES);
	}

	if (opt.GetOptionSet("hide_trajectories"))
	{
		LOG("Hide trajectories");
		viewer_->ClearNodeMaskBits(viewer::NodeMask::NODE_MASK_TRAJECTORY_LINES);
	}

	if (opt.GetOptionArg("road_features") == "on")
	{
		viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_ODR_FEATURES);
	}
	else if (opt.GetOptionArg("road_features") == "off")
	{
		viewer_->ClearNodeMaskBits(viewer::NodeMask::NODE_MASK_ODR_FEATURES);
	}

	if (opt.GetOptionSet("hide_route_waypoints"))
	{
		LOG("Disable route waypoint visualization");
		viewer_->ClearNodeMaskBits(viewer::NodeMask::NODE_MASK_ROUTE_WAYPOINTS);
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

	if (opt.GetOptionSet("custom_camera") == true)
	{
		int counter = 0;

		while ((arg_str = opt.GetOptionArg("custom_camera", counter)) != "")
		{
			size_t pos = 0;
			double v[5];
			for (int i = 0; i < 5; i++)
			{
				pos = arg_str.find(",");
				if (i < 4 && pos == std::string::npos)
				{
					LOG_AND_QUIT("Expected custom_camera <x,y,z,h,p>, got only %d values", i+1);
				}
				v[i] = strtod(arg_str.substr(0, pos));
				arg_str.erase(0, pos == std::string::npos ? pos : pos + 1);
			}
			if (!arg_str.empty())
			{
				LOG_AND_QUIT("Expected custom_camera <x,y,z,h,p>, got too many values. Make sure only five values is specified");
			}

			viewer_->AddCustomCamera(v[0], v[1], v[2], v[3], v[4]);
			LOG("Created custom camera %d (%.2f, %.2f, %.2f, %.2f, %.2f)", counter, v[0], v[1], v[2], v[3], v[4]);
			counter++;
		}
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
		else if (arg_str == "driver")
		{
			viewer_->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_DRIVER);
		}
		else if (arg_str == "custom")
		{
			viewer_->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_CUSTOM);
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
	for (size_t i = 0; i < scenarioEngine->entities_.object_.size(); i++)
	{
		osg::Vec4 trail_color;
		Object* obj = scenarioEngine->entities_.object_[i];

		// Create trajectory/trails for all entities
		if (obj->GetId() == 0)
		{
			trail_color.set(color_white[0], color_white[1], color_white[2], 1.0);
		}
		else if (obj->IsGhost())
		{
			trail_color.set(color_black[0], color_black[1], color_black[2], 1.0);
		}
		else if (obj->GetAssignedControllerType() == Controller::Type::CONTROLLER_TYPE_EXTERNAL)
		{
			trail_color.set(color_yellow[0], color_yellow[1], color_yellow[2], 1.0);
		}
		else
		{
			trail_color.set(color_red[0], color_red[1], color_red[2], 1.0);
		}

		//  Create vehicles for visualization
		bool road_sensor = false;
		if (obj->GetGhost() ||
			obj->GetAssignedControllerType() == Controller::Type::CONTROLLER_TYPE_INTERACTIVE ||
			obj->GetAssignedControllerType() == Controller::Type::CONTROLLER_TYPE_EXTERNAL)
		{
			road_sensor = true;
		}

		if (viewer_->AddEntityModel(viewer_->CreateEntityModel(obj->model3d_, trail_color,
			obj->type_ == Object::Type::VEHICLE ? viewer::EntityModel::EntityType::VEHICLE : viewer::EntityModel::EntityType::OTHER,
			road_sensor, obj->name_, &obj->boundingbox_, obj->scaleMode_)) != 0)
		{
			CloseViewer();
			return -1;
		}

		viewer_->entities_.back()->routewaypoints_->SetWayPoints(obj->pos_.GetRoute());

		// Connect callback for setting transparency
		viewer::VisibilityCallback* cb = new viewer::VisibilityCallback(viewer_->entities_.back()->txNode_, obj, viewer_->entities_.back());
		viewer_->entities_.back()->txNode_->setUpdateCallback(cb);

		if (viewer_->entities_.back()->GetType() == viewer::EntityModel::EntityType::VEHICLE)
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
			if (scenarioEngine->entities_.object_[i]->GetGhost())
			{
				if (odr_manager->GetNumOfRoads() > 0)
				{
					model->trail_sensor_ = viewer_->CreateSensor(color_red, true, false, 0.4, 3);
				}

			}
			else if (scenarioEngine->entities_.object_[i]->IsGhost())
			{
				scenarioEngine->entities_.object_[i]->SetVisibilityMask(scenarioEngine->entities_.object_[i]->visibilityMask_ &= ~(Object::Visibility::SENSORS));
			}
		}
	}

	// Choose vehicle to look at initially (switch with 'Tab')
	viewer_->SetVehicleInFocus(0);
	for (size_t i = 0; i < scenarioEngine->entities_.object_.size(); i++)
	{
		Object* obj = scenarioEngine->entities_.object_[i];

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

	while (!player->viewer_->GetQuitRequest())
	{
		player->ViewerFrame();
	}

	player->SetQuitRequest(true);
	player->CloseViewer();
}

#endif

void ScenarioPlayer::AddObjectSensor(int object_index, double x, double y, double z, double h, double near, double far, double fovH, int maxObj)
{
	sensor.push_back(new ObjectSensor(&scenarioEngine->entities_, scenarioEngine->entities_.object_[object_index], x, y, z, h, near, far, fovH, maxObj));

#ifdef _USE_OSG
	if (viewer_)
	{
		mutex.Lock();
		sensorFrustum.push_back(new viewer::SensorViewFrustum(sensor.back(), viewer_->entities_[object_index]->txNode_));
		mutex.Unlock();
	}
#endif
}

void ScenarioPlayer::AddOSIDetection(int object_index)
{
#ifdef _USE_OSG
	if (viewer_)
	{
		if(!OSISensorDetection)
		{
			mutex.Lock();
			OSISensorDetection = new viewer::OSISensorDetection(viewer_->entities_[object_index]->txNode_);
			mutex.Unlock();
		}
	}
#endif
}

void ScenarioPlayer::ShowObjectSensors(bool mode)
{
	// Switch on sensor visualization as defult when sensors are added
#ifdef _USE_OSG
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

void ScenarioPlayer::PrintUsage()
{
	opt.PrintUsage();
#ifdef _USE_OSG
	viewer::Viewer::PrintUsage();
#endif
}

int ScenarioPlayer::Init()
{
	// Use logger callback
	if (!(Logger::Inst().IsCallbackSet()))
	{
		Logger::Inst().SetCallback(log_callback);
	}

	std::string arg_str;

	// use an ArgumentParser object to manage the program arguments.
	opt.AddOption("osc", "OpenSCENARIO filename (required) - if path includes spaces, enclose with \"\"", "filename");
	opt.AddOption("aa_mode", "Anti-alias mode=number of multisamples (subsamples, 0=off, 4=default)", "mode");
	opt.AddOption("bounding_boxes", "Show entities as bounding boxes (toggle modes on key ',') ");
	opt.AddOption("capture_screen", "Continuous screen capture. Warning: Many jpeg files will be created");
	opt.AddOption("camera_mode", "Initial camera mode (\"orbit\" (default), \"fixed\", \"flex\", \"flex-orbit\", \"top\", \"driver\", \"custom\") (swith with key 'k') ", "mode");
	opt.AddOption("custom_camera", "Additional custom camera position <x,y,z,h,p,r> (multiple occurrences supported)", "position");
	opt.AddOption("csv_logger", "Log data for each vehicle in ASCII csv format", "csv_filename");
	opt.AddOption("collision", "Enable global collision detection, potentially reducing performance");
	opt.AddOption("disable_controllers", "Disable controllers");
	opt.AddOption("disable_log", "Prevent logfile from being created");
	opt.AddOption("disable_off_screen", "Disable off-screen rendering, potentially gaining performance");
	opt.AddOption("disable_stdout", "Prevent messages to stdout");
	opt.AddOption("enforce_generate_model", "Generate road 3D model even if SceneGraphFile is specified");
	opt.AddOption("fixed_timestep", "Run simulation decoupled from realtime, with specified timesteps", "timestep");
	opt.AddOption("generate_no_road_objects", "Do not generate any OpenDRIVE road objects (e.g. when part of referred 3D model)");
	opt.AddOption("headless", "Run without viewer window");
	opt.AddOption("help", "Show this help message");
	opt.AddOption("hide_route_waypoints", "Disable route waypoint visualization (toggle with key 'R')");
	opt.AddOption("hide_trajectories", "Hide trajectories from start (toggle with key 'n')");
	opt.AddOption("info_text", "Show info text HUD (\"on\" (default), \"off\")", "mode");
	opt.AddOption("logfile_path", "logfile path/filename, e.g. \"../esmini.log\" (default: log.txt)", "path");
	opt.AddOption("osc_str", "OpenSCENARIO XML string", "string");
#ifdef _USE_OSI
	opt.AddOption("osi_file", "save osi trace file", "filename", DEFAULT_OSI_TRACE_FILENAME);
	opt.AddOption("osi_freq", "relative frequence for writing the .osi file e.g. --osi_freq=2 -> we write every two simulation steps", "frequence");
	opt.AddOption("osi_lines", "Show OSI road lines (toggle during simulation by press 'u') ");
	opt.AddOption("osi_points", "Show OSI road pointss (toggle during simulation by press 'y') ");
	opt.AddOption("osi_receiver_ip", "IP address where to send OSI UDP packages", "IP address");
#endif
	opt.AddOption("path", "Search path prefix for assets, e.g. OpenDRIVE files (multiple occurrences supported)", "path");
	opt.AddOption("record", "Record position data into a file for later replay", "filename");
	opt.AddOption("road_features", "Show OpenDRIVE road features (\"on\", \"off\"  (default)) (toggle during simulation by press 'o') ", "mode");
	opt.AddOption("save_generated_model", "Save generated 3D model (n/a when a scenegraph is loaded)");
	opt.AddOption("seed", "Specify seed number for random generator", "number");
	opt.AddOption("sensors", "Show sensor frustums (toggle during simulation by press 'r') ");
	opt.AddOption("server", "Launch server to receive state of external Ego simulator");
	opt.AddOption("threads", "Run viewer in a separate thread, parallel to scenario engine");
	opt.AddOption("trail_mode", "Show trail lines and/or dots (toggle key 'j') mode 0=None 1=lines 2=dots 3=both", "mode");
	opt.AddOption("version", "Show version and quit");

	exe_path_ = argv_[0];
	if (opt.ParseArgs(&argc_, argv_) != 0)
	{
		PrintUsage();
		return -2;
	}

	if (opt.GetOptionSet("version"))
	{
		Logger::Inst().LogVersion();
		return -2;
	}

	if (opt.GetOptionSet("help"))
	{
		PrintUsage();
		return -2;
	}

	if (opt.GetOptionSet("disable_stdout"))
	{
		Logger::Inst().SetCallback(0);
	}

	if (opt.GetOptionSet("disable_log"))
	{
		SE_Env::Inst().SetLogFilePath("");
		printf("Disable logfile\n");
	}
	else if (opt.IsOptionArgumentSet("logfile_path"))
	{
		arg_str = opt.GetOptionArg("logfile_path");
		SE_Env::Inst().SetLogFilePath(arg_str);
		if (arg_str.empty())
		{
			printf("Custom logfile path empty, disable logfile\n");
		}
		else
		{
			printf("Custom logfile path: %s\n", arg_str.c_str());
		}
	}
	Logger::Inst().OpenLogfile();
	Logger::Inst().LogVersion();

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

	if (opt.GetOptionArg("path") != "")
	{
		int counter = 0;
		while ((arg_str = opt.GetOptionArg("path", counter)) != "")
		{
			SE_Env::Inst().AddPath(arg_str);
			LOG("Added path %s", arg_str.c_str());
			counter++;
		}
	}

	if (opt.GetOptionSet("disable_controllers"))
	{
		disable_controllers_ = true;
		LOG("Disable entity controllers");
	}

	// Use specific seed for repeatable scenarios?
	if ((arg_str = opt.GetOptionArg("seed")) != "")
	{
		unsigned int seed = static_cast<unsigned int>(std::stoul(arg_str));
		LOG("Using specified seed %u", seed);
		SE_Env::Inst().SetSeed(seed);
	}
	else
	{
		LOG("Generated seed %u", SE_Env::Inst().GetSeed());
	}

	if (opt.GetOptionSet("disable_off_screen"))
	{
		SE_Env::Inst().SetDisableOffScreen(true);
	}

	if (opt.GetOptionSet("collision"))
	{
		SE_Env::Inst().SetCollisionDetection(true);
	}

	// Create scenario engine
	try
	{
		if ((arg_str = opt.GetOptionArg("osc")) != "")
		{
			scenarioEngine = new ScenarioEngine(arg_str, disable_controllers_);
			Logger::Inst().SetTimePtr(scenarioEngine->GetSimulationTimePtr());
			//Logger::Inst().SetTimePtr(scenarioEngine->GetTrueTimePtr());
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
			//Logger::Inst().SetTimePtr(scenarioEngine->GetTrueTimePtr());
		}
		else
		{
			LOG("Error: Missing required OpenSCENARIO filename argument or XML string");
			PrintUsage();

			return -1;
		}
	}
	catch (std::logic_error &e)
	{
		LOG(std::string("Exception: ").append(e.what()).c_str());
		return -1;
	}

	// Fetch scenario gateway and OpenDRIVE manager objects
	scenarioGateway = scenarioEngine->getScenarioGateway();
	odr_manager = scenarioEngine->getRoadManager();

#ifdef _USE_OSI
	osiReporter = new OSIReporter();

	if (opt.GetOptionSet("osi_receiver_ip"))
	{
		osiReporter->OpenSocket(opt.GetOptionArg("osi_receiver_ip"));
	}

	if (opt.GetOptionSet("osi_file"))
	{
		osiReporter->OpenOSIFile(opt.GetOptionArg("osi_file").c_str());
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
#endif  // USE_OSI

	// Initialize CSV logger for recording vehicle data
	if (opt.GetOptionSet("csv_logger"))
	{
		CSV_Log = &CSV_Logger::InstVehicleLog(scenarioEngine->getScenarioFilename(),
			(int)scenarioEngine->entities_.object_.size(), opt.GetOptionArg("csv_logger"));
		LOG("Log all vehicle data in csv file");
	}

	// Create a data file for later replay?
	if ((arg_str = opt.GetOptionArg("record")) != "")
	{
		LOG("Recording data to file %s", arg_str.c_str());
		scenarioGateway->RecordToFile(arg_str, scenarioEngine->getOdrFilename(), scenarioEngine->getSceneGraphFilename());
	}

	// Step scenario engine - zero time - just to reach and report init state of all vehicles
	ScenarioFrame(0.0, true);

	if (opt.IsInOriginalArgs("--window") || opt.IsInOriginalArgs("--borderless-window"))
	{
#ifdef _USE_OSG

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
			if (InitViewer() != 0)
			{
				LOG("Viewer initialization failed");
				return -1;
			}
		}

		// Decorate window border with application name and arguments
		viewer_->SetWindowTitleFromArgs(opt.GetOriginalArgs());

		viewer_->RegisterKeyEventCallback(ReportKeyEvent, this);
#else
		LOG("window requested, but esmini compiled without OSG capabilities");
#endif
	}
	else if (opt.GetOptionSet("capture_screen"))
	{
		PrintUsage();
		LOG_AND_QUIT("Capture screen requires a window to be specified!");
	}

	if (argc_ > 1)
	{
		opt.PrintArgs(argc_, argv_, "Unrecognized arguments:");
		PrintUsage();
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
	objCallback.push_back(cb);
}

void ScenarioPlayer::UpdateCSV_Log()
{
	//Flag for signalling end of data line, all vehicles reported
	bool isendline = false;

	//For each vehicle (entitity) stored in the ScenarioPlayer
	for (size_t i = 0; i < scenarioEngine->entities_.object_.size(); i++)
	{
		//Create a pointer to the object at position i in the entities vector
		Object* obj = scenarioEngine->entities_.object_[i];

		//Create a Position object for extracting this vehicles XYZ coordinates
		roadmanager::Position pos = obj->pos_;

		//Extract the String name of the object and store in a compatable const char array
		const char* name_ = &(*obj->name_.c_str());

		if ((i + 1) == scenarioEngine->entities_.object_.size())
		{
			isendline = true;
		}

		// Log the extracted data of ego vehicle and additonal scenario vehicles
		std::string collision_ids;
		if (SE_Env::Inst().GetCollisionDetection())
		{
			for (size_t j = 0; j < obj->collisions_.size(); j++)
			{
				collision_ids += std::to_string(obj->collisions_[j]->GetId()) + " ";
			}
		}
		CSV_Log->LogVehicleData(isendline, scenarioEngine->getSimulationTime(), name_,
			obj->id_, obj->speed_, obj->wheel_angle_, obj->wheel_rot_,
			pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetVelX(), pos.GetVelY(), pos.GetVelZ(), pos.GetAccX(), pos.GetAccY(), pos.GetAccZ(),
			pos.GetS(), pos.GetT(), pos.GetH(), pos.GetHRate(), pos.GetHRelative(), pos.GetHRelativeDrivingDirection(),
			pos.GetP(), pos.GetCurvature(), collision_ids.c_str());
	}
}

int ScenarioPlayer::GetNumberOfParameters()
{
	return scenarioEngine->scenarioReader->parameters.GetNumberOfParameters();
}

const char* ScenarioPlayer::GetParameterName(int index, OSCParameterDeclarations::ParameterType* type)
{
	return scenarioEngine->scenarioReader->parameters.GetParameterName(index, type);
}

int ScenarioPlayer::SetParameterValue(const char* name, const void* value)
{
	return scenarioEngine->scenarioReader->parameters.setParameterValue(name, value);
}

int ScenarioPlayer::GetParameterValue(const char* name, void* value)
{
	return scenarioEngine->scenarioReader->parameters.getParameterValue(name, value);
}

int ScenarioPlayer::GetParameterValueInt(const char* name, int& value)
{
	return scenarioEngine->scenarioReader->parameters.getParameterValueInt(name, value);
}

int ScenarioPlayer::GetParameterValueDouble(const char* name, double& value)
{
	return scenarioEngine->scenarioReader->parameters.getParameterValueDouble(name, value);
}

int ScenarioPlayer::GetParameterValueString(const char* name, const char*& value)
{
	return scenarioEngine->scenarioReader->parameters.getParameterValueString(name, value);
}

int ScenarioPlayer::GetParameterValueBool(const char* name, bool& value)
{
	return scenarioEngine->scenarioReader->parameters.getParameterValueBool(name, value);
}

int ScenarioPlayer::SetParameterValue(const char* name, int value)
{
	return scenarioEngine->scenarioReader->parameters.setParameterValue(name, value);
}

int ScenarioPlayer::SetParameterValue(const char* name, double value)
{
	return scenarioEngine->scenarioReader->parameters.setParameterValue(name, value);
}

int ScenarioPlayer::SetParameterValue(const char* name, const char* value)
{
	return scenarioEngine->scenarioReader->parameters.setParameterValue(name, value);
}

int ScenarioPlayer::SetParameterValue(const char* name, bool value)
{
	return scenarioEngine->scenarioReader->parameters.setParameterValue(name, value);
}


//todo
int ScenarioPlayer::GetNumberOfProperties(int index)
{
	return (int)scenarioEngine->entities_.object_[index]->properties_.property_.size();
}

const char* ScenarioPlayer::GetPropertyName(int index,int propertyIndex)
{
	return scenarioEngine->entities_.object_[index]->properties_.property_[propertyIndex].name_.c_str();
}

const char* ScenarioPlayer::GetPropertyValue(int index,int propertyIndex)
{
	return scenarioEngine->entities_.object_[index]->properties_.property_[propertyIndex].value_.c_str();
}

#ifdef _USE_OSG
	void ReportKeyEvent(viewer::KeyEvent* keyEvent, void* data)
	{
		ScenarioPlayer* player = (ScenarioPlayer*)data;
		for (size_t i = 0; i < player->scenarioEngine->GetScenarioReader()->controller_.size(); i++)
		{
			player->scenarioEngine->GetScenarioReader()->controller_[i]->ReportKeyEvent(keyEvent->key_, keyEvent->down_);
		}

		if (keyEvent->down_)
		{
			if (keyEvent->key_ == 'H')
			{
				puts(helpText);
			}
			else if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Space))
			{
				if (player->GetState() == ScenarioPlayer::PlayerState::PLAYER_STATE_PLAYING)
				{
					player->SetState(ScenarioPlayer::PlayerState::PLAYER_STATE_PAUSE);
				}
				else if (player->GetState() == ScenarioPlayer::PlayerState::PLAYER_STATE_PAUSE ||
					player->GetState() == ScenarioPlayer::PlayerState::PLAYER_STATE_STEP)
				{
					player->SetState(ScenarioPlayer::PlayerState::PLAYER_STATE_PLAYING);
				}
			}
			else if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Return))
			{
				player->SetState(ScenarioPlayer::PlayerState::PLAYER_STATE_STEP);
			}
		}
	}
#endif

