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

#include "PlayerServer.hpp"
#include "ScenarioEngine.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "Server.hpp"
#include "playerbase.hpp"
#include "helpText.hpp"
#include "OSCParameterDistribution.hpp"
#include "logger.hpp"
#include "Config.hpp"
#include "ConfigParser.hpp"

#ifdef _USE_OSG
#include "viewer.hpp"
#endif

#include "spdlog/fmt/fmt.h"

using namespace scenarioengine;

#define GHOST_HEADSTART 2.5
#define TRAIL_Z_OFFSET  0.02

#ifdef _USE_OSG

static ImageCallBack imageCallback = {0, 0};

void RegisterImageCallback(viewer::ImageCallbackFunc func, void* data)
{
    imageCallback.func = func;
    imageCallback.data = data;
}
#endif

ScenarioPlayer::ScenarioPlayer(int argc, char* argv[])
    : maxStepSize(0.1),
      minStepSize(0.001),
      argc_(argc),
      argv_(argv),
      state_(PlayerState::PLAYER_STATE_PLAYING)
{
    quit_request         = false;
    threads              = false;
    launch_server        = false;
    fixed_timestep_      = -1.0;
    osi_receiver_addr    = "";
    osi_updated_         = false;
    CSV_Log              = NULL;
    osiReporter          = NULL;
    disable_controllers_ = false;
    frame_counter_       = 0;
    scenarioEngine       = nullptr;
    osiReporter          = nullptr;
    viewer_              = nullptr;
    player_server_       = nullptr;

#ifdef _USE_OSG
    viewerState_ = ViewerState::VIEWER_STATE_NOT_STARTED;
#ifdef _USE_OSI
    OSISensorDetection = nullptr;
#endif  // _USE_OSI
#endif

    player_server_ = std::make_unique<PlayerServer>(this);
}

ScenarioPlayer::~ScenarioPlayer()
{
    if (launch_server)
    {
        StopServer();
    }

    if (SE_Env::Inst().GetOptions().GetOptionSet("player_server"))
    {
        player_server_->Stop();
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
    for (auto& s : sensor)
    {
        delete s;
    }
    TxtLogger::Inst().SetLoggerTime(0);
    if (scenarioEngine)
    {
        delete scenarioEngine;
        scenarioEngine = nullptr;
    }

#ifdef _USE_OSI
    if (osiReporter)
    {
        delete osiReporter;
        osiReporter = nullptr;
    }
#endif  // _USE_OSI

    SE_Env::Inst().GetOptions().Reset();
}

void ScenarioPlayer::SetOSIFileStatus(bool is_on, const char* filename)
{
    (void)is_on;
    (void)filename;
#ifdef _USE_OSI
    if (osiReporter)
    {
        if (is_on)
        {
            if (filename == nullptr || !strcmp(filename, ""))
            {
                filename = DEFAULT_OSI_TRACE_FILENAME;
            }
            std::string strFileName = esmini::common::ValidateAndCreateFilePath(filename, DEFAULT_OSI_TRACE_FILENAME, "osi");
            filename                = strFileName.c_str();
            if (OSCParameterDistribution::Inst().GetNumPermutations() > 0)
            {
                osiReporter->OpenOSIFile(OSCParameterDistribution::Inst().AddInfoToFilepath(filename).c_str());
            }
            else
            {
                osiReporter->OpenOSIFile(filename);
            }
        }
        else
        {
            osiReporter->CloseOSIFile();
        }
    }
#endif  // _USE_OSI
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

            if (viewer_->GetQuitRequest())
            {
                SetQuitRequest(true);
                CloseViewer();
            }
        }
#endif
    }
}

int ScenarioPlayer::Frame(double timestep_s, bool server_mode)
{
    static bool messageShown  = false;
    int         retval        = 0;
    double      ghost_solo_dt = 0.05;

    if (!IsPaused() || server_mode)
    {
#ifdef _USE_OSI
        if (osiReporter != nullptr)
        {
            osiReporter->SetUpdated(false);
        }
#endif
        scenarioEngine->mutex_.Lock();
        retval = ScenarioFrame(timestep_s, true);

        if (SE_Env::Inst().GetGhostMode() != GhostMode::NORMAL)
        {
            while (retval == 0 && SE_Env::Inst().GetGhostMode() != GhostMode::NORMAL && !IsQuitRequested())
            {
                Draw();
                if (!IsPaused() && !IsQuitRequested())
                {
                    retval = ScenarioFrame(ghost_solo_dt, false);
                }
            }
        }

        if (retval == 0)
        {
            ScenarioPostFrame();
        }

        if (GetState() == PlayerState::PLAYER_STATE_STEP)
        {
            SetState(PlayerState::PLAYER_STATE_PAUSE);
        }
        scenarioEngine->mutex_.Unlock();
    }

    if (!server_mode)
    {
        Draw();

        if (scenarioEngine->getSimulationTime() > 3600 && !messageShown)
        {
            LOG_INFO("Info: Simulation time > 1 hour. Put a stopTrigger for automatic ending");
            messageShown = true;
        }

        if (player_server_)
        {
            player_server_->Step();
        }
    }

    return retval;
}

int ScenarioPlayer::Frame(bool server_mode)
{
    double dt;

    if ((dt = GetFixedTimestep()) < 0.0)
    {
        static __int64 time_stamp = 0;
        return Frame(SE_getSimTimeStep(time_stamp, minStepSize, maxStepSize), server_mode);
    }
    else
    {
        return Frame(dt, server_mode);
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

        if (SE_Env::Inst().GetGhostMode() != GhostMode::RESTART)
        {
            scenarioGateway->WriteStatesToFile();

            if (CSV_Log)
            {
                UpdateCSV_Log();
            }
        }

        if (keyframe)
        {
            frame_counter_++;
        }
    }

    scenarioEngine->UpdateGhostMode();

    mutex.Unlock();

    quit_request |= scenarioEngine->GetQuitFlag();

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
        // Update OSI info
        if (osiReporter->GetOSIFrequency() > 0)
        {
            osiReporter->ReportSensors(sensor);

            osiReporter->UpdateOSIGroundTruth(scenarioGateway->objectState_);

            osiReporter->UpdateOSITrafficCommand();
        }
    }
#endif  // _USE_OSI

    mutex.Unlock();
}

#ifdef _USE_OSG
void ScenarioPlayer::ViewerFrame(bool init)
{
    if (viewer_ == nullptr)
    {
        return;
    }

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
            viewer_->RemoveCar(static_cast<int>(i));
            i--;  // test same object again against next in viewer list
        }
    }

    // Add missing cars
    while (viewer_->entities_.size() < scenarioEngine->entities_.object_.size())
    {
        Object* obj = scenarioEngine->entities_.object_[viewer_->entities_.size()];
        viewer_->AddEntityModel(viewer_->CreateEntityModel(obj->model3d_,
                                                           trail_color,
                                                           viewer::EntityModel::EntityType::VEHICLE,
                                                           false,
                                                           obj->name_,
                                                           &obj->boundingbox_,
                                                           obj->scaleMode_));

        // Connect callback for setting transparency
        viewer::VisibilityCallback* cb = new viewer::VisibilityCallback(obj, viewer_->entities_.back());
        viewer_->entities_.back()->txNode_->setUpdateCallback(cb);

        InitVehicleModel(obj, static_cast<viewer::CarModel*>(viewer_->entities_.back()));
    }

    // remove obsolete cars
    while (viewer_->entities_.size() > scenarioEngine->entities_.object_.size())
    {
        if (viewer_->entities_.back()->trajectory_->activeRMTrajectory_)
        {
            viewer_->entities_.back()->trajectory_->Disable();
        }
        viewer_->RemoveCar(static_cast<int>(viewer_->entities_.size() - 1));
    }

    if (!init)
    {
        // Visualize entities
        for (size_t i = 0; i < scenarioEngine->entities_.object_.size(); i++)
        {
            viewer::EntityModel* entity = viewer_->entities_[i];
            Object*              obj    = scenarioEngine->entities_.object_[i];

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
            else if (entity->routewaypoints_->group_all_wp_->getNumChildren() && obj->pos_.GetRoute() == nullptr)
            {
                entity->routewaypoints_->SetWayPoints(nullptr);
            }

            if (entity->IsMoving())
            {
                if (entity->IsVehicle())
                {
                    viewer::CarModel* car = static_cast<viewer::CarModel*>(entity);
                    car->UpdateWheels(obj->wheel_angle_, obj->wheel_rot_);
                }

                viewer::MovingModel* mov = static_cast<viewer::MovingModel*>(entity);

                if (mov->steering_sensor_ && mov->steering_sensor_->IsVisible())
                {
                    viewer_->SensorSetPivotPos(mov->steering_sensor_, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ());
                    viewer_->SensorSetTargetPos(mov->steering_sensor_, obj->sensor_pos_[0], obj->sensor_pos_[1], obj->sensor_pos_[2]);
                    viewer_->UpdateSensor(mov->steering_sensor_);
                }
                if (mov->trail_sensor_ && mov->steering_sensor_->IsVisible())
                {
                    viewer_->SensorSetPivotPos(mov->trail_sensor_, obj->trail_closest_pos_.x, obj->trail_closest_pos_.y, obj->trail_closest_pos_.z);
                    viewer_->SensorSetTargetPos(mov->trail_sensor_, obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ());
                    viewer_->UpdateSensor(mov->trail_sensor_);
                }

                if (odr_manager->GetNumOfRoads() > 0 && mov->road_sensor_)
                {
                    mov->ShowRouteSensor(obj->pos_.GetRoute() ? true : false);
                    viewer_->UpdateRoadSensors(mov->road_sensor_, mov->route_sensor_, mov->lane_sensor_, &obj->pos_);
                }
            }

            if (entity->trail_->pline_vertex_data_->size() > obj->trail_.GetNumberOfVertices())
            {
                // Reset the trail, probably there has been a ghost restart
                entity->trail_->Reset();
                for (unsigned int j = 0; j < obj->trail_.GetNumberOfVertices(); j++)
                {
                    entity->trail_->AddPoint(obj->trail_.vertex_[j].x,
                                             obj->trail_.vertex_[j].y,
                                             obj->trail_.vertex_[j].z + (obj->GetId() + 1) * TRAIL_Z_OFFSET);
                }
            }

            if (obj->trail_.GetNumberOfVertices() > entity->trail_->pline_vertex_data_->size())
            {
                entity->trail_->AddPoint(obj->pos_.GetX(), obj->pos_.GetY(), obj->pos_.GetZ() + (obj->GetId() + 1) * TRAIL_Z_OFFSET);
            }

            // on screen text following each entity
            snprintf(entity->on_screen_info_.string_,
                     sizeof(entity->on_screen_info_.string_),
                     " %s (%d) %.2fm\n %.2fkm/h road %d lane %d/%.2f s %.2f\n x %.2f y %.2f hdg %.2f\n osi x %.2f y %.2f \n|",
                     obj->name_.c_str(),
                     obj->GetId(),
                     obj->odometer_,
                     3.6 * obj->speed_,
                     obj->pos_.GetTrackId(),
                     obj->pos_.GetLaneId(),
                     fabs(obj->pos_.GetOffset()) < SMALL_NUMBER ? 0 : obj->pos_.GetOffset(),
                     obj->pos_.GetS(),
                     obj->pos_.GetX(),
                     obj->pos_.GetY(),
                     obj->pos_.GetH(),
                     obj->pos_.GetX() + static_cast<double>(obj->boundingbox_.center_.x_) * cos(obj->pos_.GetH()),
                     obj->pos_.GetY() + static_cast<double>(obj->boundingbox_.center_.x_) * sin(obj->pos_.GetH()));
            entity->on_screen_info_.osg_text_->setText(entity->on_screen_info_.string_);
        }

        for (size_t i = 0; i < sensorFrustum.size(); i++)
        {
            sensorFrustum[i]->Update();
        }

        // Update info text
        static char str_buf[128];
        if (viewer_->currentCarInFocus_ >= 0 && static_cast<unsigned int>(viewer_->currentCarInFocus_) < viewer_->entities_.size())
        {
            Object* obj = scenarioEngine->entities_.object_[static_cast<unsigned int>(viewer_->currentCarInFocus_)];
            snprintf(str_buf,
                     sizeof(str_buf),
                     "%.2fs entity[%d]: %s (%d) %.2fkm/h %.2fm (%d, %d, %.2f, %.2f) / (%.2f, %.2f %.2f)",
                     scenarioEngine->getSimulationTime(),
                     viewer_->currentCarInFocus_,
                     obj->name_.c_str(),
                     obj->GetId(),
                     3.6 * obj->speed_,
                     obj->odometer_,
                     obj->pos_.GetTrackId(),
                     obj->pos_.GetLaneId(),
                     fabs(obj->pos_.GetOffset()) < SMALL_NUMBER ? 0 : obj->pos_.GetOffset(),
                     obj->pos_.GetS(),
                     obj->pos_.GetX(),
                     obj->pos_.GetY(),
                     obj->pos_.GetH());
        }
        else
        {
            snprintf(str_buf, sizeof(str_buf), "%.2fs No entity in focus...", scenarioEngine->getSimulationTime());
        }
        viewer_->SetInfoText(str_buf);
    }
    mutex.Unlock();

    if (!init)
    {
        viewer_->Frame(scenarioEngine->getSimulationTime());
    }
}

int ScenarioPlayer::SaveImagesToRAM(bool state)
{
    SE_Env::Inst().SaveImagesToRAM(state);

    if (viewer_)
    {
        viewer_->imageMutex.Lock();
        viewer_->UpdateOffScreenStatus();
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

    if (viewer_ && viewer_->IsOffScreenRequested())
    {
        viewer_->renderSemaphore.Wait();  // Wait until rendering is done

        if (viewer_->capturedImage_.data == nullptr)
        {
            LOG_ERROR("FetchCapturedImagePtr Error: No image data");
            return nullptr;
        }

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
                    img.data = static_cast<unsigned char*>(
                        malloc(static_cast<unsigned int>(tmpImg->pixelSize * tmpImg->height * tmpImg->width) * sizeof(unsigned char)));
                }
            }
            if (img.data != nullptr)
            {
                img.height      = tmpImg->height;
                img.width       = tmpImg->width;
                img.pixelSize   = tmpImg->pixelSize;
                img.pixelFormat = tmpImg->pixelFormat;
                memcpy(img.data, tmpImg->data, static_cast<unsigned int>(tmpImg->pixelSize * tmpImg->height * tmpImg->width) * sizeof(unsigned char));
            }
        }

        viewer_->imageMutex.Unlock();

        return &img;
    }

    return nullptr;
}

int ScenarioPlayer::AddCustomCamera(double x, double y, double z, double h, double p, bool fixed_pos)
{
    if (viewer_)
    {
        viewer_->AddCustomCamera(x, y, z, h, p, fixed_pos);
        viewer_->SetCameraMode(-1);  // activate last camera which is the one just added
        return viewer_->GetNumberOfCameraModes() - 1;
    }

    return -1;
}

int ScenarioPlayer::AddCustomCamera(double x, double y, double z, bool fixed_pos)
{
    if (viewer_)
    {
        viewer_->AddCustomCamera(x, y, z, fixed_pos);
        viewer_->SetCameraMode(-1);  // activate last camera which is the one just added
        return viewer_->GetNumberOfCameraModes() - 1;
    }

    return -1;
}

int ScenarioPlayer::AddCustomFixedTopCamera(double x, double y, double z, double rot)
{
    if (viewer_)
    {
        viewer_->AddCustomFixedTopCamera(x, y, z, rot);
        viewer_->SetCameraMode(-1);  // activate last camera which is the one just added
        return viewer_->GetNumberOfCameraModes() - 1;
    }

    return -1;
}

int ScenarioPlayer::AddCustomLightSource(double x, double y, double z, double intensity)
{
    if (viewer_)
    {
        return viewer_->AddCustomLightSource(x, y, z, intensity);
    }

    return -1;
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

    SE_Options& opt = SE_Env::Inst().GetOptions();

    // osg::ArgumentParser modifies given args array so we copy it for safe modification
    std::vector<char*> args      = {argv_, std::next(argv_, argc_)};
    int                arg_count = static_cast<int>(args.size());

    // Create viewer
    osg::ArgumentParser arguments(&arg_count, args.data());
    viewer_ = new viewer::Viewer(roadmanager::Position::GetOpenDrive(),
                                 scenarioEngine->getSceneGraphFilename().c_str(),
                                 scenarioEngine->getScenarioFilename().c_str(),
                                 exe_path_.c_str(),
                                 arguments,
                                 &opt);

    if (viewer_->osgViewer_ == 0)
    {
        viewerState_ = ViewerState::VIEWER_STATE_FAILED;
        return -1;
    }

    viewer_->osgViewer_->setKeyEventSetsDone(0);  // Disable default Escape key event handler, take over control

    if ((arg_str = opt.GetOptionArg("info_text")) != "")
    {
        int mask = strtoi(arg_str);
        if (mask < 0 || mask > 3)
        {
            LOG_ERROR_AND_QUIT("Invalid on-screen info mode {}. Valid range is 0-3", mask);
        }
        viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_INFO | viewer::NodeMask::NODE_MASK_INFO_PER_OBJ,
                                 mask * viewer::NodeMask::NODE_MASK_INFO);
    }

    viewer_->RegisterImageCallback(imageCallback.func, imageCallback.data);

    if (opt.GetOptionSet("capture_screen"))
    {
        LOG_INFO("Activate continuous screen capture");
        viewer_->SaveImagesToFile(-1);
    }

    if ((arg_str = opt.GetOptionArg("trail_mode")) != "")
    {
        int mask = strtoi(arg_str);
        if (mask < 0 || mask > 3)
        {
            LOG_ERROR_AND_QUIT("Invalid trail_mode {}. Valid range is 0-3", mask);
        }
        viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_TRAIL_LINES | viewer::NodeMask::NODE_MASK_TRAIL_DOTS,
                                 mask * viewer::NodeMask::NODE_MASK_TRAIL_LINES);
    }

    if (opt.GetOptionSet("hide_trajectories"))
    {
        LOG_INFO("Hide trajectories");
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
        LOG_INFO("Disable route waypoint visualization");
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
            LOG_INFO("Unsupported camera mode: {} - using default (orbit)", arg_str);
        }
    }

    if (opt.GetOptionSet("custom_camera") == true)
    {
        int counter = 0;

        while ((arg_str = opt.GetOptionArg("custom_camera", counter)) != "")
        {
            const auto splitted = SplitString(arg_str, ',');

            if (splitted.size() == 3)
            {
                AddCustomCamera(strtod(splitted[0]), strtod(splitted[1]), strtod(splitted[2]), false);
                LOG_INFO("Created custom fixed camera {} ({}, {}, {})", counter, splitted[0], splitted[1], splitted[2]);
            }
            else if (splitted.size() == 5)
            {
                AddCustomCamera(strtod(splitted[0]), strtod(splitted[1]), strtod(splitted[2]), strtod(splitted[3]), strtod(splitted[4]), false);
                LOG_INFO("Created custom fixed camera {} ({}, {}, {}, {}, {})",
                         counter,
                         splitted[0],
                         splitted[1],
                         splitted[2],
                         splitted[3],
                         splitted[4]);
            }
            else
            {
                LOG_ERROR_AND_QUIT("Expected custom_camera <x,y,z>[,h,p]. Got {} values instead of 3 or 5.", splitted.size());
            }

            counter++;
        }
    }

    if (opt.GetOptionSet("custom_fixed_camera") == true)
    {
        int counter = 0;

        while ((arg_str = opt.GetOptionArg("custom_fixed_camera", counter)) != "")
        {
            const auto splitted = SplitString(arg_str, ',');

            if (splitted.size() == 3)
            {
                AddCustomCamera(strtod(splitted[0]), strtod(splitted[1]), strtod(splitted[2]), true);
                LOG_INFO("Created custom fixed camera {} ({}, {}, {})", counter, splitted[0], splitted[1], splitted[2]);
            }
            else if (splitted.size() == 5)
            {
                AddCustomCamera(strtod(splitted[0]), strtod(splitted[1]), strtod(splitted[2]), strtod(splitted[3]), strtod(splitted[4]), true);
                LOG_INFO("Created custom fixed camera {} ({}, {}, {}, {}, {})",
                         counter,
                         splitted[0],
                         splitted[1],
                         splitted[2],
                         splitted[3],
                         splitted[4]);
            }
            else
            {
                LOG_ERROR_AND_QUIT("Expected custom_fixed_camera <x,y,z>[,h,p]. Got {} values instead of 3 or 5.", splitted.size());
            }
            counter++;
        }
    }

    if (opt.GetOptionSet("custom_fixed_top_camera") == true)
    {
        int counter = 0;

        while ((arg_str = opt.GetOptionArg("custom_fixed_top_camera", counter)) != "")
        {
            const auto splitted = SplitString(arg_str, ',');
            if (splitted.size() != 4)
            {
                LOG_ERROR_AND_QUIT("Expected custom_fixed_top_camera <x,y,z,rot>. Got {} values instead of 4", splitted.size());
            }
            AddCustomFixedTopCamera(strtod(splitted[0]), strtod(splitted[1]), strtod(splitted[2]), strtod(splitted[3]));

            LOG_INFO("Created custom fixed top camera {} ({}, {}, {}, {})", counter, splitted[0], splitted[1], splitted[2], splitted[3]);
            counter++;
        }
    }

    if (opt.GetOptionSet("custom_light") == true)
    {
        int counter      = 0;
        int lightCounter = 0;

        while ((arg_str = opt.GetOptionArg("custom_light", counter)) != "")
        {
            const auto splitted = SplitString(arg_str, ',');
            if (splitted.size() != 4)
            {
                LOG_ERROR_AND_QUIT("Expected custom_light <x,y,z,intensity>. Got {} values instead of 4", splitted.size());
            }
            if (AddCustomLightSource(strtod(splitted[0]), strtod(splitted[1]), strtod(splitted[2]), strtod(splitted[3])) == 0)
            {
                LOG_INFO("Created custom light source {} ({}, {}, {}, {})", lightCounter, splitted[0], splitted[1], splitted[2], splitted[3]);
                lightCounter++;
            }
            else
            {
                LOG_INFO("Max nr custom lights ({}) reached. Ignoring ({:.2f}, {:.2f}, {:.2f}, {:.2f})",
                         lightCounter,
                         splitted[0],
                         splitted[1],
                         splitted[2],
                         splitted[3]);
            }
            counter++;
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
        Object*   obj = scenarioEngine->entities_.object_[i];

        // Create trajectory/trails for all entities
        if (obj->GetId() == 0)
        {
            trail_color.set(color_white[0], color_white[1], color_white[2], 1.0);
        }
        else if (obj->IsGhost())
        {
            trail_color.set(color_black[0], color_black[1], color_black[2], 1.0);
        }
        else if (obj->IsAnyAssignedControllerOfType(Controller::Type::CONTROLLER_TYPE_EXTERNAL))
        {
            trail_color.set(color_yellow[0], color_yellow[1], color_yellow[2], 1.0);
        }
        else
        {
            trail_color.set(color_red[0], color_red[1], color_red[2], 1.0);
        }

        //  Create vehicles for visualization
        bool road_sensor = false;
        if (obj->GetGhost() || obj->IsControllerActiveOnAnyOfDomains(static_cast<unsigned int>(ControlDomains::DOMAIN_LAT_AND_LONG)))
        {
            road_sensor = true;
        }

        if (viewer_->AddEntityModel(viewer_->CreateEntityModel(obj->model3d_,
                                                               trail_color,
                                                               obj->type_ == Object::Type::VEHICLE      ? viewer::EntityModel::EntityType::VEHICLE
                                                               : obj->type_ == Object::Type::PEDESTRIAN ? viewer::EntityModel::EntityType::MOVING
                                                                                                        : viewer::EntityModel::EntityType::ENTITY,
                                                               road_sensor,
                                                               obj->name_,
                                                               &obj->boundingbox_,
                                                               obj->scaleMode_)) != 0)
        {
            CloseViewer();
            return -1;
        }

        // Connect callback for setting transparency
        viewer::VisibilityCallback* cb = new viewer::VisibilityCallback(obj, viewer_->entities_.back());
        viewer_->entities_.back()->txNode_->setUpdateCallback(cb);

        if (viewer_->entities_.back()->IsVehicle())
        {
            InitVehicleModel(obj, static_cast<viewer::CarModel*>(viewer_->entities_.back()));
        }
    }

    // Choose vehicle to look at initially (switch with 'Tab')
    if (opt.GetOptionSet("follow_object"))
    {
        LOG_INFO("Follow object {}", strtoi(opt.GetOptionArg("follow_object")));
        viewer_->SetVehicleInFocus(strtoi(opt.GetOptionArg("follow_object")));
    }
    else
    {
        viewer_->SetVehicleInFocus(0);
    }

    for (size_t i = 0; i < scenarioEngine->entities_.object_.size(); i++)
    {
        Object* obj = scenarioEngine->entities_.object_[i];

        if (obj->IsAnyAssignedControllerOfType(Controller::Type::CONTROLLER_TYPE_INTERACTIVE) ||
            obj->IsAnyAssignedControllerOfType(Controller::Type::CONTROLLER_TYPE_EXTERNAL) ||
            obj->IsAnyAssignedControllerOfType(Controller::Type::CONTROLLER_TYPE_FOLLOW_GHOST))
        {
            if (viewer_->GetEntityInFocus() == 0)
            {
                // Focus on first vehicle of specified types
                viewer_->SetVehicleInFocus(static_cast<int>(i));
            }
        }
    }

    // Decorate window border with application name and arguments
    viewer_->SetWindowTitleFromArgs(opt.GetOriginalArgs());
    viewer_->RegisterKeyEventCallback(ReportKeyEvent, this);

    viewerState_ = ViewerState::VIEWER_STATE_STARTED;

    return 0;
}

void viewer_thread(void* args)
{
    ScenarioPlayer* player = static_cast<ScenarioPlayer*>(args);

    if (player->InitViewer() != 0)
    {
        return;
    }

    player->viewer_init_semaphore.Release();
    player->player_init_semaphore.Wait();

    while (player->viewer_->GetQuitRequest() == 0)
    {
        player->ViewerFrame();
    }

    player->viewer_->renderSemaphore.Wait();
    player->SetQuitRequest(true);
    player->CloseViewer();
}

#endif

void ScenarioPlayer::InitControllersPostPlayer()
{
    if (!scenarioEngine->GetDisableControllersFlag())
    {
        //  Create relation between controllers and player
        for (size_t i = 0; i < scenarioEngine->entities_.object_.size(); i++)
        {
            if (scenarioEngine->entities_.object_[i]->controllers_.size() > 0)
            {
                for (auto ctrl : scenarioEngine->entities_.object_[i]->controllers_)
                {
                    ctrl->SetPlayer(this);
                    ctrl->InitPostPlayer();
                }
            }
        }
    }
}

int ScenarioPlayer::AddObjectSensor(Object* obj,
                                    double  pos_x,
                                    double  pos_y,
                                    double  pos_z,
                                    double  heading,
                                    double  near_dist,
                                    double  far_dist,
                                    double  fovH,
                                    int     maxObj)
{
    if (obj == nullptr)
    {
        return -1;
    }

    sensor.push_back(new ObjectSensor(&scenarioEngine->entities_, obj, pos_x, pos_y, pos_z, heading, near_dist, far_dist, fovH, maxObj));

#ifdef _USE_OSG
    if (viewer_)
    {
        int object_index = scenarioEngine->entities_.GetObjectIdxById(obj->GetId());
        if (object_index >= 0)
        {
            mutex.Lock();
            sensorFrustum.push_back(
                new viewer::SensorViewFrustum(viewer_, sensor.back(), viewer_->entities_[static_cast<unsigned int>(object_index)]->txNode_));
            mutex.Unlock();
        }
    }
#endif

    return static_cast<int>(sensor.size()) - 1;
}

int ScenarioPlayer::GetNumberOfObjectSensors() const
{
    return static_cast<int>(sensor.size());
}

int ScenarioPlayer::GetNumberOfSensorsAttachedToObject(const Object* obj) const
{
    if (obj == nullptr)
    {
        return -1;
    }

    int counter = 0;
    for (size_t i = 0; i < sensor.size(); i++)
    {
        if (sensor[i]->host_ == obj)
        {
            counter++;
        }
    }

    return counter;
}

#ifdef _USE_OSG
void ScenarioPlayer::InitVehicleModel(Object* obj, viewer::CarModel* model)
{
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
    if (obj->GetGhost())
    {
        if (odr_manager->GetNumOfRoads() > 0)
        {
            model->trail_sensor_ = viewer_->CreateSensor(color_red, true, false, 0.4, 3);
        }
    }
    else if (obj->IsGhost())
    {
        obj->SetVisibilityMask(obj->visibilityMask_ &= ~(Object::Visibility::SENSORS));
    }

    viewer_->entities_.back()->routewaypoints_->SetWayPoints(obj->pos_.GetRoute());
}
#endif

void ScenarioPlayer::AddOSIDetection(int object_index)
{
    (void)object_index;
#ifdef _USE_OSG
    if (viewer_)
    {
#ifdef _USE_OSI
        if (!OSISensorDetection)
        {
            mutex.Lock();
            OSISensorDetection = new viewer::OSISensorDetection(viewer_->entities_[static_cast<unsigned int>(object_index)]->txNode_);
            mutex.Unlock();
        }
#endif  // _USE_OSI
    }
#endif
}

void ScenarioPlayer::SteeringSensorSetVisible(int object_index, bool value)
{
    (void)object_index;
    (void)value;
#ifdef _USE_OSG
    if (viewer_ == nullptr)
    {
        return;
    }

    int obj_index = scenarioEngine->entities_.GetObjectIdxById(object_index);
    if (obj_index >= 0)
    {
        viewer::EntityModel* m = viewer_->entities_[static_cast<unsigned int>(obj_index)];
        if (m->IsMoving())
        {
            if (value == true)
            {
                reinterpret_cast<viewer::MovingModel*>(m)->steering_sensor_->Show();
            }
            else
            {
                reinterpret_cast<viewer::MovingModel*>(m)->steering_sensor_->Hide();
            }
        }
    }
#endif
}

void ScenarioPlayer::ShowObjectSensors(bool mode)
{
    (void)mode;
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
    SE_Env::Inst().GetOptions().PrintUsage();
#ifdef _USE_OSG
    PrintOSGUsage();
#endif
}

int ScenarioPlayer::Init()
{
    std::string arg_str;

    SE_Options& opt = SE_Env::Inst().GetOptions();

    // use an ArgumentParser object to manage the program arguments.
    opt.AddOption("osc", "OpenSCENARIO filename (required) - if path includes spaces, enclose with \"\"", "filename");
    opt.AddOption("aa_mode", "Anti-alias mode=number of multisamples (subsamples, 0=off)", "mode", "4");
    opt.AddOption("align_routepositions", "Align t-axis of route positions to the direction of the route");
    opt.AddOption("bounding_boxes", "Show entities as bounding boxes. Toggle key ','");
    opt.AddOption("capture_screen", "Continuous screen capture. Warning: Many jpeg files will be created");
    opt.AddOption("camera_mode",
                  "Initial camera mode (\"orbit\", \"fixed\", \"flex\", \"flex-orbit\", \"top\", \"driver\", \"custom\"). Toggle key 'k'",
                  "mode",
                  "orbit",
                  true);
    opt.AddOption("csv_logger", "Log data for each vehicle in ASCII csv format", "csv_filename", "log.csv");
    opt.AddOption("collision", "Enable global collision detection, potentially reducing performance");
    opt.AddOption(CONFIG_FILE_OPTION_NAME, "Configuration file path/filename, e.g. \"../my_config.txt\"", "path", DEFAULT_CONFIG_FILE, false, false);
    opt.AddOption("custom_camera", "Additional custom camera position <x,y,z>[,h,p]", "position", "", false, false);
    opt.AddOption("custom_fixed_camera",
                  "Additional custom fixed camera position <x,y,z>[,h,p]",
                  "position and optional orientation",
                  "",
                  false,
                  false);
    opt.AddOption("custom_fixed_top_camera", "Additional custom top camera <x,y,z,rot>", "position and rotation", "", false, false);
    opt.AddOption("custom_light",
                  "Additional custom light source <x,y,z,intensity> intensity range 0..1",
                  "position and intensity",
                  "",
                  false,
                  false);
    opt.AddOption("disable_controllers", "Disable controllers");
    opt.AddOption("disable_log", "Prevent logfile from being created");
    opt.AddOption("disable_stdout", "Prevent messages to stdout");
    opt.AddOption("enforce_generate_model", "Generate road 3D model even if SceneGraphFile is specified");
    opt.AddOption("fixed_timestep", "Run simulation decoupled from realtime, with specified timesteps", "timestep");
    opt.AddOption("follow_object", "Set index of initial object for camera to follow (change with Tab/shift-Tab)", "index", "0", true);
    opt.AddOption("generate_no_road_objects", "Do not generate any OpenDRIVE road objects (e.g. when part of referred 3D model)");
    opt.AddOption("generate_without_textures", "Do not apply textures on any generated road model (set colors instead as for missing textures)");
    opt.AddOption("ground_plane", "Add a large flat ground surface");
    opt.AddOption("headless", "Run without viewer window");
    opt.AddOption("help", "Show this help message (-h works as well)");
    opt.AddOption("hide_route_waypoints", "Disable route waypoint visualization. Toggle key 'R'");
    opt.AddOption("hide_trajectories", "Hide trajectories from start. Toggle key 'n'");
    opt.AddOption("ignore_heading_for_traj_motion", "Ignore heading when deciding motion direction along trajectory");
    opt.AddOption("ignore_odr_offset", "Ignore any offset specified in the OpenDRIVE file header");
    opt.AddOption("ignore_z", "Ignore provided z values from OSC file and place vehicle relative to road");
    opt.AddOption("ignore_p", "Ignore provided pitch values from OSC file and place vehicle relative to road");
    opt.AddOption("ignore_r", "Ignore provided roll values from OSC file and place vehicle relative to road");
    opt.AddOption("info_text", "Show on-screen info text. Modes: 0=None 1=current 2=per_object 3=both. Toggle key 'i'", "mode", "1", true);
    opt.AddOption("log_append", "Log all scenarios in the same txt file");
    opt.AddOption("logfile_path", "Logfile path/filename, e.g. \"../my_log.txt\"", "path", LOG_FILENAME, true);
    opt.AddOption("log_meta_data", "Log file name, function name and line number");
    opt.AddOption("log_level", "Log level debug, info, warn, error", "mode", "info", true);
    opt.AddOption("log_only_modules", "Log from only these modules. Overrides log_skip_modules. See User guide for more info", "modulename(s)");
    opt.AddOption("log_skip_modules",
                  "Skip log from these modules, all remaining modules will be logged. See User guide for more info",
                  "modulename(s)");
    opt.AddOption("osc_str", "OpenSCENARIO XML string", "string");
    opt.AddOption("osg_screenshot_event_handler", "Revert to OSG default jpg images ('c'/'C' keys handler)");
#ifdef _USE_OSI
    opt.AddOption("osi_crop_dynamic", "Crop the dynamic osi data around the given object id with given radius", "id,radius", "", false, false);
    opt.AddOption("osi_exclude_ghost", "Excludes ghost from osi dynamic osi ground truth");
    opt.AddOption("osi_file", "Save osi trace file", "filename", DEFAULT_OSI_TRACE_FILENAME);
    opt.AddOption("osi_freq", "Decrease OSI file entries, e.g. --osi_freq 2 -> OSI written every two simulation steps", "frequency");
    opt.AddOption("osi_lines", "Show OSI road lines. Toggle key 'u'");
    opt.AddOption("osi_points", "Show OSI road points. Toggle key 'y'");
    opt.AddOption("osi_receiver_ip", "IP address where to send OSI UDP packages", "IP address", "127.0.0.1");
    opt.AddOption("osi_static_reporting",
                  "Decide how the static data should be reported, 0=Default (first frame), 1=API (expose on API) 2=API_AND_LOG (Always log)",
                  "mode",
                  "0");
#endif
    opt.AddOption("param_dist", "Run variations of the scenario according to specified parameter distribution file", "filename");
    opt.AddOption("param_permutation", "Run specific permutation of parameter distribution, index in range (0 .. NumberOfPermutations-1)", "index");
    opt.AddOption("pause", "Pause simulation after initialization");
    opt.AddOption("path", "Search path prefix for assets, e.g. OpenDRIVE files.", "path", "", false, false);
    opt.AddOption("player_server", "Launch UDP server for action/command injection");
#ifdef _USE_IMPLOT
    opt.AddOption("plot", "Show window with line-plots of interesting data. Modes: asynchronous, synchronous", "mode", "asynchronous");
#endif
    opt.AddOption("pline_interpolation", "Interpolate orientation (\"segment\", \"corner\", \"off\")", "mode");
    opt.AddOption("record", "Record position data into a file for later replay", "filename", DAT_FILENAME);
    opt.AddOption("road_features", "Show OpenDRIVE road features. Modes: on, off. Toggle key 'o'", "mode", "on");
    opt.AddOption("return_nr_permutations", "Return number of permutations without executing the scenario (-1 = error)");
    opt.AddOption("save_generated_model", "Save generated 3D model (n/a when a scenegraph is loaded)");
    opt.AddOption("save_xosc",
                  "Save OpenSCENARIO file with any populated parameter values (from distribution). Modes: quit, continue.",
                  "mode",
                  "continue",
                  false);
    opt.AddOption("seed", "Specify seed number for random generator", "number");
    opt.AddOption("sensors", "Show sensor frustums. Toggle key 'r'");
    opt.AddOption("server", "Launch server to receive state of external Ego simulator");
    opt.AddOption("text_scale", "Scale screen overlay text", "size factor", "1.0", true);
    opt.AddOption("threads", "Run viewer in a separate thread, parallel to scenario engine");
    opt.AddOption("trail_mode", "Show trail lines and/or dots. Modes: 0=None 1=lines 2=dots 3=both. Toggle key 'j'", "mode", "0");
    opt.AddOption("traj_filter", "Simple filter merging close points. Set 0.0 to disable", "radius", "0.1", true);
    opt.AddOption("use_signs_in_external_model", "When external scenegraph 3D model is loaded, skip creating signs from OpenDRIVE");
    opt.AddOption("version", "Show version and quit");

    if (int ret = OnRequestShowHelpOrVersion(argc_, argv_, opt); ret > 0)
    {
        return ret;
    }

    exe_path_ = argv_[0];
    SE_Env::Inst().AddPath(DirNameOf(exe_path_));  // Add location of exe file to search paths

    esmini::common::Config config("esmini", argc_, argv_);
    std::tie(argc_, argv_) = config.Load();

    if (opt.ParseArgs(argc_, argv_) != 0)
    {
        PrintUsage();
        return -2;
    }

    config.LogLoadedConfigFiles();

    std::string strAllSetOptions = opt.GetSetOptionsAsStr();

    std::string logFilePathOptionValue = TxtLogger::Inst().CreateLogFilePath();
    if (opt.IsOptionArgumentSet("param_dist"))
    {
        // deferring the creation of log file as name of it will be changed afterwards due to permutation distribution
        opt.ClearOption("logfile_path");
    }

    TxtLogger::Inst().SetMetaDataEnabled(opt.IsOptionArgumentSet("log_meta_data"));
    if (opt.IsOptionArgumentSet("log_only_modules"))
    {
        arg_str             = opt.GetOptionArg("log_only_modules");
        const auto splitted = SplitString(arg_str, ',');
        if (!splitted.empty())
        {
            std::unordered_set<std::string> logOnlyModules(splitted.begin(), splitted.end());
            TxtLogger::Inst().SetLogOnlyModules(logOnlyModules);
        }
    }
    if (opt.IsOptionArgumentSet("log_skip_modules"))
    {
        arg_str             = opt.GetOptionArg("log_skip_modules");
        const auto splitted = SplitString(arg_str, ',');
        if (!splitted.empty())
        {
            std::unordered_set<std::string> logSkipModules(splitted.begin(), splitted.end());
            TxtLogger::Inst().SetLogSkipModules(logSkipModules);
        }
    }
    TxtLogger::Inst().SetLoggersVerbosity();
    OSCParameterDistribution& dist = OSCParameterDistribution::Inst();

    if (dist.GetNumPermutations() > 0)
    {
        LOG_INFO("Re-using parameter distribution {}", dist.GetFilename());
    }
    else if (opt.IsOptionArgumentSet("param_dist") || opt.IsOptionArgumentSet("osc"))
    {
        if (dist.GetNumPermutations() == 0)
        {
            std::string strParamDist = "";
            if (opt.IsOptionArgumentSet("param_dist"))
            {  // use the param_dist to read the distribution and scenario file
                strParamDist = opt.GetOptionArg("param_dist");
            }
            else if (opt.IsOptionArgumentSet("osc"))
            {  // use osc to read the distribution and scenario file
                strParamDist     = opt.GetOptionArg("osc");
                dist.IsParamDist = false;
            }

            if (LoadParameterDistribution(strParamDist) != 0)
            {
                return -1;
            }
        }
    }

    if (!dist.GetScenarioFileName().empty() && !(opt.IsOptionArgumentSet("param_dist") && opt.IsOptionArgumentSet("osc")))
    {
        // User provided only param_dist file, as --osc option. Resolve --osc scenario file and --param_dist distribution file

        // move the param dist from --osc to --param_dist option
        opt.SetOptionValue("param_dist", opt.GetOptionArg("osc"));

        // scenario file is resolved from within the param_dist file
        opt.SetOptionValue("osc", dist.GetScenarioFileName());
    }

    if (opt.GetOptionSet("return_nr_permutations"))
    {
        LOG_INFO("Nr permutations: {}", dist.GetNumPermutations());
        // The distribution is loadad and we can abort the initialization here
        // the outer scope will deliver the number of permutations as return value
        return 0;
    }

    if (opt.IsOptionArgumentSet("param_permutation"))  // permutation index set by argument
    {
        int permutation_index = strtoi(opt.GetOptionArg("param_permutation"));

        if (dist.GetNumPermutations() > 0)
        {
            if (permutation_index >= static_cast<int>(dist.GetNumPermutations()) || permutation_index < 0)
            {
                LOG_ERROR("Requested permutation {} out of range [{} .. {}]", permutation_index, 0, dist.GetNumPermutations() - 1);
                return -1;
            }
            else
            {
                dist.SetIndex(static_cast<unsigned int>(permutation_index));
            }
        }
        else if (permutation_index > 0)
        {
            LOG_ERROR("No permutations available, requested permutation {} ignored", permutation_index);
            return -1;
        }
    }
    else if (dist.GetNumPermutations() > 0)
    {
        if (dist.GetRequestedIndex() > -1)  // Requested via lib API
        {
            if (dist.SetIndex(static_cast<unsigned int>(dist.GetRequestedIndex())) != 0)
            {
                LOG_ERROR_AND_QUIT("Failed to set requested index {}", dist.GetRequestedIndex());
            }
        }
        else
        {
            dist.IncrementIndex();
        }
    }

    if (dist.GetNumPermutations() > 0)
    {
        logFilePathOptionValue = dist.AddInfoToFilepath(logFilePathOptionValue);
        opt.SetOptionValue("logfile_path", logFilePathOptionValue);
    }

    TxtLogger::Inst().SetLogFilePath(logFilePathOptionValue);
    TxtLogger::Inst().LogTimeOnly();
    LOG_INFO("Player options: {}", strAllSetOptions);

    if (opt.GetOptionSet("use_signs_in_external_model"))
    {
        LOG_INFO("Use sign models in external scene graph model, skip creating sign models");
    }

    if (dist.GetNumPermutations() > 0)
    {
        LOG_INFO("Using parameter distribution file: {}", dist.GetFilename());
    }

    if (opt.GetOptionSet("threads"))
    {
#ifdef __APPLE__
        LOG_WARN("Separate viewer thread requested. Unfortunately only supported on Windows and Linux.");
        LOG_INFO("See https://www.mail-archive.com/osg-users@lists.openscenegraph.org/msg72698.html for an explanation.");
        return -1;
#else
        threads = true;
        LOG_INFO("Run viewer in separate thread");
#endif
    }

    if (opt.GetOptionSet("server"))
    {
        launch_server = true;
        LOG_INFO("Launch server to receive state of external Ego simulator");
    }

    int index = 0;
    for (; (arg_str = opt.GetOptionArg("fixed_timestep", index)) != ""; index++)
    {
        double timestep = std::stod(arg_str);
        if (timestep > SMALL_NUMBER)
        {
            SetFixedTimestep(std::stod(arg_str));
            LOG_INFO("Run simulation decoupled from realtime, with fixed timestep: {:.2f}", GetFixedTimestep());
        }
        else
        {
            LOG_INFO("Zero timestep ignored, running in realtime speed");
        }
    }
    if (index == 0)
    {
        LOG_INFO("No fixed timestep specified - running in realtime speed");
    }

    if (opt.GetOptionArg("path") != "")
    {
        int counter = 0;
        while ((arg_str = opt.GetOptionArg("path", counter)) != "")
        {
            SE_Env::Inst().AddPath(arg_str);
            LOG_INFO("Added path {}", arg_str);
            counter++;
        }
    }

    if (opt.GetOptionSet("disable_controllers"))
    {
        disable_controllers_ = true;
        LOG_INFO("Disable entity controllers");
    }

    if (opt.GetOptionSet("ignore_z"))
    {
        LOG_INFO("Ignoring z values and placing vehicle relative to road");
    }

    if (opt.GetOptionSet("ignore_p"))
    {
        LOG_INFO("Ignoring pitch values and placing vehicle relative to road");
    }

    if (opt.GetOptionSet("ignore_r"))
    {
        LOG_INFO("Ignoring roll values and placing vehicle relative to road");
    }

    // Use specific seed for repeatable scenarios?
    if ((arg_str = opt.GetOptionArg("seed")) != "")
    {
        unsigned int seed = static_cast<unsigned int>(std::stoul(arg_str));
        LOG_INFO("Using specified seed {}", seed);
        SE_Env::Inst().GetRand().SetSeed(seed);
    }
    else
    {
        LOG_INFO("Generated seed {}", SE_Env::Inst().GetRand().GetSeed());
    }

    if (opt.GetOptionSet("collision"))
    {
        SE_Env::Inst().SetCollisionDetection(true);
    }

    if (opt.GetOptionSet("plot"))
    {
        if (opt.GetOptionArg("plot") != "synchronous")
        {
#ifdef __APPLE__
            LOG_INFO("Plot mode {} not supported on mac systems (OpenGL graphics must run in main thread), applying synchronous mode",
                     opt.GetOptionArg("plot"));
            opt.ChangeOptionArg("plot", "synchronous");
#else
            if (opt.GetOptionArg("plot") != "asynchronous")
            {
                LOG_WARN("Plot mode {} not recognized. applying default asynchronous mode", opt.GetOptionArg("plot"));
                opt.ChangeOptionArg("plot", "asynchronous");
            }
#endif  // __APPLE__
        }
        LOG_INFO("Plot mode: {}", opt.GetOptionArg("plot"));
    }

    // Create scenario engine
    try
    {
        if ((arg_str = opt.GetOptionArg("osc")) != "")
        {
            SE_Env::Inst().AddPath(DirNameOf(arg_str));  // add scenario directory to list pf paths
            scenarioEngine = new ScenarioEngine(arg_str, disable_controllers_);
            Logger::Inst().SetTimePtr(scenarioEngine->GetSimulationTimePtr());
        }
        else if ((arg_str = opt.GetOptionArg("osc_str")) != "")
        {
            // parse XML string as document
            pugi::xml_document doc;
            std::string        xml_str(arg_str);
            if (!doc.load_buffer(xml_str.c_str(), xml_str.length()))
            {
                return -1;
            }
            scenarioEngine = new ScenarioEngine(doc, disable_controllers_);
            Logger::Inst().SetTimePtr(scenarioEngine->GetSimulationTimePtr());
        }
        else
        {
            LOG_ERROR("Error: Missing required OpenSCENARIO filename argument or XML string or parameter distribution");
            PrintUsage();

            return -1;
        }
    }
    catch (std::exception& e)
    {
        LOG_ERROR(std::string("Exception: ").append(e.what()));
        return -1;
    }

    // register list of injected actions to the scenario engine
    scenarioEngine->SetInjectedActionsPtr(player_server_->GetInjectedActionsPtr());

    if (scenarioEngine->GetInitStatus() != 0)
    {
        return -1;
    }

    // Save xml
    if (opt.GetOptionSet("save_xosc"))
    {
        std::string         filename = FileNameOf(scenarioEngine->getScenarioFilename());
        pugi::xml_document* xml_doc  = scenarioEngine->scenarioReader->GetDXMLDocument();

        if (xml_doc)
        {
            filename = dist.AddInfoToFilepath(filename);
            xml_doc->save_file(filename.c_str());
        }

        if (opt.GetOptionArg("save_xosc") == "quit")
        {
            SetQuitRequest(true);
            return 0;
        }
    }

    // Fetch scenario gateway and OpenDRIVE manager objects
    scenarioGateway = scenarioEngine->getScenarioGateway();
    odr_manager     = scenarioEngine->getRoadManager();

#ifdef _USE_OSI
    osiReporter = new OSIReporter(scenarioEngine);
    osiReporter->SetCounterPtr(&frame_counter_);
    osiReporter->SetStationaryModelReference(scenarioEngine->getSceneGraphFilename());
    scenarioEngine->storyBoard.SetOSIReporter(osiReporter);

    if (opt.GetOptionSet("osi_receiver_ip"))
    {
        osiReporter->OpenSocket(opt.GetOptionArg("osi_receiver_ip"));
        if (osiReporter->GetOSIFrequency() == 0)
        {
            osiReporter->SetOSIFrequency(1);
        }
    }

    if (opt.GetOptionSet("osi_crop_dynamic") == true)
    {
        int counter = 0;

        while ((arg_str = opt.GetOptionArg("osi_crop_dynamic", counter)) != "")
        {
            const auto splitted = SplitString(arg_str, ',');
            if (splitted.size() == 2)
            {
                osiReporter->CropOSIDynamicGroundTruth(strtoi(splitted[0]), strtod(splitted[1]));
            }
            else
            {
                LOG_ERROR("Expected osi_crop <id,radius>. Got {} values instead of 2.", splitted.size());
            }

            counter++;
        }
    }

    if (opt.GetOptionSet("osi_exclude_ghost"))
    {
        osiReporter->ExcludeGhost();
    }

    std::string osi_filename;
    // First check arguments
    if (opt.GetOptionSet("osi_file"))
    {
        osi_filename = opt.GetOptionArg("osi_file");
        if (osiReporter->GetOSIFrequency() == 0)
        {
            osiReporter->SetOSIFrequency(1);
        }
        SetOSIFileStatus(true, osi_filename.c_str());
    }

    if ((arg_str = opt.GetOptionArg("osi_freq")) != "")
    {
        osiReporter->SetOSIFrequency(atoi(arg_str.c_str()));
        LOG_INFO("Run simulation decoupled from realtime, with fixed timestep: {:.2f}", GetFixedTimestep());
    }

    if ((arg_str = opt.GetOptionArg("osi_static_reporting")) != "")
    {
        osiReporter->SetOSIStaticReportMode(static_cast<OSIReporter::OSIStaticReportMode>(atoi(arg_str.c_str())));
        LOG_INFO("OSI static data reporting mode: {}", arg_str);
    }
#endif  // _USE_OSI

    // Initialize CSV logger for recording vehicle data
    if (opt.GetOptionSet("csv_logger"))
    {
        CSV_Log = &CSV_Logger::Inst();
        if (CSV_Log)
        {
            std::string filename = opt.GetOptionArg("csv_logger");

            if (dist.GetNumPermutations() > 0)
            {
                filename = dist.AddInfoToFilepath(filename);
            }

            CSV_Log->Open(scenarioEngine->getScenarioFilename(), static_cast<int>(scenarioEngine->entities_.object_.size()), filename);
            LOG_INFO("Log all vehicle data in csv file");
        }
        else
        {
            LOG_ERROR("Failed to open CSV log");
        }
    }

    // Create a data file for later replay?
    if ((arg_str = opt.GetOptionArg("record")) != "")
    {
        std::string filename;

        if (!arg_str.empty())
        {
            if (IsDirectoryName(arg_str))
            {
                filename = arg_str + FileNameWithoutExtOf(scenarioEngine->getScenarioFilename()) + ".dat";
            }
            else
            {
                filename = arg_str;
            }
        }
        else
        {
            filename = SE_Env::Inst().GetDatFilePath();
        }
        filename = esmini::common::ValidateAndCreateFilePath(filename, DAT_FILENAME, ".dat");
        if (dist.GetNumPermutations() > 0)
        {
            filename = dist.AddInfoToFilepath(filename);
        }

        LOG_INFO("Recording data to file {}", filename);
        scenarioGateway->RecordToFile(filename, scenarioEngine->getOdrFilename(), scenarioEngine->getSceneGraphFilename());
    }

    if (launch_server)
    {
        // Launch UDP server to receive external Ego state
        StartServer(scenarioEngine);
    }

    if (opt.GetOptionSet("player_server"))
    {
        LOG_INFO("Launch server to receive actions to inject");

        // Launch UDP server to receive actions from external process
        player_server_->Start();
    }

    player_init_semaphore.Set();

    if (opt.IsInOriginalArgs("--window") || opt.IsInOriginalArgs("--borderless-window"))
    {
#ifdef _USE_OSG

        if (threads)
        {
            viewer_init_semaphore.Set();

            // Launch Viewer in a separate thread
            thread.Start(viewer_thread, static_cast<void*>(this));

            viewer_init_semaphore.Wait();

            if (viewerState_ == ViewerState::VIEWER_STATE_NOT_STARTED)
            {
                LOG_WARN("Viewer still not ready. Start scenario anyway. Viewer will launch when ready.");
            }
            else if (viewerState_ == ViewerState::VIEWER_STATE_DONE)
            {
                LOG_ERROR("Viewer already signaled done - something went wrong");
                return -1;
            }
            else if (viewerState_ == ViewerState::VIEWER_STATE_FAILED)
            {
                LOG_ERROR("Viewer initialization failed");
                return -1;
            }
        }
        else
        {
            if (InitViewer() != 0)
            {
                LOG_ERROR("Viewer initialization failed");
                return -1;
            }
        }

#else
        LOG_WARN("window requested, but esmini compiled without OSG capabilities");
#endif
    }
    else if (opt.GetOptionSet("capture_screen"))
    {
        PrintUsage();
        LOG_ERROR_AND_QUIT("Capture screen requires a window to be specified!");
    }

    InitControllersPostPlayer();

    if (opt.HasUnknownArgs())
    {
        opt.PrintUnknownArgs("Unrecognized arguments:");
        PrintUsage();
        return -1;  // we harmonize all applications to quit on unknown arguments
    }

    Frame(0.0);

    if (opt.GetOptionSet("pause"))
    {
        SetState(PlayerState::PLAYER_STATE_PAUSE);
    }

    player_init_semaphore.Release();

    return 0;
}

void ScenarioPlayer::RegisterObjCallback(int id, ObjCallbackFunc func, void* data)
{
    ObjCallback cb;
    cb.id   = id;
    cb.func = func;
    cb.data = data;
    objCallback.push_back(cb);
}

void ScenarioPlayer::UpdateCSV_Log()
{
    // Flag for signalling end of data line, all vehicles reported
    bool isendline = false;

    CSV_Log->LogEntryHeader(scenarioEngine->getSimulationTime());

    // For each vehicle (entitity) stored in the ScenarioPlayer
    for (size_t i = 0; i < scenarioEngine->entities_.object_.size(); i++)
    {
        // Create a pointer to the object at position i in the entities vector
        Object* obj = scenarioEngine->entities_.object_[i];

        // Create a Position object for extracting this vehicles XYZ coordinates
        roadmanager::Position pos = obj->pos_;

        // Extract the String name of the object and store in a compatable const char array
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
        CSV_Log->LogVehicleData(isendline,
                                name_,
                                obj->id_,
                                obj->speed_,
                                obj->wheel_angle_,
                                obj->wheel_rot_,
                                obj->boundingbox_.center_.x_,
                                obj->boundingbox_.center_.y_,
                                obj->boundingbox_.center_.z_,
                                obj->boundingbox_.dimensions_.length_,
                                obj->boundingbox_.dimensions_.width_,
                                obj->boundingbox_.dimensions_.height_,
                                pos.GetX(),
                                pos.GetY(),
                                pos.GetZ(),
                                pos.GetVelX(),
                                pos.GetVelY(),
                                pos.GetVelZ(),
                                pos.GetAccX(),
                                pos.GetAccY(),
                                pos.GetAccZ(),
                                pos.GetS(),
                                pos.GetT(),
                                pos.GetLaneId(),
                                pos.GetOffset(),
                                pos.GetH(),
                                pos.GetHRate(),
                                pos.GetHRelative(),
                                pos.GetHRelativeDrivingDirection(),
                                pos.GetP(),
                                pos.GetCurvature(),
                                collision_ids.c_str());
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

int ScenarioPlayer::LoadParameterDistribution(std::string filename)
{
    OSCParameterDistribution& dist = OSCParameterDistribution::Inst();

    if (dist.GetNumPermutations() > 0)
    {
        LOG_INFO("Parameter distribution already loaded, reusing it");
        return -2;
    }
    else
    {
        if (dist.Load(filename) != 0)
        {
            return -1;
        }
    }

    return 0;
}

int ScenarioPlayer::GetNumberOfVariables()
{
    return scenarioEngine->scenarioReader->variables.GetNumberOfParameters();
}

const char* ScenarioPlayer::GetVariableName(int index, OSCParameterDeclarations::ParameterType* type)
{
    return scenarioEngine->scenarioReader->variables.GetParameterName(index, type);
}

int ScenarioPlayer::SetVariableValue(const char* name, const void* value)
{
    return scenarioEngine->scenarioReader->variables.setParameterValue(name, value);
}

int ScenarioPlayer::GetVariableValue(const char* name, void* value)
{
    return scenarioEngine->scenarioReader->variables.getParameterValue(name, value);
}

int ScenarioPlayer::GetVariableValueInt(const char* name, int& value)
{
    return scenarioEngine->scenarioReader->variables.getParameterValueInt(name, value);
}

int ScenarioPlayer::GetVariableValueDouble(const char* name, double& value)
{
    return scenarioEngine->scenarioReader->variables.getParameterValueDouble(name, value);
}

int ScenarioPlayer::GetVariableValueString(const char* name, const char*& value)
{
    return scenarioEngine->scenarioReader->variables.getParameterValueString(name, value);
}

int ScenarioPlayer::GetVariableValueBool(const char* name, bool& value)
{
    return scenarioEngine->scenarioReader->variables.getParameterValueBool(name, value);
}

int ScenarioPlayer::SetVariableValue(const char* name, int value)
{
    return scenarioEngine->scenarioReader->variables.setParameterValue(name, value);
}

int ScenarioPlayer::SetVariableValue(const char* name, double value)
{
    return scenarioEngine->scenarioReader->variables.setParameterValue(name, value);
}

int ScenarioPlayer::SetVariableValue(const char* name, const char* value)
{
    return scenarioEngine->scenarioReader->variables.setParameterValue(name, value);
}

int ScenarioPlayer::SetVariableValue(const char* name, bool value)
{
    return scenarioEngine->scenarioReader->variables.setParameterValue(name, value);
}

// todo
int ScenarioPlayer::GetNumberOfProperties(int index)
{
    return static_cast<int>(scenarioEngine->entities_.object_[static_cast<unsigned int>(index)]->properties_.property_.size());
}

const char* ScenarioPlayer::GetPropertyName(int index, int propertyIndex)
{
    return scenarioEngine->entities_.object_[static_cast<unsigned int>(index)]
        ->properties_.property_[static_cast<unsigned int>(propertyIndex)]
        .name_.c_str();
}

const char* ScenarioPlayer::GetPropertyValue(int index, int propertyIndex)
{
    return scenarioEngine->entities_.object_[static_cast<unsigned int>(index)]
        ->properties_.property_[static_cast<unsigned int>(propertyIndex)]
        .value_.c_str();
}

#ifdef _USE_OSG
void ReportKeyEvent(viewer::KeyEvent* keyEvent, void* data)
{
    ScenarioPlayer* player = static_cast<ScenarioPlayer*>(data);
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
