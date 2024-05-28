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

#ifdef _USE_OSG
#include "viewer.hpp"
#endif

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

static void log_callback(const char* str)
{
    printf("%s\n", str);
}

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
    osi_freq_            = 0;
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
    Logger::Inst().SetTimePtr(0);
    if (scenarioEngine)
    {
        delete scenarioEngine;
        scenarioEngine = nullptr;
    }

#ifdef _USE_OSI
    if (osiReporter)
    {
        delete osiReporter;
    }
#endif  // _USE_OSI
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
        osiReporter->SetUpdated(false);
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
            LOG("Info: Simulation time > 1 hour. Put a stopTrigger for automatic ending");
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
    static __int64 time_stamp = 0;
    double         dt;

    if ((dt = GetFixedTimestep()) < 0.0)
    {
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
        if (osi_freq_ > 0)
        {
            osiReporter->ReportSensors(sensor);

            if ((GetCounter() - 1) % osi_freq_ == 0)
            {
                osiReporter->UpdateOSIGroundTruth(scenarioGateway->objectState_);
            }

            osiReporter->UpdateOSITrafficCommand();
        }
    }
#endif  // _USE_OSI

    // LOG("%d %d %.2f h: %.5f road_h %.5f h_relative_road %.5f",
    //     scenarioEngine->entities_.object_[0]->pos_.GetTrackId(),
    //     scenarioEngine->entities_.object_[0]->pos_.GetLaneId(),
    //     scenarioEngine->entities_.object_[0]->pos_.GetS(),
    //     scenarioEngine->entities_.object_[0]->pos_.GetH(),
    //     scenarioEngine->entities_.object_[0]->pos_.GetHRoad(),
    //     scenarioEngine->entities_.object_[0]->pos_.GetHRelative());

    mutex.Unlock();
}

#ifdef _USE_OSG
void ScenarioPlayer::ViewerFrame(bool init)
{
    if (viewer_ == nullptr)
    {
        return;
    }

    static double last_dot_time = scenarioEngine->getSimulationTime();
    (void)last_dot_time;

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
            else if (entity->routewaypoints_->group_->getNumChildren() && obj->pos_.GetRoute() == nullptr)
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

            if (entity->trail_->pline_vertex_data_->size() > static_cast<unsigned int>(obj->trail_.GetNumberOfVertices()))
            {
                // Reset the trail, probably there has been a ghost restart
                entity->trail_->Reset();
                for (size_t j = 0; j < static_cast<unsigned int>(obj->trail_.GetNumberOfVertices()); j++)
                {
                    entity->trail_->AddPoint(osg::Vec3(static_cast<float>(obj->trail_.vertex_[j].x),
                                                       static_cast<float>(obj->trail_.vertex_[j].y),
                                                       static_cast<float>(obj->trail_.vertex_[j].z + (obj->GetId() + 1) * TRAIL_Z_OFFSET)));
                }
            }

            if (static_cast<unsigned int>(obj->trail_.GetNumberOfVertices()) > entity->trail_->pline_vertex_data_->size())
            {
                entity->trail_->AddPoint(osg::Vec3(static_cast<float>(obj->pos_.GetX()),
                                                   static_cast<float>(obj->pos_.GetY()),
                                                   static_cast<float>(obj->pos_.GetZ() + (obj->GetId() + 1) * TRAIL_Z_OFFSET)));
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
        viewer_->Frame();
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
            LOG("FetchCapturedImagePtr Error: No image data");
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
            LOG_AND_QUIT("Invalid on-screen info mode %d. Valid range is 0-3", mask);
        }
        viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_INFO | viewer::NodeMask::NODE_MASK_INFO_PER_OBJ,
                                 mask * viewer::NodeMask::NODE_MASK_INFO);
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
        viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_TRAIL_LINES | viewer::NodeMask::NODE_MASK_TRAIL_DOTS,
                                 mask * viewer::NodeMask::NODE_MASK_TRAIL_LINES);
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
            size_t pos  = 0;
            double v[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
            int    i    = 0;
            for (i = 0; i < 5; i++)
            {
                pos = arg_str.find(",");

                if (i < 2 && pos == std::string::npos)
                {
                    LOG_AND_QUIT("Expected custom_camera <x,y,z>[,h,p], got only %d values", i + 1);
                }
                else if (i == 3 && pos == std::string::npos)
                {
                    LOG_AND_QUIT("Expected custom_camera <x,y,z>[,h,p], got %d values", i + 1);
                }
                v[i] = strtod(arg_str.substr(0, pos));
                arg_str.erase(0, pos == std::string::npos ? pos : pos + 1);

                if (i == 2 && pos == std::string::npos)
                {
                    // Only position specified, stop now

                    break;
                }
            }
            if (!arg_str.empty())
            {
                LOG_AND_QUIT("Expected custom_camera <x,y,z>[,h,p], got too many values. Make sure only 3 or 5 values is specified");
            }

            if (i == 2)
            {
                AddCustomCamera(v[0], v[1], v[2], false);
                LOG("Created custom fixed camera %d (%.2f, %.2f, %.2f)", counter, v[0], v[1], v[2]);
            }
            else
            {
                AddCustomCamera(v[0], v[1], v[2], v[3], v[4], false);
                LOG("Created custom fixed camera %d (%.2f, %.2f, %.2f, %.2f, %.2f)", counter, v[0], v[1], v[2], v[3], v[4]);
            }
            counter++;
        }
    }

    if (opt.GetOptionSet("custom_fixed_camera") == true)
    {
        int counter = 0;

        while ((arg_str = opt.GetOptionArg("custom_fixed_camera", counter)) != "")
        {
            size_t pos  = 0;
            double v[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
            int    i    = 0;
            for (i = 0; i < 5; i++)
            {
                pos = arg_str.find(",");

                if (i < 2 && pos == std::string::npos)
                {
                    LOG_AND_QUIT("Expected custom_fixed_camera <x,y,z>[,h,p], got only %d values", i + 1);
                }
                else if (i == 3 && pos == std::string::npos)
                {
                    LOG_AND_QUIT("Expected custom_fixed_camera <x,y,z>[,h,p], got %d values", i + 1);
                }
                v[i] = strtod(arg_str.substr(0, pos));
                arg_str.erase(0, pos == std::string::npos ? pos : pos + 1);

                if (i == 2 && pos == std::string::npos)
                {
                    // Only position specified, stop now

                    break;
                }
            }
            if (!arg_str.empty())
            {
                LOG_AND_QUIT("Expected custom_fixed_camera <x,y,z>[,h,p], got too many values. Make sure only 3 or 5 values is specified");
            }

            if (i == 2)
            {
                AddCustomCamera(v[0], v[1], v[2], true);
                LOG("Created custom fixed camera %d (%.2f, %.2f, %.2f)", counter, v[0], v[1], v[2]);
            }
            else
            {
                AddCustomCamera(v[0], v[1], v[2], v[3], v[4], true);
                LOG("Created custom fixed camera %d (%.2f, %.2f, %.2f, %.2f, %.2f)", counter, v[0], v[1], v[2], v[3], v[4]);
            }
            counter++;
        }
    }

    if (opt.GetOptionSet("custom_fixed_top_camera") == true)
    {
        int counter = 0;

        while ((arg_str = opt.GetOptionArg("custom_fixed_top_camera", counter)) != "")
        {
            size_t pos  = 0;
            double v[4] = {0.0, 0.0, 0.0, 0.0};
            for (int i = 0; i < 4; i++)
            {
                pos = arg_str.find(",");
                if (i < 3 && pos == std::string::npos)
                {
                    LOG_AND_QUIT("Expected custom_fixed_top_camera <x,y,z,rot>, got only %d values", i + 1);
                }
                v[i] = strtod(arg_str.substr(0, pos));
                arg_str.erase(0, pos == std::string::npos ? pos : pos + 1);
            }
            if (!arg_str.empty())
            {
                LOG_AND_QUIT("Expected custom_fixed_top_camera <x,y,z,rot>, got too many values. Make sure only 4 values is specified");
            }

            AddCustomFixedTopCamera(v[0], v[1], v[2], v[3]);
            LOG("Created custom fixed top camera %d (%.2f, %.2f, %.2f, %.2f)", counter, v[0], v[1], v[2], v[3]);
            counter++;
        }
    }

    if (opt.GetOptionSet("custom_light") == true)
    {
        int counter      = 0;
        int lightCounter = 0;

        while ((arg_str = opt.GetOptionArg("custom_light", counter)) != "")
        {
            size_t pos  = 0;
            double v[4] = {0.0, 0.0, 0.0, 0.0};
            for (int i = 0; i < 4; i++)
            {
                pos = arg_str.find(",");
                if (i < 3 && pos == std::string::npos)
                {
                    LOG_AND_QUIT("Expected custom_light <x,y,z,intensity>, got only %d values", i + 1);
                }
                v[i] = strtod(arg_str.substr(0, pos));
                arg_str.erase(0, pos == std::string::npos ? pos : pos + 1);
            }
            if (!arg_str.empty())
            {
                LOG_AND_QUIT("Expected custom_light <x,y,z,intensity>, got too many values. Make sure only 4 values is specified");
            }

            if (AddCustomLightSource(v[0], v[1], v[2], v[3]) == 0)
            {
                LOG("Created custom light source %d (%.2f, %.2f, %.2f, %.2f)", lightCounter, v[0], v[1], v[2], v[3]);
                lightCounter++;
            }
            else
            {
                LOG("Max nr custom lights (%d) reached. Ignoring (%.2f, %.2f, %.2f, %.2f)", lightCounter, v[0], v[1], v[2], v[3]);
            }
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
        LOG("Follow object %d", strtoi(opt.GetOptionArg("follow_object")));
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

    player->InitControllersPostPlayer();

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

int ScenarioPlayer::AddObjectSensor(Object* obj, double x, double y, double z, double h, double near_dist, double far_dist, double fovH, int maxObj)
{
    if (obj == nullptr)
    {
        return -1;
    }

    sensor.push_back(new ObjectSensor(&scenarioEngine->entities_, obj, x, y, z, h, near_dist, far_dist, fovH, maxObj));

#ifdef _USE_OSG
    if (viewer_)
    {
        int object_index = scenarioEngine->entities_.GetObjectIdxById(obj->GetId());
        if (object_index >= 0)
        {
            mutex.Lock();
            sensorFrustum.push_back(
                new viewer::SensorViewFrustum(sensor.back(), viewer_->entities_[static_cast<unsigned int>(object_index)]->txNode_));
            mutex.Unlock();
        }
    }
#endif

    return static_cast<int>(sensor.size()) - 1;
}

int ScenarioPlayer::GetNumberOfObjectSensors()
{
    return static_cast<int>(sensor.size());
}

int ScenarioPlayer::GetNumberOfSensorsAttachedToObject(Object* obj)
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

    SE_Options& opt = SE_Env::Inst().GetOptions();

    opt.Reset();

    // use an ArgumentParser object to manage the program arguments.
    opt.AddOption("osc", "OpenSCENARIO filename (required) - if path includes spaces, enclose with \"\"", "filename");
    opt.AddOption("aa_mode", "Anti-alias mode=number of multisamples (subsamples, 0=off, 4=default)", "mode");
    opt.AddOption("bounding_boxes", "Show entities as bounding boxes (toggle modes on key ',') ");
    opt.AddOption("capture_screen", "Continuous screen capture. Warning: Many jpeg files will be created");
    opt.AddOption(
        "camera_mode",
        "Initial camera mode (\"orbit\" (default), \"fixed\", \"flex\", \"flex-orbit\", \"top\", \"driver\", \"custom\") (swith with key 'k') ",
        "mode");
    opt.AddOption("csv_logger", "Log data for each vehicle in ASCII csv format", "csv_filename");
    opt.AddOption("collision", "Enable global collision detection, potentially reducing performance");
    opt.AddOption("custom_camera", "Additional custom camera position <x,y,z>[,h,p] (multiple occurrences supported)", "position");
    opt.AddOption("custom_fixed_camera",
                  "Additional custom fixed camera position <x,y,z>[,h,p] (multiple occurrences supported)",
                  "position and optional orientation");
    opt.AddOption("custom_fixed_top_camera", "Additional custom top camera <x,y,z,rot> (multiple occurrences supported)", "position and rotation");
    opt.AddOption("custom_light",
                  "Additional custom light source <x,y,z,intensity> intensity range 0..1 (multiple occurrences supported)",
                  "position and intensity");
    opt.AddOption("disable_controllers", "Disable controllers");
    opt.AddOption("disable_log", "Prevent logfile from being created");
    opt.AddOption("disable_stdout", "Prevent messages to stdout");
    opt.AddOption("enforce_generate_model", "Generate road 3D model even if SceneGraphFile is specified");
    opt.AddOption("fixed_timestep", "Run simulation decoupled from realtime, with specified timesteps", "timestep");
    opt.AddOption("follow_object", "Set index of intial object for camera to follow (change with Tab/shift-Tab)", "index");
    opt.AddOption("generate_no_road_objects", "Do not generate any OpenDRIVE road objects (e.g. when part of referred 3D model)");
    opt.AddOption("generate_without_textures", "Do not apply textures on any generated road model (set colors instead as for missing textures)");
    opt.AddOption("ground_plane", "Add a large flat ground surface");
    opt.AddOption("headless", "Run without viewer window");
    opt.AddOption("help", "Show this help message");
    opt.AddOption("hide_route_waypoints", "Disable route waypoint visualization (toggle with key 'R')");
    opt.AddOption("hide_trajectories", "Hide trajectories from start (toggle with key 'n')");
    opt.AddOption("info_text", "Show on-screen info text (toggle key 'i') mode 0=None 1=current (default) 2=per_object 3=both", "mode");
    opt.AddOption("logfile_path", "logfile path/filename, e.g. \"../esmini.log\" (default: log.txt)", "path");
    opt.AddOption("osc_str", "OpenSCENARIO XML string", "string");
    opt.AddOption("osg_screenshot_event_handler", "Revert to OSG default jpg images ('c'/'C' keys handler)");
#ifdef _USE_OSI
    opt.AddOption("osi_file", "save osi trace file", "filename", DEFAULT_OSI_TRACE_FILENAME);
    opt.AddOption("osi_freq", "relative frequence for writing the .osi file e.g. --osi_freq=2 -> we write every two simulation steps", "frequence");
    opt.AddOption("osi_lines", "Show OSI road lines (toggle during simulation by press 'u') ");
    opt.AddOption("osi_points", "Show OSI road pointss (toggle during simulation by press 'y') ");
    opt.AddOption("osi_receiver_ip", "IP address where to send OSI UDP packages", "IP address");
#endif
    opt.AddOption("param_dist", "Run variations of the scenario according to specified parameter distribution file", "filename");
    opt.AddOption("param_permutation", "Run specific permutation of parameter distribution", "index (0 .. NumberOfPermutations-1)");
    opt.AddOption("pause", "Pause simulation after initialization");
    opt.AddOption("path", "Search path prefix for assets, e.g. OpenDRIVE files (multiple occurrences supported)", "path");
    opt.AddOption("player_server", "Launch UDP server for action/command injection");
#ifdef _USE_IMPLOT
    opt.AddOption("plot", "Show window with line-plots of interesting data", "mode (asynchronous|synchronous)", "asynchronous");
#endif
    opt.AddOption("record", "Record position data into a file for later replay", "filename");
    opt.AddOption("road_features", "Show OpenDRIVE road features (\"on\", \"off\"  (default)) (toggle during simulation by press 'o') ", "mode");
    opt.AddOption("return_nr_permutations", "Return number of permutations without executing the scenario (-1 = error)");
    opt.AddOption("save_generated_model", "Save generated 3D model (n/a when a scenegraph is loaded)");
    opt.AddOption("save_xosc", "Save OpenSCENARIO file with any populated parameter values (from distribution)");
    opt.AddOption("seed", "Specify seed number for random generator", "number");
    opt.AddOption("sensors", "Show sensor frustums (toggle during simulation by press 'r') ");
    opt.AddOption("server", "Launch server to receive state of external Ego simulator");
    opt.AddOption("text_scale", "Scale screen overlay text", "factor", "1.0");
    opt.AddOption("threads", "Run viewer in a separate thread, parallel to scenario engine");
    opt.AddOption("trail_mode", "Show trail lines and/or dots (toggle key 'j') mode 0=None 1=lines 2=dots 3=both", "mode");
    opt.AddOption("use_signs_in_external_model", "When external scenegraph 3D model is loaded, skip creating signs from OpenDRIVE");
    opt.AddOption("version", "Show version and quit");

    exe_path_ = argv_[0];
    SE_Env::Inst().AddPath(DirNameOf(exe_path_));  // Add location of exe file to search paths

    if (opt.ParseArgs(argc_, argv_) != 0)
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

    if (opt.GetOptionSet("use_signs_in_external_model"))
    {
        LOG("Use sign models in external scene graph model, skip creating sign models");
    }

    OSCParameterDistribution& dist = OSCParameterDistribution::Inst();

    if (dist.GetNumPermutations() > 0)
    {
        LOG("Re-using parameter distribution %s", dist.GetFilename().c_str());
    }
    else if (opt.IsOptionArgumentSet("param_dist"))
    {
        if (dist.GetNumPermutations() == 0)
        {
            if (LoadParameterDistribution(opt.GetOptionArg("param_dist")) != 0)
            {
                return -1;
            }
        }
    }

    if (opt.GetOptionSet("return_nr_permutations"))
    {
        LOG("Nr permutations: %d", dist.GetNumPermutations());
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
                LOG("Requested permutation %d out of range [%d .. %d]", permutation_index, 0, dist.GetNumPermutations() - 1);
                return -1;
            }
            else
            {
                dist.SetIndex(static_cast<unsigned int>(permutation_index));
            }
        }
        else if (permutation_index > 0)
        {
            LOG("No permutations available, requested permutation %d ignored", permutation_index);
            return -1;
        }
    }
    else if (dist.GetNumPermutations() > 0)
    {
        if (dist.GetRequestedIndex() > -1)  // Requested via lib API
        {
            if (dist.SetIndex(static_cast<unsigned int>(dist.GetRequestedIndex())) != 0)
            {
                LOG_AND_QUIT("Failed to set requested index %d", dist.GetRequestedIndex());
            }
        }
        else
        {
            dist.IncrementIndex();
        }
    }

    std::string log_filename = SE_Env::Inst().GetLogFilePath();

    if (opt.GetOptionSet("disable_log"))
    {
        log_filename = "";
        printf("Disable logfile\n");
    }
    else if (opt.IsOptionArgumentSet("logfile_path"))
    {
        arg_str = opt.GetOptionArg("logfile_path");

        if (!arg_str.empty())
        {
            if (IsDirectoryName(arg_str))
            {
                log_filename = arg_str + LOG_FILENAME;
            }
            else
            {
                log_filename = arg_str;
            }
        }

        if (arg_str.empty())
        {
            printf("Custom logfile path empty, disable logfile\n");
        }
        else
        {
            printf("Custom logfile path: %s\n", log_filename.c_str());
        }
    }

    if (dist.GetNumPermutations() > 0)
    {
        log_filename = dist.AddInfoToFilepath(log_filename);
    }

    Logger::Inst().OpenLogfile(log_filename);
    Logger::Inst().LogVersion();

    if (dist.GetNumPermutations() > 0)
    {
        LOG("Using parameter distribution file: %s", dist.GetFilename().c_str());
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

    if (opt.GetOptionSet("server"))
    {
        launch_server = true;
        LOG("Launch server to receive state of external Ego simulator");
    }

    int index = 0;
    for (; (arg_str = opt.GetOptionArg("fixed_timestep", index)) != ""; index++)
    {
        double timestep = std::stod(arg_str);
        if (timestep > SMALL_NUMBER)
        {
            SetFixedTimestep(std::stod(arg_str));
            LOG("Run simulation decoupled from realtime, with fixed timestep: %.2f", GetFixedTimestep());
        }
        else
        {
            LOG("Zero timestep ignored, running in realtime speed");
        }
    }
    if (index == 0)
    {
        LOG("No fixed timestep specified - running in realtime speed");
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
        SE_Env::Inst().GetRand().SetSeed(seed);
    }
    else
    {
        LOG("Generated seed %u", SE_Env::Inst().GetRand().GetSeed());
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
            LOG("Plot mode %s not supported on mac systems (OpenGL graphics must run in main thread), applying synchronous mode",
                opt.GetOptionArg("plot").c_str());
            opt.ChangeOptionArg("plot", "synchronous");
#else
            if (opt.GetOptionArg("plot") != "asynchronous")
            {
                LOG("Plot mode %s not recognized. applying default asynchronous mode", opt.GetOptionArg("plot").c_str());
                opt.ChangeOptionArg("plot", "asynchronous");
            }
#endif  // __APPLE__
        }
        LOG("Plot mode: %s", opt.GetOptionArg("plot").c_str());
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
            LOG("Error: Missing required OpenSCENARIO filename argument or XML string");
            PrintUsage();

            return -1;
        }
    }
    catch (std::logic_error& e)
    {
        LOG(std::string("Exception: ").append(e.what()).c_str());
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
    }

    // Fetch scenario gateway and OpenDRIVE manager objects
    scenarioGateway = scenarioEngine->getScenarioGateway();
    odr_manager     = scenarioEngine->getRoadManager();

#ifdef _USE_OSI
    osiReporter = new OSIReporter(scenarioEngine);
    osiReporter->SetStationaryModelReference(scenarioEngine->getSceneGraphFilename());
    scenarioEngine->storyBoard.SetOSIReporter(osiReporter);

    if (opt.GetOptionSet("osi_receiver_ip"))
    {
        osiReporter->OpenSocket(opt.GetOptionArg("osi_receiver_ip"));
        if (osi_freq_ == 0)
        {
            osi_freq_ = 1;
        }
    }

    std::string osi_filename;
    // First check arguments
    if (opt.GetOptionSet("osi_file"))
    {
        osi_filename = opt.GetOptionArg("osi_file");
        if (osi_freq_ == 0)
        {
            osi_freq_ = 1;
        }
    }

    // Secondly check esmini environment variables
    if (osi_filename.empty())
    {
        osi_filename = SE_Env::Inst().GetOSIFilePath();
    }

    if (!osi_filename.empty() || SE_Env::Inst().GetOSIFileEnabled())
    {
        SetOSIFileStatus(true, osi_filename.c_str());
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
            LOG("Log all vehicle data in csv file");
        }
        else
        {
            LOG("Failed to open CSV log %s");
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

        if (dist.GetNumPermutations() > 0)
        {
            filename = dist.AddInfoToFilepath(filename);
        }

        LOG("Recording data to file %s", filename.c_str());
        scenarioGateway->RecordToFile(filename, scenarioEngine->getOdrFilename(), scenarioEngine->getSceneGraphFilename());
    }

    if (launch_server)
    {
        // Launch UDP server to receive external Ego state
        StartServer(scenarioEngine);
    }

    if (opt.GetOptionSet("player_server"))
    {
        LOG("Launch server to receive actions to inject");

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

            InitControllersPostPlayer();
        }

#else
        LOG("window requested, but esmini compiled without OSG capabilities");
#endif
    }
    else if (opt.GetOptionSet("capture_screen"))
    {
        PrintUsage();
        LOG_AND_QUIT("Capture screen requires a window to be specified!");
    }

    if (opt.HasUnknownArgs())
    {
        opt.PrintUnknownArgs("Unrecognized arguments:");
        PrintUsage();
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
                                scenarioEngine->getSimulationTime(),
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
        LOG("Parameter distribution already loaded, reusing it");
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
