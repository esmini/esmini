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
 * This application uses the Replay class to read and replay binary recordings of previously executed scenarios
 */

#include <random>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>
#include <map>

#include "StudioViewer.hpp"
#include "ScenarioGateway.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "Replay.hpp"
#include "collision.hpp"

using namespace scenarioengine;

#define TIME_SCALE_FACTOR     1.1
#define GHOST_CTRL_TYPE       100  // control type 100 indicates ghost
#define JUMP_DELTA_TIME_LARGE 1.0
#define JUMP_DELTA_TIME_SMALL 0.1

static const double     stepSize       = 0.01;
static const double     maxStepSize    = 0.1;
static const double     minStepSize    = 0.001;
static bool             pause_player   = false;  // continuous play
static double           time_scale     = 1.0;
static bool             no_ghost       = false;
static bool             no_ghost_model = false;
static std::vector<int> removeObjects;

double deltaSimTime;  // external - used by Viewer::RubberBandCamera

static std::vector<ScenarioEntity> scenarioEntity;
static std::weak_ptr<Replay>       g_player;

void setEntityVisibility(int index, bool visible)
{
    if (index >= 0 && index < static_cast<int>(scenarioEntity.size()))
    {
        if (visible != scenarioEntity[static_cast<unsigned int>(index)].visible)
        {
            scenarioEntity[static_cast<unsigned int>(index)].entityModel->txNode_->setNodeMask(visible ? 0xffffffff : 0x0);
            scenarioEntity[static_cast<unsigned int>(index)].visible = visible;
            if (scenarioEntity[static_cast<unsigned int>(index)].trajectory)
            {
                scenarioEntity[static_cast<unsigned int>(index)].trajectory->SetNodeMaskLines(visible ? 0xffffffff : 0x0);
            }
        }
    }
}

ScenarioEntity* getScenarioEntityById(int id)
{
    for (size_t i = 0; i < scenarioEntity.size(); i++)
    {
        if (scenarioEntity[i].id == id)
        {
            return &scenarioEntity[i];
        }
    }

    return 0;
}

int ShowGhosts(Replay* player, bool show)
{
    ObjectStateStructDat* state = 0;

    for (size_t j = 0; j < scenarioEntity.size(); j++)
    {
        ScenarioEntity* entity = &scenarioEntity[j];
        state                  = player->GetState(entity->id);

        if (entity->entityModel != nullptr && state->info.ctrl_type == GHOST_CTRL_TYPE)
        {
            entity->entityModel->txNode_->setNodeMask(show ? 0xffffffff : 0x0);
        }
    }

    return 0;
}

int ParseEntities(viewer::StudioViewer* viewer, Replay* player)
{
    double minTrajPointDist = 1;
    double z_offset         = 0.2;
    double width            = 1.75;

    struct OdoInfo
    {
        double x, y, odometer;
    };
    std::map<int, OdoInfo> odo_info;  // temporary keep track of entity odometers

    for (int i = 0; i < static_cast<int>(player->data_.size()); i++)
    {
        ReplayEntry*          entry = &player->data_[static_cast<unsigned int>(i)];
        ObjectStateStructDat* state = &entry->state;
        OdoInfo               odo_entry;

        if (no_ghost && state->info.ctrl_type == GHOST_CTRL_TYPE)
        {
            continue;
        }

        if (std::find(removeObjects.begin(), removeObjects.end(), state->info.id) != removeObjects.end())
        {
            continue;
        }

        ScenarioEntity* sc = getScenarioEntityById(state->info.id);

        // If not available, create it
        if (sc == 0)
        {
            ScenarioEntity new_sc;

            new_sc.id             = state->info.id;
            new_sc.trajPoints     = 0;
            new_sc.pos            = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0.0f};
            new_sc.trajectory     = nullptr;
            new_sc.wheel_angle    = 0.0f;
            new_sc.wheel_rotation = 0.0f;
            new_sc.name           = state->info.name;
            new_sc.visible        = true;
            std::string filename;
            if (state->info.model_id >= 0)
            {
                filename = SE_Env::Inst().GetModelFilenameById(state->info.model_id);
            }

            if ((new_sc.entityModel = viewer->CreateEntityModel(filename.c_str(),
                                                                osg::Vec4(0.5, 0.5, 0.5, 1.0),
                                                                viewer::EntityModel::EntityType::VEHICLE,
                                                                false,
                                                                state->info.name,
                                                                &state->info.boundingbox,
                                                                static_cast<EntityScaleMode>(state->info.scaleMode))) == 0)
            {
                return -1;
            }
            else
            {
                if (viewer->AddEntityModel(new_sc.entityModel) != 0)
                {
                    return -1;
                }
            }

            if (state->info.ctrl_type == GHOST_CTRL_TYPE && no_ghost_model)
            {
                new_sc.entityModel->txNode_->setNodeMask(0x0);
            }

            new_sc.bounding_box = state->info.boundingbox;

            // Add it to the list of scenario cars
            scenarioEntity.push_back(new_sc);

            sc = &scenarioEntity.back();

            odo_entry.x        = state->pos.x;
            odo_entry.y        = state->pos.y;
            odo_entry.odometer = 0.0;

            odo_info.insert(std::make_pair(new_sc.id, odo_entry));  // Set inital odometer value for the entity
        }

        if (sc->trajPoints == 0)
        {
            sc->trajPoints = new osg::Vec3Array;
        }

        if (sc->trajPoints->size() == 0)
        {
            sc->trajPoints->push_back(osg::Vec3f(static_cast<float>(static_cast<double>(state->pos.x) - viewer->origin_[0]),
                                                 static_cast<float>(static_cast<double>(state->pos.y) - viewer->origin_[1]),
                                                 state->pos.z + static_cast<float>(z_offset)));
        }
        else
        {
            if (sc->trajPoints->size() > 2 && GetLengthOfLine2D(state->pos.x,
                                                                state->pos.y,
                                                                (*sc->trajPoints)[sc->trajPoints->size() - 2][0],
                                                                (*sc->trajPoints)[sc->trajPoints->size() - 2][1]) < minTrajPointDist)
            {
                // Replace last point until distance is above threshold
                sc->trajPoints->back() = osg::Vec3f(static_cast<float>(static_cast<double>(state->pos.x) - viewer->origin_[0]),
                                                    static_cast<float>(static_cast<double>(state->pos.y) - viewer->origin_[1]),
                                                    state->pos.z + static_cast<float>(z_offset));
            }
            else
            {
                sc->trajPoints->push_back(osg::Vec3f(static_cast<float>(static_cast<double>(state->pos.x) - viewer->origin_[0]),
                                                     static_cast<float>(static_cast<double>(state->pos.y) - viewer->origin_[1]),
                                                     state->pos.z + static_cast<float>(z_offset)));
            }
        }

        // calculate odometer
        odo_entry    = odo_info[sc->id];
        double delta = GetLengthOfLine2D(odo_entry.x, odo_entry.y, state->pos.x, state->pos.y);
        odo_entry.x  = state->pos.x;
        odo_entry.y  = state->pos.y;
        odo_entry.odometer += delta;
        odo_info[sc->id] = odo_entry;  // save updated odo info for next calculation

        entry->odometer = odo_entry.odometer;  // update odometer
    }

    for (int i = 0; i < static_cast<int>(scenarioEntity.size()); i++)
    {
        osg::Vec4 color;
        if (scenarioEntity[static_cast<unsigned int>(i)].id == 0)
        {
            color = osg::Vec4d(0.9, 0.8, 0.75, 1.0);
        }
        else
        {
            // color = osg::Vec4d(0.9, 0.3, 0.2, 1.0);
            color = osg::Vec4d(0.9, 0.7, 0.3, 1.0);
        }
        scenarioEntity[static_cast<unsigned int>(i)].trajectory =
            viewer->AddPolyLine(viewer, viewer->trajectoryLines_, scenarioEntity[static_cast<unsigned int>(i)].trajPoints, color, width);
    }

    return 0;
}

int GetGhostIdx()
{
    for (size_t i = 0; i < scenarioEntity.size(); i++)
    {
        if (scenarioEntity[i].name.find("_ghost") != std::string::npos)
        {
            return static_cast<int>(i);
        }
    }
    return -1;  // No ghost
}

namespace replayer
{

    Replay* GetReplay()
    {
        // assume invoked only from main thread.
        auto ptr = g_player.lock();
        return ptr.get();
    }

    void ReportKeyEvent(viewer::KeyEvent* keyEvent, void* data, float* time)
    {
        Replay* player = static_cast<Replay*>(data);

        if (keyEvent->down_)
        {
            if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Right))
            {
                if (keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_CTRL) &&
                    keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_SHIFT))
                {
                    player->GoToDeltaTime(JUMP_DELTA_TIME_LARGE);
                }
                else if (keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_SHIFT))
                {
                    player->GoToDeltaTime(JUMP_DELTA_TIME_SMALL);
                }
                else if (keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_CTRL))
                {
                    player->GoToEnd();
                }
                else
                {
                    player->GoToNextFrame();
                }

                pause_player = true;  // step by step
            }
            else if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Left))
            {
                if (keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_CTRL) &&
                    keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_SHIFT))
                {
                    player->GoToDeltaTime(-JUMP_DELTA_TIME_LARGE);
                }
                else if (keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_SHIFT))
                {
                    player->GoToDeltaTime(-JUMP_DELTA_TIME_SMALL);
                }
                else if (keyEvent->modKeyMask_ & static_cast<int>(ModKeyMask::MODKEY_CTRL))
                {
                    // rewind to beginning
                    player->GoToStart();
                }
                else
                {
                    player->GoToPreviousFrame();
                }

                pause_player = true;  // step by step
            }
        }
        if (!keyEvent->down_)
        {
            if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Space))
            {
                pause_player = !pause_player;
            }
            // else if (keyEvent->key_ == 'H')
            // {
            //     puts(helpText);
            // }
            else if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Up))
            {
                time_scale = MIN(100, time_scale * TIME_SCALE_FACTOR);
            }
            else if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Down))
            {
                time_scale = MAX(0.01, time_scale / TIME_SCALE_FACTOR);
            }
            else if (keyEvent->key_ == static_cast<int>('g'))
            {
                no_ghost_model = !no_ghost_model;
                ShowGhosts(player, !no_ghost_model);
            }
        }
        *time = player->GetTime();
    }

    int Run(int argc, char** argv, viewer::StudioViewer* viewer, float* time, bool* quit)
    {
        roadmanager::OpenDrive* odrManager;
        std::shared_ptr<Replay> player;
        double                  simTime   = 0;
        double                  view_mode = viewer::NodeMask::NODE_MASK_ENTITY_MODEL;
        bool                    overlap   = false;
        static char             info_str_buf[256];
        std::string             arg_str;

        // use common options parser to manage the program arguments
        SE_Options& opt = SE_Env::Inst().GetOptions();
        opt.Reset();
        scenarioEntity.clear();
        removeObjects.clear();
        pause_player = true;

        opt.AddOption("file", "Simulation recording data file (.dat)", "filename");
        opt.AddOption("aa_mode", "Anti-alias mode=number of multisamples (subsamples, 0=off, 4=default)", "mode");
        opt.AddOption("capture_screen", "Continuous screen capture. Warning: Many jpeg files will be created");
        opt.AddOption("collision", "Pauses the replay if the ego collides with another entity");
        opt.AddOption("dir",
                      "Directory containing replays to overlay, pair with \"file\" argument, where \"file\" is .dat filename match substring",
                      "path");
        opt.AddOption("fixed_timestep", "Run simulation decoupled from realtime, with specified timesteps", "timestep");
        opt.AddOption("ground_plane", "Add a large flat ground surface");
        opt.AddOption("hide_trajectories", "Hide trajectories from start (toggle with key 'n')");
        opt.AddOption("info_text", "Show on-screen info text (toggle key 'i') mode 0=None 1=current (default) 2=per_object 3=both", "mode");
        opt.AddOption("no_ghost", "Remove ghost entities");
        opt.AddOption("no_ghost_model", "Remove only ghost model, show trajectory (toggle with key 'g')");
        opt.AddOption("osg_screenshot_event_handler", "Revert to OSG default jpg images ('c'/'C' keys handler)");
        opt.AddOption("path", "Search path prefix for assets, e.g. model_ids.txt file (multiple occurrences supported)", "path");
        opt.AddOption("remove_object", "Remove object(s). Multiple ids separated by comma, e.g. 2,3,4.", "id");
        opt.AddOption("repeat", "loop scenario");
        opt.AddOption("res_path", "Path to resources root folder - relative or absolut", "path");
        opt.AddOption("road_features", "Show OpenDRIVE road features");
        opt.AddOption("save_merged", "Save merged data into one dat file, instead of viewing", "filename");
        opt.AddOption("start_time", "Start playing at timestamp", "ms");
        opt.AddOption("stop_time", "Stop playing at timestamp (set equal to time_start for single frame)", "ms");
        opt.AddOption("time_scale", "Playback speed scale factor (1.0 == normal)", "factor");
        opt.AddOption("view_mode", "Entity visualization: \"model\"(default)/\"boundingbox\"/\"both\"", "view_mode");
        opt.AddOption("use_signs_in_external_model", "When external scenegraph 3D model is loaded, skip creating signs from OpenDRIVE");

        if (opt.ParseArgs(argc, argv) != 0 || argc < 2)
        {
            opt.PrintUsage();
#ifdef _USE_OSG
            viewer::StudioViewer::PrintUsage();
#endif
            return -1;
        }

        if (opt.GetOptionArg("file").empty())
        {
            printf("Missing file argument\n");
            opt.PrintUsage();
#ifdef _USE_OSG
            viewer::StudioViewer::PrintUsage();
#endif
            return -1;
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

        arg_str = opt.GetOptionArg("res_path");
        if (!arg_str.empty())
        {
            SE_Env::Inst().AddPath(arg_str);
        }

        if (opt.GetOptionSet("use_signs_in_external_model"))
        {
            LOG_WARN("Use sign models in external scene graph model, skip creating sign models");
        }

        double fixed_timestamp = -1.0;
        auto   timestamp_str   = opt.GetOptionArg("fixed_timestep");
        if (!timestamp_str.empty())
        {
            double timestep = std::stod(timestamp_str);
            if (timestep > SMALL_NUMBER)
            {
                fixed_timestamp = timestep;
            }
        }

        // Create player
        arg_str = opt.GetOptionArg("dir");

        std::string save_merged = opt.GetOptionArg("save_merged");  // name of new dat file
        try
        {
            if (!arg_str.empty())
            {
                player   = std::make_shared<Replay>(arg_str, opt.GetOptionArg("file"), save_merged);
                g_player = player;

                if (!save_merged.empty())
                {
                    LOG_INFO("Merged data saved in {}", save_merged);
                    return 0;
                }
            }
            else
            {
                if (!save_merged.empty())
                {
                    LOG_ERROR("\"--saved_merged\" works only in combination with \"--dir\" argument, combining multiple dat files");
                    return -1;
                }
                player   = std::make_shared<Replay>(opt.GetOptionArg("file"), true);
                g_player = player;
            }
        }
        catch (const std::exception& e)
        {
            LOG_ERROR("Exception: ", e.what());
            return -1;
        }

        try
        {
#if 0
        if (strcmp(player->header_.odr_filename, ""))
        {
            // find and OpenDRIVE file. Test some combinations of paths and filename
            std::vector<std::string> file_name_candidates;

            // just filepath as stated in .dat file
            file_name_candidates.push_back(player->header_.odr_filename);

            // Check registered paths
            for (size_t i = 0; i < SE_Env::Inst().GetPaths().size(); i++)
            {
                // Including file path
                file_name_candidates.push_back(CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], player->header_.odr_filename));

                // Excluding file path
                file_name_candidates.push_back(
                    CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i], FileNameOf(player->header_.odr_filename)));

                // Including file path and xodr sub folder
                file_name_candidates.push_back(
                    CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i] + "/xodr/", FileNameOf(player->header_.odr_filename)));

                // Excluding file path but add xodr sub folder
                file_name_candidates.push_back(
                    CombineDirectoryPathAndFilepath(SE_Env::Inst().GetPaths()[i] + "/xodr/", player->header_.odr_filename));
            }

            size_t i;
            for (i = 0; i < file_name_candidates.size(); i++)
            {
                if (FileExists(file_name_candidates[i].c_str()))
                {
                    if (roadmanager::Position::LoadOpenDrive(file_name_candidates[i].c_str()))
                    {
                        break;
                    }
                }
            }

            if (i == file_name_candidates.size())
            {
                printf("Failed to load OpenDRIVE file %s. Tried:\n", player->header_.odr_filename);
                for (int j = 0; j < static_cast<int>(file_name_candidates.size()); j++)
                {
                    printf("   %s\n", file_name_candidates[static_cast<unsigned int>(j)].c_str());
                }
                printf("continue without road description\n");
            }
        }
#endif

            odrManager = roadmanager::Position::GetOpenDrive();

            osg::ArgumentParser arguments(&argc, argv);

            if (viewer == nullptr)
            {
                printf("Failed to create viewer");
                return -1;
            }

            if ((arg_str = opt.GetOptionArg("info_text")) != "")
            {
                int mask = strtoi(arg_str);
                if (mask < 0 || mask > 3)
                {
                    LOG_ERROR_AND_QUIT("Invalid on-screen info mode {}. Valid range is 0-3", mask);
                }
                viewer->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_INFO | viewer::NodeMask::NODE_MASK_INFO_PER_OBJ,
                                        mask * viewer::NodeMask::NODE_MASK_INFO);
            }

            // viewer->RegisterKeyEventCallback(ReportKeyEvent, player.get());

            if (opt.HasUnknownArgs())
            {
                opt.PrintUnknownArgs("Unrecognized arguments:");
                opt.PrintUsage();
#ifdef _USE_OSG
                viewer::StudioViewer::PrintUsage();
#endif
                return -1;
            }
            // viewer->SetWindowTitle("esmini - " + FileNameWithoutExtOf(argv[0]) + " " + (FileNameOf(opt.GetOptionArg("file"))));

            __int64 now           = 0;
            __int64 lastTimeStamp = 0;

            if (opt.GetOptionSet("time_scale"))
            {
                time_scale = atof(opt.GetOptionArg("time_scale").c_str());
                if (time_scale < SMALL_NUMBER)
                {
                    time_scale = SMALL_NUMBER;
                }
            }

            if (opt.GetOptionSet("repeat"))
            {
                player->SetRepeat(true);
            }

            if (opt.GetOptionSet("capture_screen"))
            {
                LOG_INFO("Activate continuous screen capture");
                viewer->SaveImagesToFile(-1);
            }

            if (opt.GetOptionSet("road_features"))
            {
                viewer->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_ODR_FEATURES);
            }
            else
            {
                viewer->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_ODR_FEATURES, 0x0);
            }

            // Set visual representation of entities
            std::string view_mode_string = opt.GetOptionArg("view_mode");
            if (view_mode_string == "boundingbox")
            {
                view_mode = viewer::NodeMask::NODE_MASK_ENTITY_BB;
            }
            else if (view_mode_string == "both")
            {
                view_mode = viewer::NodeMask::NODE_MASK_ENTITY_MODEL | viewer::NodeMask::NODE_MASK_ENTITY_BB;
            }
            viewer->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_ENTITY_MODEL | viewer::NodeMask::NODE_MASK_ENTITY_BB, static_cast<int>(view_mode));

            if (opt.GetOptionSet("no_ghost"))
            {
                no_ghost = true;
            }

            if (opt.GetOptionSet("no_ghost_model"))
            {
                no_ghost_model = true;
            }

            if (opt.GetOptionSet("remove_object"))
            {
                std::string ids       = opt.GetOptionArg("remove_object");
                std::string delimiter = ",";
                size_t      pivot     = 0;
                size_t      pos       = 0;
                int         id;
                do
                {
                    pos = ids.find(delimiter, pivot);
                    if (pos != std::string::npos)
                    {
                        id = strtoi(ids.substr(pivot, pos - pivot));
                    }
                    else
                    {
                        id = strtoi(ids.substr(pivot));
                    }

                    printf("Removing object[%d]\n", id);
                    removeObjects.push_back(id);
                    pivot = pos + delimiter.length();

                } while (pos != std::string::npos);
            }

            if (ParseEntities(viewer, player.get()) != 0)
            {
                delete viewer;
                return -1;
            }

            const int ghost_id = GetGhostIdx();

            /* TODO: Some functionality to distinguish "main replay" from variations
            for (size_t i = 0; i < viewer->entities_.size(); i++)
            {
                    if (i % player->GetNumberOfScenarios() != 0)
                    {
                            viewer->entities_[i]->SetTransparency(0.5);
                    }
            }
            */

            if (opt.GetOptionSet("hide_trajectories"))
            {
                viewer->ClearNodeMaskBits(viewer::NodeMask::NODE_MASK_TRAJECTORY_LINES);
            }

            std::string start_time_str = opt.GetOptionArg("start_time");
            if (!start_time_str.empty())
            {
                double startTime = 1E-3 * strtod(start_time_str);
                if (static_cast<float>(startTime) < player->data_[0].state.info.timeStamp)
                {
                    printf("Specified start time (%.2f) < first timestamp (%.2f), adapting.\n",
                           startTime,
                           static_cast<double>(player->data_[0].state.info.timeStamp));
                    startTime = static_cast<double>(player->data_[0].state.info.timeStamp);
                }
                else if (static_cast<float>(startTime) > player->data_.back().state.info.timeStamp)
                {
                    printf("Specified start time (%.2f) > last timestamp (%.2f), adapting.\n",
                           startTime,
                           static_cast<double>(player->data_.back().state.info.timeStamp));
                    startTime = static_cast<double>(player->data_.back().state.info.timeStamp);
                }
                player->SetStartTime(startTime);
                player->GoToTime(startTime);
            }
            player->GoToTime(*time);

            std::string stop_time_str = opt.GetOptionArg("stop_time");
            if (!stop_time_str.empty())
            {
                double stopTime = 1E-3 * strtod(stop_time_str);
                if (static_cast<float>(stopTime) > player->data_.back().state.info.timeStamp)
                {
                    printf("Specified stop time (%.2f) > last timestamp (%.2f), adapting.\n",
                           stopTime,
                           static_cast<double>(player->data_.back().state.info.timeStamp));
                    stopTime = static_cast<double>(player->data_.back().state.info.timeStamp);
                }
                else if (static_cast<float>(stopTime) < player->data_[0].state.info.timeStamp)
                {
                    printf("Specified stop time (%.2f) < first timestamp (%.2f), adapting.\n",
                           stopTime,
                           static_cast<double>(player->data_[0].state.info.timeStamp));
                    stopTime = static_cast<double>(player->data_[0].state.info.timeStamp);
                }
                player->SetStopTime(stopTime);
            }

            bool col_analysis = false;
            if (opt.GetOptionSet("collision"))
            {
                col_analysis = true;
            }

            while (!(viewer->osgViewer_->done()) && !(*quit))
            {
                bool time_updated = false;
                if (player->GetTime() != *time)
                {
                    player->GoToTime(*time);
                    time_updated = true;
                }

                simTime = player->GetTime();  // potentially wrapped for repeat

                double targetSimTime = simTime;

                if (!pause_player)
                {
                    if (simTime + SMALL_NUMBER > player->GetStopTime())
                        return -1;  // continue previewing
                    if (viewer->GetSaveImagesToFile())
                    {
                        player->GoToNextFrame();
                    }
                    else
                    {
                        if (fixed_timestamp > 0.0)
                        {
                            deltaSimTime  = fixed_timestamp;
                            targetSimTime = simTime + deltaSimTime;
                        }
                        else
                        {
                            // Get milliseconds since Jan 1 1970
                            now           = SE_getSystemTime();
                            deltaSimTime  = static_cast<double>(now - lastTimeStamp) / 1000.0;  // step size in seconds
                            lastTimeStamp = now;
                            if (deltaSimTime > maxStepSize)  // limit step size
                            {
                                deltaSimTime = maxStepSize;
                            }
                            else if (deltaSimTime < minStepSize)  // avoid CPU rush, sleep for a while
                            {
                                SE_sleep(static_cast<unsigned int>(minStepSize - deltaSimTime));
                                deltaSimTime = minStepSize;
                            }
                            deltaSimTime *= time_scale;
                            targetSimTime = simTime + deltaSimTime;
                        }
                    }
                }

                do
                {
                    if (!(pause_player || viewer->GetSaveImagesToFile()))
                    {
                        player->GoToDeltaTime(deltaSimTime, true);
                        simTime = player->GetTime();  // potentially wrapped for repeat
                    }

                    // Fetch states of scenario objects
                    ReplayEntry*          entry = nullptr;
                    ObjectStateStructDat* state = nullptr;
                    for (int index = 0; index < static_cast<int>(scenarioEntity.size()); index++)
                    {
                        ScenarioEntity* sc = &scenarioEntity[static_cast<unsigned int>(index)];

                        entry = player->GetEntry(sc->id);
                        if (entry)
                        {
                            state = &entry->state;
                        }
                        else
                        {
                            state = nullptr;
                        }

                        if (state == nullptr || (state->info.visibilityMask & 0x01) == 0)  // no state for given object (index) at this timeframe
                        {
                            setEntityVisibility(index, false);

                            // if (index == viewer->currentCarInFocus_)
                            // {
                            //     // Update overlay info text
                            //     snprintf(info_str_buf,
                            //              sizeof(info_str_buf),
                            //              "Time: %.2f s",
                            //              simTime);
                            //     viewer->SetInfoText(info_str_buf);
                            // }
                            if (state == nullptr)
                            {
                                continue;
                            }
                        }
                        else
                        {
                            setEntityVisibility(index, true);
                        }

                        // If not available, create it
                        if (sc == 0)
                        {
                            throw std::runtime_error(std::string("Unexpected entity found: ").append(std::to_string(state->info.id)));
                        }

                        sc->pos            = state->pos;
                        sc->wheel_angle    = state->info.wheel_angle;
                        sc->wheel_rotation = state->info.wheel_rot;

                        // on screen text following each entity
                        if (strcmp(state->info.name, "ego+") != 0)
                        {
                            if (state->info.speed == 0.0)
                                snprintf(sc->entityModel->on_screen_info_.string_,
                                         sizeof(sc->entityModel->on_screen_info_.string_),
                                         " %s ",
                                         state->info.name);
                            else
                                snprintf(sc->entityModel->on_screen_info_.string_,
                                         sizeof(sc->entityModel->on_screen_info_.string_),
                                         " %s [%.1f km/h]",
                                         state->info.name,
                                         3.6 * static_cast<double>(state->info.speed));
                            sc->entityModel->on_screen_info_.osg_text_->setText(sc->entityModel->on_screen_info_.string_);
                        }

                        // if (index == viewer->currentCarInFocus_)
                        // {
                        //     // Update overlay info text
                        //     snprintf(info_str_buf,
                        //              sizeof(info_str_buf),
                        //              "Time: %.2fs",
                        //              simTime);
                        //     viewer->SetInfoText(info_str_buf);
                        // }
                    }

                    if (col_analysis && scenarioEntity.size() > 1)
                    {
                        state = player->GetState(scenarioEntity[0].id);
                        if (state && state->info.visibilityMask != 0)  // skip if Ego invisible for graphics, traffic and sensors
                        {
                            for (size_t i = 0; i < scenarioEntity.size(); i++)
                            {
                                if (static_cast<int>(i) != ghost_id)  // Ignore ghost
                                {
                                    updateCorners(scenarioEntity[i]);
                                }
                            }

                            bool overlap_now = false;
                            for (size_t i = 1; i < scenarioEntity.size(); i++)
                            {
                                state = player->GetState(scenarioEntity[i].id);

                                if (static_cast<int>(i) != ghost_id &&         // Ignore ghost and
                                    state && state->info.visibilityMask != 0)  // and objects invisible for graphics, traffic and sensors
                                {
                                    if (separating_axis_intersect(scenarioEntity[0], scenarioEntity[i]))
                                    {
                                        overlap_now = true;
                                        if (!overlap)
                                        {
                                            overlap          = true;
                                            pause_player     = true;
                                            double rel_speed = abs((player->GetState(scenarioEntity[0].id))->info.speed -
                                                                   (player->GetState(scenarioEntity[i].id)->info.speed)) *
                                                               3.6f;
                                            double rel_angle = static_cast<double>(scenarioEntity[0].pos.h - scenarioEntity[i].pos.h) * 180.0 / M_PI;
                                            LOG_WARN(
                                                "Collision between {} and {} at time {:.2f}\n- Relative speed {:.2f} km/h\n- Angle {:.2f} degrees (ego to target)",
                                                0,
                                                i,
                                                simTime,
                                                rel_speed,
                                                rel_angle);
                                        }
                                    }
                                }
                            }
                            if (!overlap_now)
                            {
                                overlap = false;
                            }
                        }
                    }

                } while (
                    !pause_player && simTime < player->GetStopTime() - SMALL_NUMBER &&  // As long as time is < end
                    simTime > player->GetStartTime() + SMALL_NUMBER &&                  // As long as time is > start time
                    (deltaSimTime < 0 ? (player->GetTime() > targetSimTime) : (player->GetTime() < targetSimTime)));  // until reached target timestep

                *time = player->GetTime();

                // Visualize scenario cars
                for (size_t j = 0; j < scenarioEntity.size(); j++)
                {
                    ScenarioEntity* c = &scenarioEntity[j];
                    if (c->entityModel != nullptr)
                    {
                        c->entityModel->SetPosition(c->pos.x, c->pos.y, c->pos.z);
                        c->entityModel->SetRotation(c->pos.h, c->pos.p, c->pos.r);

                        if (c->entityModel->GetType() == viewer::EntityModel::EntityType::VEHICLE)
                        {
                            (static_cast<viewer::CarModel*>(c->entityModel))->UpdateWheels(c->wheel_angle, c->wheel_rotation);
                        }
                    }
                }

                if (!pause_player || time_updated)
                {
                    for (auto* entity : viewer->entities_)
                    {
                        if (entity->name_ == "ego")
                        {
                            auto& pos = entity->txNode_->getPosition();
                            viewer->MoveCameraTo(pos.x(), pos.y());
                            break;
                        }
                    }
                }

                // Update graphics
                viewer->osgViewer_->frame();
            }
        }
        catch (std::logic_error& e)
        {
            printf("%s\n", e.what());
            return 2;
        }
        catch (std::runtime_error& e)
        {
            printf("%s\n", e.what());
            return 3;
        }

        return 0;
    }
}  // namespace replayer
