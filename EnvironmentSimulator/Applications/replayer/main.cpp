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

#ifdef _USE_OSG
#include "viewer.hpp"
#include "RubberbandManipulator.hpp"
#include "helpText.hpp"
#endif  // _USE_OSG

#include "CommonMini.hpp"
#include "ScenarioGateway.hpp"
#include "collision.hpp"
#include "RoadManager.hpp"
#include "Replay.hpp"
#include "logger.hpp"
#include "Config.hpp"

#include <signal.h>

using namespace scenarioengine;

#define TIME_SCALE_FACTOR     1.1
#define GHOST_CTRL_TYPE       100  // control type 100 indicates ghost
#define JUMP_DELTA_TIME_LARGE 1.0
#define JUMP_DELTA_TIME_SMALL 0.1

static const double                stepSize    = 0.01;
static const double                maxStepSize = 0.1;
static const double                minStepSize = 0.001;
static std::vector<int>            removeObjects;
static bool                        quit_request = false;
static std::vector<ScenarioEntity> scenarioEntity;

static bool pause_player   = false;  // continuous play
static bool no_ghost       = false;
static bool no_ghost_model = false;
#ifdef _USE_OSG
static double          time_scale = 1.0;
static viewer::Viewer* viewer_    = nullptr;
double                 deltaSimTime;  // external - used by Viewer::RubberBandCamera

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

int ShowGhosts(Replay* player, bool show)
{
    ObjectStateStructDat* state = 0;

    for (size_t j = 0; j < scenarioEntity.size(); j++)
    {
        ScenarioEntity* entity = &scenarioEntity[j];
        if (entity == nullptr)
        {
            continue;
        }

        state = player->GetState(entity->id);
        if (state == nullptr)
        {
            continue;
        }

        if (entity->entityModel != nullptr && state->info.ctrl_type == GHOST_CTRL_TYPE)
        {
            entity->entityModel->txNode_->setNodeMask(show ? 0xffffffff : 0x0);
        }
    }
    return 0;
}

void ReportKeyEvent(viewer::KeyEvent* keyEvent, void* data)
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
        else if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Space))
        {
            pause_player = !pause_player;
        }
        else if (keyEvent->key_ == 'H')
        {
            puts(helpText);
        }
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
}
#endif  // _USE_OSG

static void signal_handler(int s)
{
    if (s == SIGINT)
    {
        LOG_INFO("Quit request from user");
        quit_request = true;
    }
}

void log_callback(const char* str)
{
    printf("%s\n", str);
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

int ParseEntities(Replay* player)
{
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
            new_sc.pos            = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0.0f, 0.0f, 0.0f};
            new_sc.wheel_angle    = 0.0f;
            new_sc.wheel_rotation = 0.0f;
            new_sc.name           = state->info.name;
            new_sc.visible        = true;
            new_sc.bounding_box   = state->info.boundingbox;

            odo_entry.x        = state->pos.x;
            odo_entry.y        = state->pos.y;
            odo_entry.odometer = 0.0;
            odo_info.insert(std::make_pair(new_sc.id, odo_entry));  // Set inital odometer value for the entity

#ifdef _USE_OSG
            new_sc.trajectory = nullptr;
            new_sc.trajPoints = 0;

            std::string filename;
            if (state->info.model_id >= 0)
            {
                filename = SE_Env::Inst().GetModelFilenameById(state->info.model_id);
            }

            if ((new_sc.entityModel = viewer_->CreateEntityModel(filename.c_str(),
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
                if (viewer_->AddEntityModel(new_sc.entityModel) != 0)
                {
                    return -1;
                }
            }

            if (state->info.ctrl_type == GHOST_CTRL_TYPE && no_ghost_model)
            {
                new_sc.entityModel->txNode_->setNodeMask(0x0);
            }
#endif  // _USE_OSG

            // Add it to the list of scenario cars
            scenarioEntity.push_back(new_sc);
            sc = &scenarioEntity.back();
        }

#ifdef _USE_OSG
        if (sc->trajPoints == 0)
        {
            sc->trajPoints = new osg::Vec3Array;
        }

        double z_offset = 0.2;

        if (sc->trajPoints->size() == 0)
        {
            sc->trajPoints->push_back(osg::Vec3f(static_cast<float>(static_cast<double>(state->pos.x) - viewer_->origin_[0]),
                                                 static_cast<float>(static_cast<double>(state->pos.y) - viewer_->origin_[1]),
                                                 state->pos.z + static_cast<float>(z_offset)));
        }
        else
        {
            double minTrajPointDist = 1;

            if (sc->trajPoints->size() > 2 && GetLengthOfLine2D(state->pos.x,
                                                                state->pos.y,
                                                                (*sc->trajPoints)[sc->trajPoints->size() - 2][0],
                                                                (*sc->trajPoints)[sc->trajPoints->size() - 2][1]) < minTrajPointDist)
            {
                // Replace last point until distance is above threshold
                sc->trajPoints->back() = osg::Vec3f(static_cast<float>(static_cast<double>(state->pos.x) - viewer_->origin_[0]),
                                                    static_cast<float>(static_cast<double>(state->pos.y) - viewer_->origin_[1]),
                                                    state->pos.z + static_cast<float>(z_offset));
            }
            else
            {
                sc->trajPoints->push_back(osg::Vec3f(static_cast<float>(static_cast<double>(state->pos.x) - viewer_->origin_[0]),
                                                     static_cast<float>(static_cast<double>(state->pos.y) - viewer_->origin_[1]),
                                                     state->pos.z + static_cast<float>(z_offset)));
            }
        }
#endif  // _USE_OSG

        // calculate odometer
        odo_entry    = odo_info[sc->id];
        double delta = GetLengthOfLine2D(odo_entry.x, odo_entry.y, state->pos.x, state->pos.y);
        odo_entry.odometer += delta;
        odo_entry.x      = state->pos.x;
        odo_entry.y      = state->pos.y;
        odo_info[sc->id] = odo_entry;  // save updated odo info for next calculation

        entry->odometer = odo_entry.odometer;  // update odometer
    }

#ifdef _USE_OSG
    double width = 1.75;

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
            viewer_->AddPolyLine(viewer_, viewer_->trajectoryLines_, scenarioEntity[static_cast<unsigned int>(i)].trajPoints, color, width);
    }
#endif  // _USE_OSG

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

int main(int argc, char** argv)
{
    Replay*     player;
    double      simTime = 0;
    std::string arg_str;

    // Setup signal handler to catch Ctrl-C
    signal(SIGINT, signal_handler);

    SE_Env::Inst().AddPath(DirNameOf(argv[0]));  // Add location of exe file to search paths

    // use common options parser to manage the program arguments
    SE_Options& opt = SE_Env::Inst().GetOptions();
    opt.Reset();

    opt.AddOption("file", "Simulation recording data file (.dat)", "filename");
#ifdef _USE_OSG
    opt.AddOption("aa_mode", "Anti-alias mode=number of multisamples (subsamples, 0=off)", "mode", "4");
    opt.AddOption("camera_mode",
                  "Initial camera mode (\"orbit\", \"fixed\", \"flex\", \"flex-orbit\", \"top\", \"driver\", \"custom\"). Toggle key 'k'",
                  "mode",
                  "orbit",
                  true);
    opt.AddOption("capture_screen", "Continuous screen capture. Warning: Many jpeg files will be created");
#endif  // _USE_OSG
    opt.AddOption("collision", "Detect collisions and optionally pauses the replay <pause/continue> (pause is default)", "mode", "pause");
    opt.AddOption(CONFIG_FILE_OPTION_NAME, "Configuration file path/filename, e.g. \"../my_config.txt\"", "path", DEFAULT_CONFIG_FILE, true, false);
#ifdef _USE_OSG
    opt.AddOption("custom_camera", "Additional custom camera position <x,y,z>[,h,p]", "position", "", false, false);
    opt.AddOption("custom_fixed_camera",
                  "Additional custom fixed camera position <x,y,z>[,h,p]",
                  "position and optional orientation",
                  "",
                  false,
                  false);
    opt.AddOption("custom_fixed_top_camera", "Additional custom top camera <x,y,z,rot>", "position and rotation", "", false, false);
#endif  // _USE_OSG
    opt.AddOption("dir",
                  "Directory containing replays to overlay, pair with \"file\" argument, where \"file\" is .dat filename match substring",
                  "path");
#ifdef _USE_OSG
    opt.AddOption("ground_plane", "Add a large flat ground surface");
    opt.AddOption("generate_without_textures", "Do not apply textures on any generated road model (set colors instead as for missing textures)");
#endif  // _USEOSG
    opt.AddOption("headless", "Run without viewer window");
    opt.AddOption("help", "Show this help message (-h works as well)");
#ifdef _USE_OSG
    opt.AddOption("hide_trajectories", "Hide trajectories from start (toggle with key 'n')");
    opt.AddOption("info_text", "Show on-screen info text. Modes: 0=None 1=current 2=per_object 3=both. Toggle key 'i'", "mode", "1", true);
#endif  // _USEOSG
    opt.AddOption("logfile_path", "Logfile path/filename, e.g. \"../my_log.txt\"", "path", REPLAYER_LOG_FILENAME, true);
#ifdef _USE_OSG
    opt.AddOption("no_ghost", "Remove ghost entities");
    opt.AddOption("no_ghost_model", "Remove only ghost model, show trajectory (toggle with key 'g')");
    opt.AddOption("osg_screenshot_event_handler", "Revert to OSG default jpg images ('c'/'C' keys handler)");
#endif  // _USEOSG
    opt.AddOption("path", "Search path prefix for assets, e.g. OpenDRIVE files.", "path", "", false, false);
    opt.AddOption("quit_at_end", "Quit application when reaching end of scenario");
#ifdef _USE_OSG
    opt.AddOption("remove_object", "Remove object(s). Multiple ids separated by comma, e.g. 2,3,4.", "id");
    opt.AddOption("repeat", "loop scenario");
#endif  // _USEOSG
    opt.AddOption("res_path", "Path to resources root folder - relative or absolut", "path");
#ifdef _USE_OSG
    opt.AddOption("road_features", "Show OpenDRIVE road features. Modes: on, off. Toggle key 'o'", "mode", "on");
#endif  // _USEOSG
    opt.AddOption("save_merged", "Save merged data into one dat file, instead of viewing", "filename");
    opt.AddOption("start_time", "Start playing at timestamp", "ms");
    opt.AddOption("stop_time", "Stop playing at timestamp (set equal to time_start for single frame)", "ms");
#ifdef _USE_OSG
    opt.AddOption("text_scale", "Scale screen overlay text", "size factor", "1.0", true);
#endif  // _USEOSG
    opt.AddOption("time_scale", "Playback speed scale factor (1.0 == normal)", "factor");
#ifdef _USE_OSG
    opt.AddOption("view_mode", "Entity visualization: \"model\"(default)/\"boundingbox\"/\"both\"", "view_mode");
    opt.AddOption("use_signs_in_external_model", "When external scenegraph 3D model is loaded, skip creating signs from OpenDRIVE");
#endif  // _USEOSG
    opt.AddOption("version", "Show version and quit");

    if (int ret = OnRequestShowHelpOrVersion(argc, argv, opt); ret > 0)
    {
        return ret;
    }

    int                    argc_;
    char**                 argv_;
    esmini::common::Config config("replayer", argc, argv);
    try
    {
        std::tie(argc_, argv_) = config.Load();
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("Exception: {}", e.what());
        return -1;
    }

    if (opt.ParseArgs(argc_, argv_) != 0 || argc_ < 2)
    {
        opt.PrintUsage();
#ifdef _USE_OSG
        PrintOSGUsage();
#endif  // _USE_OSG
        return -1;
    }

    config.LogLoadedConfigFiles();

    std::string strAllSetOptions = opt.GetSetOptionsAsStr();
    LOG_INFO("replayer options: {}", strAllSetOptions);

    TxtLogger::Inst().LogTimeOnly();
    TxtLogger::Inst().SetLoggerTime(&simTime);

#ifndef _USE_OSG
    LOG_INFO("Compiled with USE_OSG=FALSE, limited functionality available");
#endif  // _USE_OSG

    if (opt.GetOptionArg("file").empty() || argc_ < 2)
    {
        printf("Missing required file argument\n");
        opt.PrintUsage();
#ifdef _USE_OSG
        PrintOSGUsage();
#endif  // _USE_OSG
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
        LOG_INFO("Use sign models in external scene graph model, skip creating sign models");
    }

    // Create player
    arg_str = opt.GetOptionArg("dir");

    std::string save_merged = opt.GetOptionArg("save_merged");  // name of new dat file
    try
    {
        if (!arg_str.empty())
        {
            player = new Replay(arg_str, opt.GetOptionArg("file"), save_merged);

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
            player = new Replay(opt.GetOptionArg("file"), true);
        }
    }
    catch (const std::exception& e)
    {
        LOG_ERROR("Exception: ", e.what());
        return -1;
    }

    try
    {
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
        simTime = player->GetTime();
#ifdef _USE_OSG
        char                    info_str_buf[256];
        double                  targetSimTime = simTime;
        roadmanager::OpenDrive* odrManager    = roadmanager::Position::GetOpenDrive();
        osg::ArgumentParser     arguments(&argc_, argv_);
        viewer_ = new viewer::Viewer(odrManager, player->header_.model_filename, NULL, argv_[0], arguments, &opt);

        if (viewer_ == nullptr)
        {
            printf("Failed to create viewer");
            return -1;
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
            else
            {
                LOG_WARN("Unsupported camera mode: {} - using default (orbit)", arg_str);
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
                    viewer_->AddCustomCamera(strtod(splitted[0]), strtod(splitted[1]), strtod(splitted[2]), false);
                    LOG_INFO("Created custom fixed camera {} ({}, {}, {})", counter, splitted[0], splitted[1], splitted[2]);
                }
                else if (splitted.size() == 5)
                {
                    viewer_->AddCustomCamera(strtod(splitted[0]),
                                             strtod(splitted[1]),
                                             strtod(splitted[2]),
                                             strtod(splitted[3]),
                                             strtod(splitted[4]),
                                             false);
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
                viewer_->SetCameraMode(-1);  // activate last camera which is the one just added

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
                    viewer_->AddCustomCamera(strtod(splitted[0]), strtod(splitted[1]), strtod(splitted[2]), true);
                    LOG_INFO("Created custom fixed camera {} ({}, {}, {})", counter, splitted[0], splitted[1], splitted[2]);
                }
                else if (splitted.size() == 5)
                {
                    viewer_->AddCustomCamera(strtod(splitted[0]),
                                             strtod(splitted[1]),
                                             strtod(splitted[2]),
                                             strtod(splitted[3]),
                                             strtod(splitted[4]),
                                             true);
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
                viewer_->SetCameraMode(-1);  // activate last camera which is the one just added
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
                viewer_->AddCustomFixedTopCamera(strtod(splitted[0]), strtod(splitted[1]), strtod(splitted[2]), strtod(splitted[3]));
                viewer_->SetCameraMode(-1);  // activate last camera which is the one just added

                LOG_INFO("Created custom fixed top camera {} ({}, {}, {}, {})", counter, splitted[0], splitted[1], splitted[2], splitted[3]);
                counter++;
            }
        }

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

        viewer_->RegisterKeyEventCallback(ReportKeyEvent, player);
        viewer_->SetWindowTitle("esmini - " + FileNameWithoutExtOf(argv_[0]) + " " + (FileNameOf(opt.GetOptionArg("file"))));

        __int64 now           = 0;
        __int64 lastTimeStamp = 0;

#endif  // _USE_OSG

        if (opt.HasUnknownArgs())
        {
            opt.PrintUnknownArgs("Unrecognized arguments:");
            opt.PrintUsage();
#ifdef _USE_OSG
            PrintOSGUsage();
#endif                  // _USE_OSG
            return -1;  // we harmonize all applications to quit on unknown arguments
        }

        if (opt.GetOptionSet("time_scale"))
        {
#ifdef _USE_OSG
            time_scale = atof(opt.GetOptionArg("time_scale").c_str());
            if (time_scale < SMALL_NUMBER)
            {
                time_scale = SMALL_NUMBER;
            }
#else
            LOG_INFO("Ignoring --time_scale option for non_osg build. Stepping all frames.");
#endif  // _USE_OSG
        }

        if (opt.GetOptionSet("repeat"))
        {
            player->SetRepeat(true);
        }
#ifdef _USE_OSG
        if (opt.GetOptionSet("capture_screen"))
        {
            LOG_INFO("Activate continuous screen capture");
            viewer_->SaveImagesToFile(-1);
        }

        if (opt.GetOptionSet("road_features"))
        {
            viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_ODR_FEATURES);
        }
        else
        {
            viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_ODR_FEATURES, 0x0);
        }

        // Set visual representation of entities
        int         view_mode        = viewer::NodeMask::NODE_MASK_ENTITY_MODEL;
        std::string view_mode_string = opt.GetOptionArg("view_mode");
        if (view_mode_string == "boundingbox")
        {
            view_mode = viewer::NodeMask::NODE_MASK_ENTITY_BB;
        }
        else if (view_mode_string == "both")
        {
            view_mode = viewer::NodeMask::NODE_MASK_ENTITY_MODEL | viewer::NodeMask::NODE_MASK_ENTITY_BB;
        }
        viewer_->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_ENTITY_MODEL | viewer::NodeMask::NODE_MASK_ENTITY_BB, view_mode);

        if (opt.GetOptionSet("hide_trajectories"))
        {
            viewer_->ClearNodeMaskBits(viewer::NodeMask::NODE_MASK_TRAJECTORY_LINES);
        }
#endif  // _USE_OSG

        bool quit_at_end = false;
#ifdef _USE_OSG
        if (opt.GetOptionSet("quit_at_end"))
        {
            quit_at_end = true;
        }
#else
        quit_at_end = true;
#endif  // _USE_OSG

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

        if (ParseEntities(player) != 0)
        {
#ifdef _USE_OSG
            delete viewer_;
#endif  // _USE_OSG
            return -1;
        }

        const int ghost_idx = GetGhostIdx();

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
        bool col_pause =
#ifdef _USE_OSG
            true;
#else
            false;
#endif  // _USE_OSG
        if (opt.GetOptionSet("collision"))
        {
            col_analysis = true;
            if (opt.GetOptionArg("collision") == "continue")
            {
                col_pause = false;
            }
        }

        while (!(
#ifdef _USE_OSG
            viewer_->osgViewer_->done() ||
#endif  // _USE_OSG
            (quit_at_end && simTime >= (player->GetStopTime() - SMALL_NUMBER)) || quit_request == true))
        {
            simTime = player->GetTime();  // potentially wrapped for repeat

            if (!pause_player)
            {
#ifdef _USE_OSG
                if (viewer_->GetSaveImagesToFile())
                {
                    player->GoToNextFrame();
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
#else
                player->GoToNextFrame();
#endif  // _USE_OSG
            }

            do
            {
#ifdef _USE_OSG
                if (!(pause_player || viewer_->GetSaveImagesToFile()))
                {
                    player->GoToDeltaTime(deltaSimTime, true);
                    simTime = player->GetTime();  // potentially wrapped for repeat
                }
#else
                simTime = player->GetTime();  // potentially wrapped for repeat
#endif  // _USE_OSG

                // Fetch states of scenario objects
                ReplayEntry*          entry = nullptr;
                ObjectStateStructDat* state = nullptr;
                for (int index = 0; index < static_cast<int>(scenarioEntity.size()); index++)
                {
                    ScenarioEntity* sc = &scenarioEntity[static_cast<unsigned int>(index)];
                    if (sc == nullptr)
                    {
                        throw std::runtime_error(std::string("Unexpected entity found: ").append(std::to_string(state->info.id)));
                    }

                    entry = player->GetEntry(sc->id);
                    if (entry)
                    {
                        state              = &entry->state;
                        sc->pos            = state->pos;
                        sc->wheel_angle    = state->info.wheel_angle;
                        sc->wheel_rotation = state->info.wheel_rot;
                    }
                    else
                    {
                        state = nullptr;
                    }

#ifdef _USE_OSG
                    if (state == nullptr || (state->info.visibilityMask & 0x01) == 0)  // no state for given object (index) at this timeframe
                    {
                        setEntityVisibility(index, false);

                        if (index == viewer_->currentCarInFocus_)
                        {
                            // Update overlay info text
                            snprintf(info_str_buf,
                                     sizeof(info_str_buf),
                                     "%.3fs entity[%d]: %s (%d) NO INFO",
                                     simTime,
                                     viewer_->currentCarInFocus_,
                                     sc->name.c_str(),
                                     sc->id);
                            viewer_->SetInfoText(info_str_buf);
                        }
                        continue;
                    }
                    setEntityVisibility(index, true);

                    // on screen text following each entity
                    snprintf(sc->entityModel->on_screen_info_.string_,
                             sizeof(sc->entityModel->on_screen_info_.string_),
                             " %s (%d) %.2fm\n %.2fkm/h road %d lane %d/%.2f s %.2f\n x %.2f y %.2f hdg %.2f\n osi x %.2f y %.2f \n|",
                             state->info.name,
                             state->info.id,
                             entry->odometer,
                             3.6 * static_cast<double>(state->info.speed),
                             sc->pos.roadId,
                             sc->pos.laneId,
                             static_cast<double>(fabs(sc->pos.offset)) < SMALL_NUMBER ? 0 : static_cast<double>(sc->pos.offset),
                             static_cast<double>(sc->pos.s),
                             static_cast<double>(sc->pos.x),
                             static_cast<double>(sc->pos.y),
                             static_cast<double>(sc->pos.h),
                             static_cast<double>(sc->pos.x + sc->bounding_box.center_.x_ * cos(sc->pos.h)),
                             static_cast<double>(sc->pos.y + sc->bounding_box.center_.x_ * sin(sc->pos.h)));
                    sc->entityModel->on_screen_info_.osg_text_->setText(sc->entityModel->on_screen_info_.string_);

                    if (index == viewer_->currentCarInFocus_)
                    {
                        // Update overlay info text
                        snprintf(info_str_buf,
                                 sizeof(info_str_buf),
                                 "%.3fs entity[%d]: %s (%d) %.2fs %.2fkm/h %.2fm (%d, %d, %.2f, %.2f)/(%.2f, %.2f %.2f) tScale: %.2f ",
                                 simTime,
                                 viewer_->currentCarInFocus_,
                                 state->info.name,
                                 state->info.id,
                                 static_cast<double>(state->info.timeStamp),
                                 3.6 * static_cast<double>(state->info.speed),
                                 entry->odometer,
                                 sc->pos.roadId,
                                 sc->pos.laneId,
                                 static_cast<double>(fabs(sc->pos.offset)) < SMALL_NUMBER ? 0 : static_cast<double>(sc->pos.offset),
                                 static_cast<double>(sc->pos.s),
                                 static_cast<double>(sc->pos.x),
                                 static_cast<double>(sc->pos.y),
                                 static_cast<double>(sc->pos.h),
                                 time_scale);
                        viewer_->SetInfoText(info_str_buf);
                    }
#endif  // _USE_OSG
                }
                // Collision detection
                if (col_analysis && scenarioEntity.size() > 1)
                {
                    state = player->GetState(scenarioEntity[0].id);
                    if (state && state->info.visibilityMask != 0)  // skip if Ego invisible for graphics, traffic and sensors
                    {
                        for (size_t i = 0; i < scenarioEntity.size(); i++)
                        {
                            if (static_cast<int>(i) != ghost_idx)  // Ignore ghost
                            {
                                updateCorners(scenarioEntity[i]);
                            }
                        }

                        for (size_t i = 0; i < scenarioEntity.size(); i++)
                        {
                            if (static_cast<int>(i) == ghost_idx)
                            {
                                continue;
                            }
                            for (size_t j = i + 1; j < scenarioEntity.size(); j++)
                            {
                                if (static_cast<int>(j) == ghost_idx)
                                {
                                    continue;
                                }

                                if (separating_axis_intersect(scenarioEntity[i], scenarioEntity[j]))
                                {
                                    if (std::find(scenarioEntity[i].overlap_entity_ids.begin(),
                                                  scenarioEntity[i].overlap_entity_ids.end(),
                                                  scenarioEntity[j].id) == scenarioEntity[i].overlap_entity_ids.end())
                                    {
                                        // overlap not registered, do it
                                        scenarioEntity[i].overlap_entity_ids.push_back(scenarioEntity[j].id);
                                        pause_player     = col_pause ? true : false;
                                        double rel_speed = abs((player->GetState(scenarioEntity[i].id))->info.speed -
                                                               (player->GetState(scenarioEntity[j].id)->info.speed)) *
                                                           3.6f;
                                        double rel_angle = static_cast<double>(scenarioEntity[i].pos.h - scenarioEntity[j].pos.h) * 180.0 / M_PI;
                                        LOG_WARN(
                                            "Collision between {} (id {}) and {} (id {}) at time {:.2f}, Relative speed {:.2f} km/h, Angle {:.2f} degrees (ego to target)",
                                            scenarioEntity[i].name,
                                            scenarioEntity[i].id,
                                            scenarioEntity[j].name,
                                            scenarioEntity[j].id,
                                            simTime,
                                            rel_speed,
                                            rel_angle);
                                    }
                                }
                                else if (std::find(scenarioEntity[i].overlap_entity_ids.begin(),
                                                   scenarioEntity[i].overlap_entity_ids.end(),
                                                   scenarioEntity[j].id) != scenarioEntity[i].overlap_entity_ids.end())
                                {
                                    // not overlapping anymore, unregister
                                    scenarioEntity[i].overlap_entity_ids.erase(std::remove(scenarioEntity[i].overlap_entity_ids.begin(),
                                                                                           scenarioEntity[i].overlap_entity_ids.end(),
                                                                                           scenarioEntity[j].id),
                                                                               scenarioEntity[i].overlap_entity_ids.end());
                                }
                            }
                        }
                    }
                }
            } while
#ifdef _USE_OSG
                (!pause_player && simTime < player->GetStopTime() - SMALL_NUMBER  // As long as time is < end
                 && simTime > player->GetStartTime() + SMALL_NUMBER               // As long as time is > start time
                 && (deltaSimTime < 0 ? (player->GetTime() > targetSimTime) : (player->GetTime() < targetSimTime)));  // until reached target timestep
#else
                // for non osg build, full steps only
                (false);
#endif  // _USE_OSG

#ifdef _USE_OSG
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

            // Update graphics
            viewer_->Frame(0.0);
#endif  // _USE_OSG
        }
        delete player;
#ifdef _USE_OSG
        viewer_->renderSemaphore.Release();  // allow rendering thread to finish
        delete viewer_;
#endif  // _USE_OSG
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
