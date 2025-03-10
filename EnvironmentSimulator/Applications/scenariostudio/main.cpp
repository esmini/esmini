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

#define _USE_MATH_DEFINES
#include <math.h>
#include <signal.h>

#include "playerbase.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "Replayer.hpp"
#include "StudioDataModel.hpp"
#include "StudioViewer.hpp"

#include <experimental/filesystem>
#include <chrono>

std::string                           g_xodr_path;
static std::string                    g_model_filename;
static SE_Options*                    g_opt_ptr = nullptr;
static std::vector<std::string>       g_args;
static unsigned int                   fixed_timestep = 50;
StudioDataModel*                      g_data_model   = nullptr;
std::unique_ptr<viewer::StudioViewer> g_viewer;

const double default_zoom_dist      = 40000;
const float  default_timeline_range = 15.0f;

static bool g_quit = false;

static void SwitchFromViewerToInspector()
{
    g_data_model->prev_mode_ = g_data_model->mode_;
    g_data_model->mode_      = StudioMode::INSPECTOR;
    g_viewer->osgViewer_->setDone(true);
}

void FetchKeyEvent(viewer::KeyEvent* keyEvent, void*)
{
    if (g_data_model)
    {
        if (g_data_model->mode_ == StudioMode::VIEWER)
        {
            if (g_data_model->virtual_time_ > 0.1f && !keyEvent->down_ && keyEvent->key_ == static_cast<int>(KeyType::KEY_Space))
            {
                g_data_model->virtual_time_ -= 0.05;
                SwitchFromViewerToInspector();
                return;
            }
        }
        else if (g_data_model->mode_ == StudioMode::INSPECTOR)
        {
            Replay* replay = replayer::GetReplay();
            if (replay)
                replayer::ReportKeyEvent(keyEvent, replay, &g_data_model->virtual_time_);
            return;
        }
    }
}

int process_args(int argc, char** argv)
{
    SE_Options& opt = SE_Env::Inst().GetOptions();
    g_opt_ptr       = &opt;
    opt.Reset();

    SE_Env::Inst().AddPath(DirNameOf(argv[0]));  // Add location of exe file to search paths

    g_args.clear();
    for (int i = 0; i < argc; i++)
        g_args.push_back(argv[i]);

    // use an ArgumentParser object to manage the program arguments.
    opt.AddOption("help", "Show this help message");
    opt.AddOption("osc", "OpenSCENARIO filename (required)", "osc_filename");
    opt.AddOption("odr", "OpenDRIVE filename (required)", "odr_filename");
    opt.AddOption("aa_mode", "Anti-alias mode=number of multisamples (subsamples, 0=off, 4=default)", "mode");
    opt.AddOption("capture_screen", "Continuous screen capture. Warning: Many .tga files will be created");
    opt.AddOption("enforce_generate_model", "Generate road 3D model even if --model is specified");
    opt.AddOption("disable_log", "Prevent logfile from being created");
    opt.AddOption("disable_off_screen", "Disable esmini off-screen rendering, revert to OSG viewer default handling");
    opt.AddOption("disable_stdout", "Prevent messages to stdout");
    opt.AddOption("generate_no_road_objects", "Do not generate any OpenDRIVE road objects (e.g. when part of referred 3D model)");
    opt.AddOption("ground_plane", "Add a large flat ground surface");
    opt.AddOption("logfile_path", "logfile path/filename, e.g. \"../esmini.log\" (default: log.txt)", "path");
    opt.AddOption("model", "3D Model filename", "model_filename");
    opt.AddOption("osg_screenshot_event_handler", "Revert to OSG default jpg images ('c'/'C' keys handler)");
    opt.AddOption("path", "Search path prefix for assets, e.g. car and sign model files", "path");
    opt.AddOption("road_features", "Show OpenDRIVE road features (toggle during simulation by press 'o') ");
    opt.AddOption("save_generated_model", "Save generated 3D model (n/a when a scenegraph is loaded)");
    opt.AddOption("use_signs_in_external_model", "When external scenegraph 3D model is loaded, skip creating signs from OpenDRIVE");

    if (opt.ParseArgs(argc, argv) != 0)
    {
        opt.PrintUsage();
        return -1;
    }

    if (opt.GetOptionSet("help"))
    {
        opt.PrintUsage();
        viewer::Viewer::PrintUsage();
        return 0;
    }

    std::string arg_str;

    if ((arg_str = opt.GetOptionArg("path")) != "")
    {
        SE_Env::Inst().AddPath(arg_str);
        LOG_INFO("Added path {}", arg_str);
    }

    g_xodr_path = opt.GetOptionArg("odr");
    if (g_xodr_path.empty())
        LOG_INFO("No command argument --odr found.\n");

    g_model_filename = opt.GetOptionArg("model");

    if (opt.GetOptionSet("use_signs_in_external_model"))
        LOG_WARN("Use sign models in external scene graph model, skip creating sign models");

    if (opt.HasUnknownArgs())
    {
        opt.PrintUnknownArgs("Unrecognized arguments:");
        opt.PrintUsage();
    }

    return 0;
}

void add_default_command_args(std::vector<const char*>* args)
{
#ifdef __linux__
    static std::vector<const char*> additional_args = {"--window", "0", "0", "1920", "1080", "--SingleThreaded"};
#else
    static std::vector<const char*> additional_args = {"--window", "0", "30", "1680", "957", "--SingleThreaded", "--path", "../../../../resources"};
#endif
    args->insert(args->end(), additional_args.begin(), additional_args.end());
}

int run(int argc, char** argv)
{
    // osg::ArgumentParser modifies given args array so we copy it for safe modification
    std::vector<const char*> args = {argv, std::next(argv, argc)};
    add_default_command_args(&args);
    int                 arg_count = static_cast<int>(args.size());
    osg::ArgumentParser arguments(&arg_count, (char**)args.data());
    int                 ret = process_args(arg_count, (char**)args.data());

    auto t0 = std::chrono::system_clock::now();

    if (g_xodr_path.empty())
    {
        LOG_INFO("Try to load the default OpenDRIVE file...");
        g_xodr_path = "../resources/xodr/e6mini.xodr";
        if (!std::experimental::filesystem::exists(g_xodr_path))
            g_xodr_path = "resources/xodr/e6mini.xodr";
        if (!roadmanager::Position::LoadOpenDrive(g_xodr_path.c_str()))
        {
            LOG_INFO("Failed to load OpenDRIVE file {}", g_xodr_path);
            g_xodr_path.clear();
        }
    }
    else if (!roadmanager::Position::LoadOpenDrive(g_xodr_path.c_str()))
    {
        LOG_INFO("Failed to load OpenDRIVE file {}", g_xodr_path);
        g_xodr_path.clear();
    }

    if (!g_xodr_path.empty())
    {
        double duration = std::chrono::duration<double>(std::chrono::system_clock::now() - t0).count();
        LOG_INFO("Finished loading OpenDRIVE file {} in {:.1f} seconds.\n", g_xodr_path, duration);
    }

    roadmanager::OpenDrive* odrManager = roadmanager::Position::GetOpenDrive();

    // osgViewer_->setThreadingModel(osgViewer::ViewerBase::ThreadingModel::SingleThreaded);

    g_viewer =
        std::make_unique<viewer::StudioViewer>(odrManager, g_model_filename.c_str(), /* scenarioFilename= */ nullptr, argv[0], arguments, g_opt_ptr);
    g_viewer->Realize();
    g_data_model = g_viewer->GetDataModel();
    if (!g_xodr_path.empty())
        g_data_model->SetXodrPath(g_xodr_path);
    args.resize(arg_count);

    g_viewer->RegisterKeyEventCallback(FetchKeyEvent, nullptr);

    // Use top view and pause the simulaiton
    g_viewer->SetCameraMode(osgGA::RubberbandManipulator::RB_MODE_TOP);
    // run_state.pause = true;
    g_viewer->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_INFO_PER_OBJ);

    g_viewer->SetWindowTitle("Scenario Studio");
    g_viewer->MoveCameraToMapCenter();
    g_viewer->SetCameraDistance(default_zoom_dist);

    std::unique_ptr<ScenarioPlayer> player;

    auto prev_mode                        = g_data_model->mode_;
    g_data_model->virtual_time_max_value_ = default_timeline_range;
    while (g_data_model->mode_ != StudioMode::ZOMBIE)
    {
        g_viewer->Cleanup();
        auto old_mode = g_data_model->mode_;
        if (g_data_model->mode_ == StudioMode::VIEWER)
        {
            // preview mode with esmini scenario engine
            if (g_data_model->prev_mode_ != StudioMode::INSPECTOR)
            {
                g_data_model->virtual_time_max_value_ = default_timeline_range;
                g_data_model->tmp_rec_path_           = "tmp.rec";
                std::vector<const char*> args         = {"preview",
                                                         "--fixed_timestep",
                                                         "0.05",
                                                         "--osc",
                                                         g_data_model->tmp_xosc_path_.c_str(),
                                                         "--record",
                                                         g_data_model->tmp_rec_path_.c_str()};
                add_default_command_args(&args);

                int arg_count = static_cast<int>(args.size());
                player        = std::make_unique<ScenarioPlayer>(arg_count, (char**)args.data());

                player->RegisterExternalViewer(g_viewer.get());
                if (player->Init() != 0)
                {
                    printf("Failed to init ScenarioPlayer!\n");
                    return -1;
                }
            }

            if (g_data_model->prev_mode_ != StudioMode::VIEWER)
                g_viewer->SetWindowTitle("Scenario Studio (Viewer)");

            g_viewer->osgViewer_->setDone(false);
            g_data_model->virtual_time_manipulated_ = false;

            __int64 time_stamp = 0;
            int     retval     = 0;
            while (!g_viewer->osgViewer_->done() && !g_quit)
            {
                double dt;
                if (player->GetFixedTimestep() > SMALL_NUMBER)
                    dt = player->GetFixedTimestep();
                else
                    dt = SE_getSimTimeStep(time_stamp, player->minStepSize, player->maxStepSize);

                if (!player->IsPaused())
                {
                    for (auto* entity : g_viewer->entities_)
                    {
                        if (entity->name_ == "ego")
                        {
                            auto& pos = entity->txNode_->getPosition();
                            g_viewer->MoveCameraTo(pos.x(), pos.y());
                            break;
                        }
                    }
                }

                retval = player->Frame(dt);

                if (retval)
                    SwitchFromViewerToInspector();

                if (g_data_model->virtual_time_manipulated_)
                {
                    g_data_model->virtual_time_manipulated_ = false;
                    SwitchFromViewerToInspector();
                    break;
                }

                g_data_model->virtual_time_ = player->scenarioEngine->getSimulationTime();
            }

            if (!std::experimental::filesystem::exists(g_data_model->tmp_rec_path_))
                g_data_model->tmp_rec_path_.clear();
        }
        else if (g_data_model->mode_ == StudioMode::INSPECTOR)
        {
            // playback mode with replayer
            std::vector<const char*> args2 = {"replay", "--fixed_timestep", "0.05", "--file", g_data_model->tmp_rec_path_.c_str()};
            add_default_command_args(&args2);

            int arg_count2 = static_cast<int>(args2.size());
            g_viewer->osgViewer_->setDone(false);
            g_viewer->SetWindowTitle("Scenario Studio (Inspector)");
            int ret = replayer::Run(arg_count2, (char**)args2.data(), g_viewer.get(), &g_data_model->virtual_time_, &g_quit);
            if (ret < 0)
            {
                g_data_model->prev_mode_ = g_data_model->mode_;
                g_data_model->mode_      = StudioMode::VIEWER;
                g_viewer->osgViewer_->setDone(true);
            }
        }

        if (g_data_model->mode_ == old_mode)
            g_data_model->mode_ = StudioMode::ZOMBIE;
        prev_mode = g_data_model->mode_;
    }
    g_viewer.reset();
    return 0;
}

int main(int argc, char** argv)
{
    run(argc, argv);
}
