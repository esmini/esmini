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
 * The purpose of this application is to support development of the RoadManager by visualizing the road network and moving objects on top.
 * Bascially it loads an OpenDRIVE file, and optionally a corresponding 3D model, and then populate vehicles at specified density. The
 * vehicles will simply follow it's lane until a potential junction where the choice of route is randomized.
 *
 * The application can be used both to debug the RoadManager and to check OpenDRIVE files, e.g. w.r.t. gemoetry, lanes and connectivity.
 *
 * New road/track segments is indicated by a yellow large dot. Geometry segments within a road are indicated by red dots.
 * Red line is the reference lane, blue lines shows drivable lanes. Non-drivable lanes are currently not indicated.
 */

#include <random>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

#include "viewer.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "helpText.hpp"
#include "logger.hpp"
#include "Utils.h"

#define ROAD_MIN_LENGTH 30.0
#define SIGN(X)         ((X < 0) ? -1 : 1)

static const double stepSize            = 0.01;
static const double maxStepSize         = 0.1;
static const double minStepSize         = 0.01;
static const bool   freerun             = true;
static double       density             = 1.0;  // Cars per 100 m
static double       global_speed_factor = 1.0;
static int          first_car_in_focus  = -1;
static double       fixed_timestep      = -1.0;
static bool         stop_at_end_of_road = false;
static double       duration            = -1.0;

static struct
{
    bool pause = false;
    bool step  = false;
} run_state;

roadmanager::Road::RoadRule rule = roadmanager::Road::RoadRule::ROAD_RULE_UNDEFINED;

double deltaSimTime;  // external - used by Viewer::RubberBandCamera

struct Car
{
    id_t                  road_id_init;
    int                   lane_id_init;
    double                heading_init;
    double                s_init;
    roadmanager::Position pos;
    double                speed_factor;  // speed vary bewtween lanes, m/s
    viewer::EntityModel  *model;
    int                   id;
    bool                  stopped = false;
};

typedef struct
{
    id_t   roadId;
    int    side;
    int    nLanes;
    double s;
} OpenEnd;

std::vector<Car *>   cars;
std::vector<OpenEnd> openEnds;

// Car models used for populating the road network
// path should be relative the OpenDRIVE file
static const char *carModelsFiles_[] = {
    "car_white.osgb",
    "car_blue.osgb",
    "car_red.osgb",
    "car_yellow.osgb",
    "truck_yellow.osgb",
    "van_red.osgb",
    "bus_blue.osgb",
};

std::vector<osg::ref_ptr<osg::LOD>> carModels_;

void FetchKeyEvent(viewer::KeyEvent *keyEvent, void *)
{
    if (keyEvent->down_)
    {
        if (keyEvent->key_ == 'H')
        {
            puts(helpText);
        }

        if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Space))
        {
            run_state.pause = !run_state.pause;
        }

        if (keyEvent->key_ == static_cast<int>(KeyType::KEY_Return))
        {
            run_state.step  = true;
            run_state.pause = true;
        }
    }
}

int SetupCars(roadmanager::OpenDrive *odrManager, viewer::Viewer *viewer)
{
    if (density < 1E-10)
    {
        // Basically no scenario vehicles
        return 0;
    }

    for (int r = 0; r < odrManager->GetNumOfRoads(); r++)
    {
        roadmanager::Road *road             = odrManager->GetRoadByIdx(r);
        double             average_distance = 100.0 / density;

        roadmanager::Road::RoadRule rrule = road->GetRule();
        if (rule != roadmanager::Road::RoadRule::ROAD_RULE_UNDEFINED)
        {
            // Enforce specified rule
            rrule = rule;
        }

        // Check for open end
        OpenEnd                openEnd;
        roadmanager::RoadLink *tmpLink = road->GetLink(roadmanager::LinkType::PREDECESSOR);
        if (tmpLink == nullptr)
        {
            openEnd.roadId = road->GetId();
            openEnd.s      = 0;
            openEnd.side   = rrule == roadmanager::Road::RoadRule::LEFT_HAND_TRAFFIC ? 1 : -1;
            openEnd.nLanes = road->GetNumberOfDrivingLanesSide(openEnd.s, openEnd.side);
            if (openEnd.nLanes > 0)
            {
                openEnds.push_back(openEnd);
            }
        }
        tmpLink = road->GetLink(roadmanager::LinkType::SUCCESSOR);
        if (tmpLink == nullptr)
        {
            openEnd.roadId = road->GetId();
            openEnd.s      = road->GetLength();
            openEnd.side   = rrule == roadmanager::Road::RoadRule::LEFT_HAND_TRAFFIC ? -1 : 1;
            openEnd.nLanes = road->GetNumberOfDrivingLanesSide(openEnd.s, openEnd.side);
            if (openEnd.nLanes > 0)
            {
                openEnds.push_back(openEnd);
            }
        }

        if (road->GetLength() > ROAD_MIN_LENGTH)
        {
            // Populate road lanes with vehicles at some random distances
            for (double s = 10; s < road->GetLength() - average_distance;
                 s += average_distance + 0.2 * average_distance * SE_Env::Inst().GetRand().GetReal())
            {
                // Pick lane by random
                int                n_lanes = road->GetNumberOfDrivingLanes(s);
                roadmanager::Lane *lane    = nullptr;
                if (n_lanes > 0)
                {
                    int lane_idx = SE_Env::Inst().GetRand().GetNumberBetween(0, n_lanes - 1);
                    lane         = road->GetDrivingLaneByIdx(s, lane_idx);
                    if (lane == nullptr)
                    {
                        LOG_ERROR("Spawn: Failed locate driving lane {} at s {:.2f}", lane_idx, s);
                        continue;
                    }
                }
                else
                {
                    LOG_ERROR("Spawn: No driving lanes on road {} at s {:.2f}", road->GetId(), s);
                    continue;
                }

                if (((SIGN(lane->GetId()) < 0) && (road->GetLength() - s < 50) && (road->GetLink(roadmanager::LinkType::SUCCESSOR) == 0)) ||
                    ((SIGN(lane->GetId()) > 0) && (s < 50) && (road->GetLink(roadmanager::LinkType::PREDECESSOR) == 0)))
                {
                    // Skip vehicles too close to road end - and where connecting road is missing
                    continue;
                }

                // randomly choose model
                int carModelID = SE_Env::Inst().GetRand().GetNumberBetween(0, (sizeof(carModelsFiles_) / sizeof(carModelsFiles_[0])) - 1);
                LOG_DEBUG("Adding car of model {} to road nr {} (road id {} s {:.2f} lane id {}), ", carModelID, r, road->GetId(), s, lane->GetId());

                Car *car_ = new Car;
                // Higher speeds in lanes closer to reference lane
                car_->speed_factor = 0.5 + 0.5 / abs(lane->GetId());  // Speed vary between 0.5 to 1.0 times default speed
                car_->road_id_init = odrManager->GetRoadByIdx(r)->GetId();
                car_->lane_id_init = lane->GetId();
                car_->s_init       = s;
                car_->pos.SetLanePos(odrManager->GetRoadByIdx(r)->GetId(), lane->GetId(), s, 0);
                if (rrule == roadmanager::Road::RoadRule::LEFT_HAND_TRAFFIC)
                {
                    car_->pos.SetHeadingRelative(lane->GetId() < 0 ? M_PI : 0);
                }
                else
                {
                    car_->pos.SetHeadingRelative(lane->GetId() < 0 ? 0 : M_PI);
                }
                car_->heading_init = car_->pos.GetHRelative();

                if ((car_->model = viewer->CreateEntityModel(carModelsFiles_[carModelID],
                                                             osg::Vec4(0.5, 0.5, 0.5, 1.0),
                                                             viewer::EntityModel::EntityType::VEHICLE,
                                                             false,
                                                             "",
                                                             0,
                                                             EntityScaleMode::BB_TO_MODEL)) == 0)
                {
                    return -1;
                }
                else
                {
                    if (viewer->AddEntityModel(car_->model) != 0)
                    {
                        return -1;
                    }
                }
                car_->id = static_cast<int>(cars.size());
                cars.push_back(car_);
                if (first_car_in_focus == -1 && lane->GetId() < 0)
                {
                    first_car_in_focus = car_->id;
                }
            }
        }
    }

    if (first_car_in_focus == -1)
    {
        first_car_in_focus = 0;
    }

    return 0;
}

int SetupCarsSpecial(roadmanager::OpenDrive *odrManager, viewer::Viewer *viewer)
{
    (void)odrManager;
    // Setup one single vehicle in a dedicated pos
    Car *car_ = new Car;

    car_->speed_factor = 1.0;
    car_->road_id_init = 1;
    car_->lane_id_init = -1;
    car_->s_init       = 40;
    car_->pos.SetLanePos(car_->road_id_init, car_->lane_id_init, car_->s_init, 0);
    car_->pos.SetHeadingRelative(car_->lane_id_init < 0 ? 0 : M_PI);
    car_->heading_init = car_->pos.GetHRelative();

    if ((car_->model = viewer->CreateEntityModel(carModelsFiles_[0],
                                                 osg::Vec4(0.5, 0.5, 0.5, 1.0),
                                                 viewer::EntityModel::EntityType::VEHICLE,
                                                 false,
                                                 "",
                                                 0,
                                                 EntityScaleMode::BB_TO_MODEL)) == 0)
    {
        return -1;
    }
    else
    {
        if (viewer->AddEntityModel(car_->model) != 0)
        {
            return -1;
        }
    }

    car_->id = static_cast<int>(cars.size());
    cars.push_back(car_);

    first_car_in_focus = 0;

    return 0;
}

void updateCar(roadmanager::OpenDrive *odrManager, Car *car, double dt)
{
    if (car->stopped)
    {
        return;
    }

    double new_speed = car->pos.GetSpeedLimit() * car->speed_factor * global_speed_factor;
    double ds        = new_speed * dt;  // right lane is < 0 in road dir;

    roadmanager::Road::RoadRule rrule = odrManager->GetRoadById(car->pos.GetTrackId())->GetRule();
    if (rule != roadmanager::Road::RoadRule::ROAD_RULE_UNDEFINED)
    {
        // Enforce specified rule
        rrule = rule;
    }

    if (static_cast<int>(car->pos.MoveAlongS(ds)) < 0)
    {
        if (stop_at_end_of_road)
        {
            car->stopped = true;
        }
        else if (openEnds.size() == 0)
        {
            // If no open ends, respawn based on initial position
            // start from beginning of lane section - not initial s-position
            roadmanager::LaneSection *ls      = odrManager->GetRoadById(car->road_id_init)->GetLaneSectionByS(car->s_init);
            double                    start_s = ls->GetS() + 5;
            if (car->lane_id_init > 0)
            {
                start_s = ls->GetS() + ls->GetLength() - 5;
            }
            car->pos.SetLanePos(car->road_id_init, car->lane_id_init, start_s, 0);
            car->pos.SetHeadingRelative(car->heading_init);
        }
        else
        {
            // Choose random open end
            int      oeIndex = SE_Env::Inst().GetRand().GetNumberBetween(0, static_cast<int>(openEnds.size()) - 1);
            OpenEnd *oe      = &openEnds[static_cast<unsigned int>(oeIndex)];
            // Choose random lane
            int                laneIndex = SE_Env::Inst().GetRand().GetNumberBetween(0, oe->nLanes - 1);
            roadmanager::Road *road      = odrManager->GetRoadById(oe->roadId);
            roadmanager::Lane *lane      = road->GetDrivingLaneSideByIdx(oe->s, oe->side, laneIndex);

            car->pos.SetLanePos(road->GetId(), lane->GetId(), oe->s, 0);

            // Ensure car is oriented along lane driving direction
            rrule = road->GetRule();
            if (rule != roadmanager::Road::RoadRule::ROAD_RULE_UNDEFINED)
            {
                // Enforce specified rule
                rrule = rule;
            }
            if (rrule == roadmanager::Road::RoadRule::LEFT_HAND_TRAFFIC)
            {
                car->pos.SetHeadingRelative(SIGN(lane->GetId()) > 0 ? 0.0 : M_PI);
            }
            else
            {
                car->pos.SetHeadingRelative(SIGN(lane->GetId()) > 0 ? M_PI : 0.0);
            }
        }
    }

    if (car->model->txNode_ != 0)
    {
        double h, p, r;
        R0R12EulerAngles(car->pos.GetHRoad(), car->pos.GetPRoad(), car->pos.GetRRoad(), car->pos.GetHRelative(), 0.0, 0.0, h, p, r);

        car->model->SetPosition(car->pos.GetX(), car->pos.GetY(), car->pos.GetZ());
        car->model->SetRotation(h, p, r);
    }
}

int main(int argc, char **argv)
{
    static char str_buf[128];

    SE_Options &opt = SE_Env::Inst().GetOptions();
    opt.Reset();

    SE_Env::Inst().AddPath(DirNameOf(argv[0]));  // Add location of exe file to search paths

    std::vector<std::string> args;
    for (int i = 0; i < argc; i++)
        args.push_back(argv[i]);

    // use an ArgumentParser object to manage the program arguments.
    opt.AddOption("help", "Show this help message");
    opt.AddOption("odr", "OpenDRIVE filename (required)", "odr_filename");
    opt.AddOption("aa_mode", "Anti-alias mode=number of multisamples (subsamples, 0=off, 4=default)", "mode");
    opt.AddOption("capture_screen", "Continuous screen capture. Warning: Many .tga files will be created");
    opt.AddOption("custom_fixed_camera",
                  "Additional custom camera position <x,y,z>[,h,p] (multiple occurrences supported)",
                  "position and optional orientation");
    opt.AddOption("custom_fixed_top_camera", "Additional custom top camera <x,y,z,rot> (multiple occurrences supported)", "position and rotation");
    opt.AddOption("density", "density (cars / 100 m)", "density", std::to_string(density));
    opt.AddOption("enforce_generate_model", "Generate road 3D model even if --model is specified");
    opt.AddOption("disable_log", "Prevent logfile from being created");
    opt.AddOption("disable_off_screen", "Disable esmini off-screen rendering, revert to OSG viewer default handling");
    opt.AddOption("disable_stdout", "Prevent messages to stdout");
    opt.AddOption("duration", "Quit automatically after specified time (seconds, floating point)", "duration");
    opt.AddOption("fixed_timestep", "Run simulation decoupled from realtime, with specified timesteps", "timestep");
    opt.AddOption("generate_no_road_objects", "Do not generate any OpenDRIVE road objects (e.g. when part of referred 3D model)");
    opt.AddOption("generate_without_textures", "Do not apply textures on any generated road model (set colors instead as for missing textures)");
    opt.AddOption("ground_plane", "Add a large flat ground surface");
    opt.AddOption("headless", "Run without viewer window");
    opt.AddOption("log_append", "log all scenarios in the same txt file");
    opt.AddOption("logfile_path", "logfile path/filename, e.g. \"../esmini.log\"", "path", LOG_FILENAME, true);
    opt.AddOption("log_meta_data", "log file name, function name and line number");
    opt.AddOption("log_level", "log level debug, info, warn, error", "mode", "info");
    opt.AddOption("log_only_modules", "log from only these modules. Overrides log_skip_modules", "modulename(s)");
    opt.AddOption("log_skip_modules", "skip log from these modules, all remaining modules will be logged.", "modulename(s)");
    opt.AddOption("model", "3D Model filename", "model_filename");
    opt.AddOption("osg_screenshot_event_handler", "Revert to OSG default jpg images ('c'/'C' keys handler)");
    opt.AddOption("osi_lines", "Show OSI road lines (toggle during simulation by press 'u') ");
    opt.AddOption("osi_points", "Show OSI road points (toggle during simulation by press 'y') ");
    opt.AddOption("path", "Search path prefix for assets, e.g. car and sign model files", "path");
    opt.AddOption("road_features", "Show OpenDRIVE road features (toggle during simulation by press 'o') ");
    opt.AddOption("save_generated_model", "Save generated 3D model (n/a when a scenegraph is loaded)");
    opt.AddOption("seed", "Specify seed number for random generator", "number");
    opt.AddOption("speed_factor", "speed_factor <number>", "speed_factor", std::to_string(global_speed_factor));
    opt.AddOption("stop_at_end_of_road", "Instead of respawning elsewhere, stop when no connection exists");
    opt.AddOption("text_scale", "Scale screen overlay text", "factor", "1.0");
    opt.AddOption("traffic_rule", "Enforce left or right hand traffic, regardless OpenDRIVE rule attribute (default: right)", "rule (right/left)");
    opt.AddOption("use_signs_in_external_model", "When external scenegraph 3D model is loaded, skip creating signs from OpenDRIVE");
    opt.AddOption("version", "Show version and quit");

    if (opt.ParseArgs(argc, argv) != 0)
    {
        opt.PrintUsage();
        return -1;
    }

    if (opt.GetOptionSet("version"))
    {
        return 0;
    }

    if (opt.GetOptionSet("help"))
    {
        opt.PrintUsage();
        viewer::Viewer::PrintUsage();
        return 0;
    }

    std::string arg_str;

    if ((arg_str = opt.GetOptionArg("fixed_timestep")) != "")
    {
        fixed_timestep = atof(arg_str.c_str());
        LOG_INFO("Run simulation decoupled from realtime, with fixed timestep: {:.3f}", fixed_timestep);
    }

    if (opt.GetOptionSet("disable_log"))
    {
        // printf("Disable logfile\n");
    }
    else if (opt.IsOptionArgumentSet("logfile_path"))
    {
        arg_str = opt.GetOptionArg("logfile_path");
        if (arg_str.empty())
        {
            printf("Custom logfile path empty, disable logfile\n");
        }
        else
        {
            printf("Custom logfile path: %s\n", arg_str.c_str());
        }
    }

    TxtLogger::Inst().SetMetaDataEnabled(opt.IsOptionArgumentSet("log_meta_data"));

    if (opt.IsOptionArgumentSet("log_only_modules"))
    {
        arg_str             = opt.GetOptionArg("log_only_modules");
        const auto splitted = utils::SplitString(arg_str, ',');
        if (!splitted.empty())
        {
            std::unordered_set<std::string> logOnlyModules;
            logOnlyModules.insert(splitted.begin(), splitted.end());
            TxtLogger::Inst().SetLogOnlyModules(logOnlyModules);
        }
    }

    if (opt.IsOptionArgumentSet("log_skip_modules"))
    {
        arg_str             = opt.GetOptionArg("log_skip_modules");
        const auto splitted = utils::SplitString(arg_str, ',');
        if (!splitted.empty())
        {
            std::unordered_set<std::string> logSkipModules;
            logSkipModules.insert(splitted.begin(), splitted.end());
            TxtLogger::Inst().SetLogSkipModules(logSkipModules);
        }
    }

    if ((arg_str = opt.GetOptionArg("path")) != "")
    {
        SE_Env::Inst().AddPath(arg_str);
        LOG_INFO("Added path {}", arg_str);
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

    std::string odrFilename = opt.GetOptionArg("odr");
    if (odrFilename.empty())
    {
        printf("Missing required argument --odr\n");
        opt.PrintUsage();
        viewer::Viewer::PrintUsage();
        return -1;
    }

    std::string modelFilename = opt.GetOptionArg("model");

    if (opt.GetOptionArg("density") != "")
    {
        density = strtod(opt.GetOptionArg("density"));
    }
    LOG_INFO("density: {:.2f}", density);

    if (opt.GetOptionArg("speed_factor") != "")
    {
        global_speed_factor = strtod(opt.GetOptionArg("speed_factor"));
    }
    LOG_INFO("global speed factor: {:.2f}", global_speed_factor);

    if (opt.GetOptionArg("traffic_rule") != "")
    {
        if (opt.GetOptionArg("traffic_rule") == "left")
        {
            rule = roadmanager::Road::RoadRule::LEFT_HAND_TRAFFIC;
            LOG_INFO("Enforce left hand traffic");
        }
        else if (opt.GetOptionArg("traffic_rule") == "right")
        {
            rule = roadmanager::Road::RoadRule::RIGHT_HAND_TRAFFIC;
            LOG_INFO("Enforce right hand traffic");
        }
    }

    if (opt.GetOptionArg("duration") != "")
    {
        duration = strtod(opt.GetOptionArg("duration"));
    }

    if (opt.GetOptionSet("use_signs_in_external_model"))
    {
        LOG_INFO("Use sign models in external scene graph model, skip creating sign models");
    }

    try
    {
        if (!roadmanager::Position::LoadOpenDrive(odrFilename.c_str()))
        {
            printf("Failed to load ODR %s\n", odrFilename.c_str());
            return -1;
        }
        roadmanager::OpenDrive *odrManager = roadmanager::Position::GetOpenDrive();

        osg::ArgumentParser arguments(&argc, argv);
        viewer::Viewer     *viewer = new viewer::Viewer(odrManager, modelFilename.c_str(), NULL, argv[0], arguments, &opt);

        viewer->SetWindowTitleFromArgs(args);
        viewer->RegisterKeyEventCallback(FetchKeyEvent, nullptr);

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
            viewer->ClearNodeMaskBits(viewer::NodeMask::NODE_MASK_ODR_FEATURES);
        }

        if (opt.GetOptionSet("osi_lines"))
        {
            viewer->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_OSI_LINES);
        }

        if (opt.GetOptionSet("osi_points"))
        {
            viewer->SetNodeMaskBits(viewer::NodeMask::NODE_MASK_OSI_POINTS);
        }

        if (opt.GetOptionSet("stop_at_end_of_road"))
        {
            stop_at_end_of_road = true;
        }

        if (opt.GetOptionSet("custom_fixed_camera") == true)
        {
            int counter = 0;

            while ((arg_str = opt.GetOptionArg("custom_fixed_camera", counter)) != "")
            {
                const auto splitted = utils::SplitString(arg_str, ',');

                if (splitted.size() == 3)
                {
                    viewer->AddCustomCamera(strtod(splitted[0]), strtod(splitted[1]), strtod(splitted[2]), true);
                    LOG_INFO("Created custom fixed camera {} ({}, {}, {})", counter, splitted[0], splitted[1], splitted[2]);
                }
                else if (splitted.size() == 5)
                {
                    viewer->AddCustomCamera(strtod(splitted[0]),
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
                viewer->SetCameraMode(-1);  // activate last camera which is the one just added
                counter++;
            }
        }

        if (opt.GetOptionSet("custom_fixed_top_camera") == true)
        {
            int counter = 0;

            while ((arg_str = opt.GetOptionArg("custom_fixed_top_camera", counter)) != "")
            {
                const auto splitted = utils::SplitString(arg_str, ',');
                if (splitted.size() != 4)
                {
                    LOG_ERROR_AND_QUIT("Expected custom_fixed_top_camera <x,y,z,rot>. Got {} values instead of 4", splitted.size());
                }
                viewer->AddCustomFixedTopCamera(strtod(splitted[0]), strtod(splitted[1]), strtod(splitted[2]), strtod(splitted[3]));
                viewer->SetCameraMode(-1);  // activate last camera which is the one just added

                LOG_INFO("Created custom fixed top camera {} ({}, {}, {}, {})", counter, splitted[0], splitted[1], splitted[2], splitted[3]);
                counter++;
            }
        }

        if (opt.HasUnknownArgs())
        {
            opt.PrintUnknownArgs("Unrecognized arguments:");
            opt.PrintUsage();
        }

        LOG_INFO("osi_features: lines %s points %s",
                 viewer->GetNodeMaskBit(viewer::NodeMask::NODE_MASK_OSI_LINES) ? "on" : "off",
                 viewer->GetNodeMaskBit(viewer::NodeMask::NODE_MASK_OSI_POINTS) ? "on" : "off");

        if (SetupCars(odrManager, viewer) == -1)
        {
            return 4;
        }
        LOG_INFO("%d cars added", static_cast<int>(cars.size()));

        __int64 now            = 0;
        __int64 lastTimeStamp  = 0;
        __int64 firstTimeStamp = 0;

        static bool first_time  = true;
        double      system_time = 0.0;

        while (!viewer->osgViewer_->done())
        {
            now = SE_getSystemTime();
            if (first_time)
            {
                firstTimeStamp = now;
                lastTimeStamp  = now;
            }
            system_time = static_cast<double>(now - firstTimeStamp) / 1000.0;  // system time time in seconds

            if (run_state.step || !run_state.pause)
            {
                if (fixed_timestep > 0)
                {
                    deltaSimTime = fixed_timestep;
                }
                else
                {
                    // Get milliseconds since Jan 1 1970
                    deltaSimTime = static_cast<double>(now - lastTimeStamp) / 1000.0;  // step size in seconds
                    if (deltaSimTime > maxStepSize)                                    // limit step size
                    {
                        deltaSimTime = maxStepSize;
                    }
                    else if (deltaSimTime < minStepSize)  // avoid CPU rush, sleep for a while
                    {
                        SE_sleep(static_cast<unsigned int>(now - lastTimeStamp));
                        deltaSimTime = minStepSize;
                    }
                }

                if (!first_time)
                {
                    for (size_t i = 0; i < cars.size(); i++)
                    {
                        updateCar(odrManager, cars[i], deltaSimTime);
                    }
                }
            }

            // Always update info text, since camera might jump between different entities also during pause
            if (static_cast<int>(cars.size()) > 0 && viewer->currentCarInFocus_ >= 0 && viewer->currentCarInFocus_ < static_cast<int>(cars.size()))
            {
                Car *car = cars[static_cast<unsigned int>(viewer->currentCarInFocus_)];
                snprintf(str_buf,
                         sizeof(str_buf),
                         "entity[%d]: %.2fkm/h (%d, %d, %.2f, %.2f) / (%.2f, %.2f %.2f)",
                         viewer->currentCarInFocus_,
                         3.6 * car->pos.GetSpeedLimit() * car->speed_factor * global_speed_factor,
                         car->pos.GetTrackId(),
                         car->pos.GetLaneId(),
                         fabs(car->pos.GetOffset()) < SMALL_NUMBER ? 0 : car->pos.GetOffset(),
                         car->pos.GetS(),
                         car->pos.GetX(),
                         car->pos.GetY(),
                         car->pos.GetH());
                viewer->SetInfoText(str_buf);
            }

            run_state.step = false;

            // Step the simulation
            viewer->osgViewer_->frame();

            if (duration > -SMALL_NUMBER && system_time > duration - SMALL_NUMBER)
            {
                printf("Stop after %.2f seconds\n", system_time);
                break;
            }

            lastTimeStamp = now;
            first_time    = false;
        }
        delete viewer;
    }
    catch (std::logic_error &e)
    {
        printf("%s\n", e.what());
        return 2;
    }
    catch (std::runtime_error &e)
    {
        printf("%s\n", e.what());
        return 3;
    }

    for (size_t i = 0; i < cars.size(); i++)
    {
        delete (cars[i]);
    }
    return 0;
}
