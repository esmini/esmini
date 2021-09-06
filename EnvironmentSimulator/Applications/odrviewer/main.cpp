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


#define DEFAULT_SPEED   70  // km/h
#define DEFAULT_DENSITY 1   // Cars per 100 m
#define ROAD_MIN_LENGTH 30
#define SIGN(X) ((X<0)?-1:1)


static bool run_only_once = false;
static const double stepSize = 0.01;
static const double maxStepSize = 0.1;
static const double minStepSize = 0.01;
static const bool freerun = true;
static std::mt19937 mt_rand;
static double density = DEFAULT_DENSITY;
static double speed = DEFAULT_SPEED;
static double global_speed_factor = 1.0;
static int first_car_in_focus = -1;
roadmanager::Road::RoadRule rule = roadmanager::Road::RoadRule::ROAD_RULE_UNDEFINED;

double deltaSimTime;  // external - used by Viewer::RubberBandCamera

typedef struct
{
	int road_id_init;
	int lane_id_init;
	double heading_init;
	double s_init;
	roadmanager::Position *pos;
	double speed_factor;  // speed vary bewtween lanes, m/s
	viewer::EntityModel *model;
	int id;
} Car;

typedef struct
{
	int roadId;
	int side;
	int nLanes;
	double s;
} OpenEnd;

std::vector<Car*> cars;
std::vector<OpenEnd> openEnds;

// Car models used for populating the road network
// path should be relative the OpenDRIVE file
static const char* carModelsFiles_[] =
{
	"car_white.osgb",
	"car_blue.osgb",
	"car_red.osgb",
	"car_yellow.osgb",
	"truck_yellow.osgb",
	"van_red.osgb",
	"bus_blue.osgb",
};

std::vector<osg::ref_ptr<osg::LOD>> carModels_;


void log_callback(const char *str)
{
	printf("%s\n", str);
}

void FetchKeyEvent(viewer::KeyEvent* keyEvent, void*)
{
	if (keyEvent->down_)
	{
		if (keyEvent->key_ == 'H')
		{
			puts(helpText);
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
		roadmanager::Road *road = odrManager->GetRoadByIdx(r);
		double average_distance = 100.0 / density;

		roadmanager::Road::RoadRule rrule = road->GetRule();
		if (rule != roadmanager::Road::RoadRule::ROAD_RULE_UNDEFINED)
		{
			// Enforce specified rule
			rrule = rule;
		}

		// Check for open end
		OpenEnd openEnd;
		roadmanager::RoadLink *tmpLink = road->GetLink(roadmanager::LinkType::PREDECESSOR);
		if (tmpLink == nullptr)
		{
			openEnd.roadId = road->GetId();
			openEnd.s = 0;
			openEnd.side = rrule == roadmanager::Road::RoadRule::LEFT_HAND_TRAFFIC ? 1 : -1;
			openEnd.nLanes = road->GetNumberOfDrivingLanesSide(openEnd.s, -1);
			if (openEnd.nLanes > 0)
			{
				openEnds.push_back(openEnd);
			}
		}
		tmpLink = road->GetLink(roadmanager::LinkType::SUCCESSOR);
		if (tmpLink == nullptr)
		{
			openEnd.roadId = road->GetId();
			openEnd.s = road->GetLength();
			openEnd.side = rrule == roadmanager::Road::RoadRule::LEFT_HAND_TRAFFIC ? -1 : 1;
			openEnd.nLanes = road->GetNumberOfDrivingLanesSide(openEnd.s, 1);
			if (openEnd.nLanes > 0)
			{
				openEnds.push_back(openEnd);
			}
		}

		if (road->GetLength() > ROAD_MIN_LENGTH)
		{
			// Populate road lanes with vehicles at some random distances
			for (double s = 10; s < road->GetLength() - average_distance; s += average_distance + 0.2 * average_distance * mt_rand() / mt_rand.max())
			{
				// Pick lane by random
				int lane_idx = ((double)road->GetNumberOfDrivingLanes(s) * mt_rand()) / (mt_rand.max)();
				roadmanager::Lane *lane = road->GetDrivingLaneByIdx(s, lane_idx);
				if (lane == 0)
				{
					LOG("Failed locate driving lane %d at s %d", lane_idx, s);
					continue;
				}


				if ((SIGN(lane->GetId()) < 0) && (road->GetLength() - s < 50) && (road->GetLink(roadmanager::LinkType::SUCCESSOR) == 0) ||
					(SIGN(lane->GetId()) > 0) && (s < 50) && (road->GetLink(roadmanager::LinkType::PREDECESSOR) == 0))
				{
					// Skip vehicles too close to road end - and where connecting road is missing
					continue;
				}

				// randomly choose model
				int carModelID = (double(sizeof(carModelsFiles_) / sizeof(carModelsFiles_[0])) * mt_rand()) / (mt_rand.max)();
				//LOG("Adding car of model %d to road nr %d (road id %d s %.2f lane id %d), ", carModelID, r, road->GetId(), s, lane->GetId());

				Car *car_ = new Car;
				// Higher speeds in lanes closer to reference lane
				car_->speed_factor = 0.5 + 0.5 / abs(lane->GetId());  // Speed vary between 0.5 to 1.0 times default speed
				car_->road_id_init = odrManager->GetRoadByIdx(r)->GetId();
				car_->lane_id_init = lane->GetId();
				car_->s_init = s;
				car_->pos = new roadmanager::Position(odrManager->GetRoadByIdx(r)->GetId(), lane->GetId(), s, 0);
				if (rrule == roadmanager::Road::RoadRule::LEFT_HAND_TRAFFIC)
				{
					car_->pos->SetHeadingRelative(lane->GetId() < 0 ? M_PI : 0);
				}
				else
				{
					car_->pos->SetHeadingRelative(lane->GetId() < 0 ? 0 : M_PI);
				}
				car_->heading_init = car_->pos->GetHRelative();

				if ((car_->model = viewer->AddEntityModel(carModelsFiles_[carModelID], osg::Vec4(0.5, 0.5, 0.5, 1.0),
					viewer::EntityModel::EntityType::ENTITY_TYPE_VEHICLE, false, "", 0)) == 0)
				{
					return -1;
				}
				car_->id = cars.size();
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

int SetupCarsSpecial(roadmanager::OpenDrive* odrManager, viewer::Viewer* viewer)
{
	// Setup one single vehicle in a dedicated pos
	Car* car_ = new Car;

	car_->speed_factor = 1.0;
	car_->road_id_init = 1;
	car_->lane_id_init = -1;
	car_->s_init = 40;
	car_->pos = new roadmanager::Position(car_->road_id_init, car_->lane_id_init, car_->s_init, 0);
	car_->pos->SetHeadingRelative(car_->lane_id_init < 0 ? 0 : M_PI);
	car_->heading_init = car_->pos->GetHRelative();

	if ((car_->model = viewer->AddEntityModel(carModelsFiles_[0], osg::Vec4(0.5, 0.5, 0.5, 1.0),
		viewer::EntityModel::EntityType::ENTITY_TYPE_VEHICLE, false, "", 0)) == 0)
	{
		return -1;
	}
	car_->id = cars.size();
	cars.push_back(car_);

	first_car_in_focus = 0;

	return 0;
}

void updateCar(roadmanager::OpenDrive *odrManager, Car *car, double dt)
{
	double new_speed = car->pos->GetSpeedLimit() * car->speed_factor * global_speed_factor;
	double ds = new_speed * dt; // right lane is < 0 in road dir;

	roadmanager::Road::RoadRule rrule = odrManager->GetRoadById(car->pos->GetTrackId())->GetRule();
	if (rule != roadmanager::Road::RoadRule::ROAD_RULE_UNDEFINED)
	{
		// Enforce specified rule
		rrule = rule;
	}

	if (rrule == roadmanager::Road::RoadRule::LEFT_HAND_TRAFFIC)
	{
		ds *= -1;
	}

	if (car->pos->MoveAlongS(ds) != roadmanager::Position::ErrorCode::ERROR_NO_ERROR)
	{
		if (openEnds.size() == 0)
		{
			// If no open ends, respawn based on initial position
			// start from beginning of lane section - not initial s-position
			roadmanager::LaneSection* ls = odrManager->GetRoadById(car->road_id_init)->GetLaneSectionByS(car->s_init);
			double start_s = ls->GetS() + 5;
			if (car->lane_id_init > 0)
			{
				start_s = ls->GetS() + ls->GetLength() - 5;
			}
			car->pos->SetLanePos(car->road_id_init, car->lane_id_init, start_s, 0);
			car->pos->SetHeadingRelative(car->heading_init);
		}
		else
		{
			// Choose random open end
			int oeIndex = (int)(((double)openEnds.size()) * mt_rand() / mt_rand.max());
			OpenEnd* oe = &openEnds[oeIndex];
			// Choose random lane
			int laneIndex = (int)(((double)oe->nLanes) * mt_rand() / mt_rand.max());
			roadmanager::Road* road = odrManager->GetRoadById(oe->roadId);
			roadmanager::Lane *lane = road->GetDrivingLaneSideByIdx(oe->s, oe->side, laneIndex);

			car->pos->SetLanePos(road->GetId(), lane->GetId(), oe->s, 0);

			// Ensure car is oriented along lane driving direction
			rrule = road->GetRule();
			if (rule != roadmanager::Road::RoadRule::ROAD_RULE_UNDEFINED)
			{
				// Enforce specified rule
				rrule = rule;
			}
			if (rrule == roadmanager::Road::RoadRule::LEFT_HAND_TRAFFIC)
			{
				car->pos->SetHeadingRelative(SIGN(lane->GetId()) > 0 ? 0.0 : M_PI);
			}
			else
			{
				car->pos->SetHeadingRelative(SIGN(lane->GetId()) > 0 ? M_PI : 0.0);
			}
		}
	}

	if (car->model->txNode_ != 0)
	{
		double h, p, r;
		ZYZ2EulerAngles(car->pos->GetHRoad(), car->pos->GetPRoad(), car->pos->GetHRelative(), h, p, r);

		car->model->SetPosition(car->pos->GetX(), car->pos->GetY(), car->pos->GetZ());
		car->model->SetRotation(h, p, r);
	}
}

int main(int argc, char** argv)
{
	static char str_buf[128];
	SE_Options opt;

	// Use logger callback
	Logger::Inst().SetCallback(log_callback);

	mt_rand.seed((unsigned int)time(0));

	std::vector<std::string> args;
	for (int i = 0; i < argc; i++) args.push_back(argv[i]);

	// use an ArgumentParser object to manage the program arguments.
	opt.AddOption("odr", "OpenDRIVE filename (required)", "odr_filename");
	opt.AddOption("model", "3D Model filename", "model_filename");
	opt.AddOption("density", "density (cars / 100 m)", "density");
	opt.AddOption("generate_no_road_objects", "Do not generate any OpenDRIVE road objects (e.g. when part of referred 3D model)");
	opt.AddOption("speed_factor", "speed_factor <number>", "speed_factor");
	opt.AddOption("osi_lines", "Show OSI road lines (toggle during simulation by press 'u') ");
	opt.AddOption("osi_points", "Show OSI road points (toggle during simulation by press 'y') ");
	opt.AddOption("road_features", "Show OpenDRIVE road features (toggle during simulation by press 'o') ");
	opt.AddOption("path", "Search path prefix for assets, e.g. car and sign model files", "path");
	opt.AddOption("logfile_path", "logfile path/filename, e.g. \"../esmini.log\" (default: log.txt)", "path");
	opt.AddOption("disable_log", "Prevent logfile from being created");
	opt.AddOption("disable_stdout", "Prevent messages to stdout");
	opt.AddOption("help", "Show this help message");
	opt.AddOption("save_generated_model", "Save generated 3D model (n/a when a scenegraph is loaded)");
	opt.AddOption("traffic_rule", "Enforce left or right hand traffic, regardless OpenDRIVE rule attribute (default: right)", "rule (right/left)");
	opt.AddOption("version", "Show version and quit");

	opt.ParseArgs(&argc, argv);

	if (opt.GetOptionSet("version"))
	{
		Logger::Inst().LogVersion();
		return 0;
	}

	if (opt.GetOptionSet("help"))
	{
		opt.PrintUsage();
		viewer::Viewer::PrintUsage();
		return 0;
	}

	std::string arg_str;

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

	if ((arg_str = opt.GetOptionArg("path")) != "")
	{
		SE_Env::Inst().AddPath(arg_str);
		LOG("Added path %s", arg_str.c_str());
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
	LOG("density: %.2f", density);

	if (opt.GetOptionArg("speed_factor") != "")
	{
		global_speed_factor = strtod(opt.GetOptionArg("speed_factor"));
	}
	LOG("global speed factor: %.2f", global_speed_factor);

	if (opt.GetOptionArg("traffic_rule") != "")
	{
		if (opt.GetOptionArg("traffic_rule") == "left")
		{
			rule = roadmanager::Road::RoadRule::LEFT_HAND_TRAFFIC;
			LOG("Enforce left hand traffic");
		}
		else if (opt.GetOptionArg("traffic_rule") == "right")
		{
			rule = roadmanager::Road::RoadRule::RIGHT_HAND_TRAFFIC;
			LOG("Enforce right hand traffic");
		}
	}

	roadmanager::Position *lane_pos = new roadmanager::Position();
	roadmanager::Position *track_pos = new roadmanager::Position();

	try
	{
		if (!roadmanager::Position::LoadOpenDrive(odrFilename.c_str()))
		{
			printf("Failed to load ODR %s\n", odrFilename.c_str());
			return -1;
		}
		roadmanager::OpenDrive *odrManager = roadmanager::Position::GetOpenDrive();

		osg::ArgumentParser arguments(&argc, argv);
		viewer::Viewer *viewer = new viewer::Viewer(
			odrManager,
			modelFilename.c_str(),
			NULL,
			argv[0],
			arguments,
			&opt);

		viewer->SetWindowTitleFromArgs(args);
		viewer->RegisterKeyEventCallback(FetchKeyEvent, nullptr);

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

		if (argc > 1)
		{
			opt.PrintArgs(argc, argv, "Unrecognized arguments:");
			opt.PrintUsage();
		}

		LOG("osi_features: lines %s points %s",
			viewer->GetNodeMaskBit(viewer::NodeMask::NODE_MASK_OSI_LINES) ? "on" : "off",
			viewer->GetNodeMaskBit(viewer::NodeMask::NODE_MASK_OSI_POINTS) ? "on" : "off");

		if (SetupCars(odrManager, viewer) == -1)
		{
			return 4;
		}
		LOG("%d cars added", (int)cars.size());
		viewer->SetVehicleInFocus(first_car_in_focus);

		__int64 now, lastTimeStamp = 0;

		static bool first_time = true;

		while (!viewer->osgViewer_->done())
		{
			// Get milliseconds since Jan 1 1970
			now = SE_getSystemTime();
			deltaSimTime = (now - lastTimeStamp) / 1000.0;  // step size in seconds
			lastTimeStamp = now;
			if (deltaSimTime > maxStepSize) // limit step size
			{
				deltaSimTime = maxStepSize;
			}
			else if (deltaSimTime < minStepSize)  // avoid CPU rush, sleep for a while
			{
				SE_sleep(now - lastTimeStamp);
				deltaSimTime = minStepSize;
			}

			if (!(run_only_once && !first_time))
			{
				for (size_t i = 0; i < cars.size(); i++)
				{
					updateCar(odrManager, cars[i], deltaSimTime);
				}
				first_time = false;
			}

			// Set info text
			if (cars.size() > 0 && viewer->currentCarInFocus_ >= 0 && viewer->currentCarInFocus_ < cars.size())
			{
				Car* car = cars[viewer->currentCarInFocus_];
				snprintf(str_buf, sizeof(str_buf), "entity[%d]: %.2fkm/h (%d, %d, %.2f, %.2f) / (%.2f, %.2f %.2f)", viewer->currentCarInFocus_,
					3.6 * car->pos->GetSpeedLimit() * car->speed_factor * global_speed_factor, car->pos->GetTrackId(), car->pos->GetLaneId(),
					fabs(car->pos->GetOffset()) < SMALL_NUMBER ? 0 : car->pos->GetOffset(), car->pos->GetS(), car->pos->GetX(), car->pos->GetY(), car->pos->GetH());
				viewer->SetInfoText(str_buf);
			}

			// Step the simulation
			viewer->osgViewer_->frame();
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
		delete(cars[i]);
	}
	delete track_pos;
	delete lane_pos;

	return 0;
}
