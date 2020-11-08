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

#include "viewer.hpp"
#include "ScenarioGateway.hpp"
#include "RoadManager.hpp"
#include "RubberbandManipulator.hpp"
#include "CommonMini.hpp"
#include "Replay.hpp"

using namespace scenarioengine;

#define TIME_SCALE_FACTOR 1.1

static const double stepSize = 0.01;
static const double maxStepSize = 0.1;
static const double minStepSize = 0.001;
static bool pause = false;  // continuous play
static double time_scale = 1.0;

double deltaSimTime;  // external - used by Viewer::RubberBandCamera

typedef struct
{
	int id;
	viewer::EntityModel *entityModel;
	roadmanager::Position pos;
} ScenarioEntity;

static std::vector<ScenarioEntity> scenarioEntity;

// Car models used for populating the road network according to scenario object model ID
// path should be relative the OpenDRIVE file
static const char* entityModelsFiles_[] =
{
	"car_white.osgb",
	"car_blue.osgb",
	"car_red.osgb",
	"car_yellow.osgb",
	"truck_yellow.osgb",
	"van_red.osgb",
	"bus_blue.osgb",
};


ScenarioEntity *getScenarioEntityById(int id)
{
	for (size_t i=0; i<scenarioEntity.size(); i++)
	{
		if (scenarioEntity[i].id == id)
		{
			return &scenarioEntity[i];
		}
	}

	return 0;
}

void ReportKeyEvent(viewer::KeyEvent* keyEvent, void* data)
{
	Replay* player = (Replay*)data;

	if (keyEvent->down_)
	{
		if (keyEvent->key_ == KeyType::KEY_Right)
		{
			player->GoToNextFrame();
			pause = true;  // step by step
		}
		else if (keyEvent->key_ == KeyType::KEY_Left)
		{
			player->GoToPreviousFrame();
			pause = true;  // step by step
		}
		else if (keyEvent->key_ == KeyType::KEY_Space)
		{
			pause = !pause;
		}
		else if (keyEvent->key_ == KeyType::KEY_Up)
		{
			time_scale = MIN(100, time_scale * TIME_SCALE_FACTOR);
		}
		else if (keyEvent->key_ == KeyType::KEY_Down)
		{
			time_scale = MAX(0.01, time_scale / TIME_SCALE_FACTOR);
		}
	}
}

int main(int argc, char** argv)
{
	roadmanager::OpenDrive *odrManager;
	Replay *player;
	double simTime = 0;
	double view_mode = viewer::NodeMask::NODE_MASK_ENTITY_MODEL;
	bool no_ghost = false;
	static char info_str_buf[128];

	// use common options parser to manage the program arguments
	SE_Options opt;
	opt.AddOption("file", "Simulation recording data file", "filename");
	opt.AddOption("res_path", "Path to resources root folder - relative or absolut", "path");
	opt.AddOption("time_scale", "Playback speed scale factor (1.0 == normal)", "factor");
	opt.AddOption("start_time", "Start playing at timestamp", "ms");
	opt.AddOption("stop_time", "Stop playing at timestamp (set equal to time_start for single frame)", "ms");
	opt.AddOption("repeat", "loop scenario");
	opt.AddOption("view_mode", "Entity visualization: \"model\"(default)/\"boundingbox\"/\"both\"", "view_mode");
	opt.AddOption("no_ghost", "Remove ghost entities");

	if (argc < 2)
	{
		opt.PrintUsage();
		return -1;
	}

	opt.ParseArgs(&argc, argv);

	if (opt.GetOptionArg("file").empty())
	{
		printf("Missing file argument\n");
		opt.PrintUsage();
		return -1;
	}

	// Create player
	try
	{
		player = new Replay(opt.GetOptionArg("file"));
	}
	catch (const std::exception& e)
	{
		LOG(e.what());
		return -1;
	}

	try
	{
		std::string odr_path = opt.GetOptionArg("res_path");
		roadmanager::Position::LoadOpenDrive(odr_path.append("/xodr/").append(player->header_.odr_filename).c_str());
		odrManager = roadmanager::Position::GetOpenDrive();

		std::string model_path = opt.GetOptionArg("res_path");
		
		osg::ArgumentParser arguments(&argc, argv);

		viewer::Viewer *viewer = new viewer::Viewer(
			odrManager, 
			model_path.append("/models/").append(player->header_.model_filename).c_str(),
			NULL,
			argv[0],
			arguments, &opt);

		viewer->RegisterKeyEventCallback(ReportKeyEvent, player);

		if (argc > 1)
		{
			opt.PrintArgs(argc, argv, "Unrecognized arguments:");
			opt.PrintUsage();
			return -1;
		}
		viewer->SetWindowTitle("esmini - " + FileNameWithoutExtOf(argv[0]) + " " + (FileNameOf(opt.GetOptionArg("file"))));

		__int64 now, lastTimeStamp = 0;
		
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
		viewer->SetNodeMaskBits(
			viewer::NodeMask::NODE_MASK_ENTITY_MODEL | 
			viewer::NodeMask::NODE_MASK_ENTITY_BB, 
			view_mode);

		if (opt.GetOptionSet("no_ghost"))
		{
			no_ghost = true;
		}

		std::string start_time_str = opt.GetOptionArg("start_time");
		if (!start_time_str.empty())
		{
			double startTime = 1E-3 * strtod(start_time_str);
			if (startTime < player->data_[0].timeStamp)
			{
				printf("Specified start time (%.2f) < first timestamp (%.2f), adapting.\n", startTime, player->data_[0].timeStamp);
				startTime = player->data_[0].timeStamp;
			}
			else if (startTime > player->data_[player->data_.size() - 1].timeStamp)
			{
				printf("Specified start time (%.2f) > first timestamp (%.2f), adapting.\n", startTime, player->data_[0].timeStamp);
				startTime = player->data_[player->data_.size() - 1].timeStamp;
			}
			player->SetStartTime(startTime);
		}

		std::string stop_time_str = opt.GetOptionArg("stop_time");
		if (!stop_time_str.empty())
		{
			double stopTime = 1E-3 * strtod(stop_time_str);
			if (stopTime > player->data_[player->data_.size()-1].timeStamp)
			{
				printf("Specified stop time (%.2f) > last timestamp (%.2f), adapting.\n", stopTime, player->data_[0].timeStamp);
				stopTime = player->data_[player->data_.size() - 1].timeStamp;
			}
			else if (stopTime < player->data_[0].timeStamp)
			{
				printf("Specified stop time (%.2f) < first timestamp (%.2f), adapting.\n", simTime, player->data_[0].timeStamp);
				stopTime = player->data_[0].timeStamp;
			}
			player->SetStopTime(stopTime);
		}

		simTime = player->GetStartTime();

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
				SE_sleep(minStepSize - deltaSimTime);
				deltaSimTime = minStepSize;
			}
			deltaSimTime *= time_scale;

			if (!pause)
			{
				player->GoToTime(simTime + deltaSimTime);
			}
			simTime = player->GetTime();  // potentially wrapped for repeat

			// Fetch states of scenario objects
			ObjectStateStruct* state;
			
			for (int index = 0; (state = player->GetState(index)) != 0; index++)
			{
				if (no_ghost && state->ctrl_type == 100)  // control type 100 indicates ghost
				{
					continue;
				}

				ScenarioEntity *sc = getScenarioEntityById(state->id);

				// If not available, create it
				if (sc == 0)
				{
					ScenarioEntity new_sc;

					LOG("Creating car %d - got state from gateway", state->id);

					new_sc.id = state->id;
					if ((new_sc.entityModel = viewer->AddEntityModel(entityModelsFiles_[state->model_id], osg::Vec3(0.5, 0.5, 0.5),
						viewer::EntityModel::EntityType::ENTITY_TYPE_OTHER, false, state->name, &state->boundingbox)) == 0)
					{
						return -1;
					}

					// Add it to the list of scenario cars
					scenarioEntity.push_back(new_sc);

					sc = &scenarioEntity.back();
				}

				sc->pos = state->pos;

				if (index == viewer->currentCarInFocus_)
				{
					// Update overlay info text
					snprintf(info_str_buf, sizeof(info_str_buf), "%.2fs entity[%d]: %s %.2fkm/h timeScale: %.2f", 
						simTime, state->id, state->name, 3.6 * state->speed, time_scale);
					viewer->SetInfoText(info_str_buf);
				}
			}

			// Visualize scenario cars
			for (size_t i=0; i<scenarioEntity.size(); i++)
			{
				ScenarioEntity *c = &scenarioEntity[i];
				c->entityModel->SetPosition(c->pos.GetX(), c->pos.GetY(), c->pos.GetZ());
				c->entityModel->SetRotation(c->pos.GetH(), c->pos.GetR(), c->pos.GetP());
			}

			// Update graphics
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

	delete player;

	return 0;
}
