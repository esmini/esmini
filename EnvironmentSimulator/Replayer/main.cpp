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

static const double stepSize = 0.01;
static const double maxStepSize = 0.1;
static const double minStepSize = 0.01;

double deltaSimTime;  // external - used by Viewer::RubberBandCamera

typedef struct
{
	int id;
	viewer::CarModel *carModel;
	roadmanager::Position pos;
} ScenarioCar;

static std::vector<ScenarioCar> scenarioCar;

// Car models used for populating the road network according to scenario object model ID
// path should be relative the OpenDRIVE file
static const char* carModelsFiles_[] =
{
	"../models/car_white.osgb",
	"../models/car_blue.osgb",
	"../models/car_red.osgb",
	"../models/car_yellow.osgb",
	"../models/truck_yellow.osgb",
	"../models/van_red.osgb",
	"../models/bus_blue.osgb",
};


ScenarioCar *getScenarioCarById(int id)
{
	for (size_t i=0; i<scenarioCar.size(); i++)
	{
		if (scenarioCar[i].id == id)
		{
			return &scenarioCar[i];
		}
	}

	return 0;
}

int main(int argc, char** argv)
{
	roadmanager::OpenDrive *odrManager;
	Replay *player;
	double simTime = 0;
	double time_scale = 1.0;

	// use common options parser to manage the program arguments
	SE_Options opt;
	opt.AddOption("file", "Simulation recording data file", "filename");
	opt.AddOption("res_path", "Path to resources root folder - relative or absolut", "path");
	opt.AddOption("time_scale", "Playback speed scale factor (1.0 == normal)", "factor");

	if (argc < 2)
	{
		opt.PrintUsage();
		return -1;
	}

	opt.ParseArgs(&argc, argv);

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
			arguments);

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
		}

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

			// Time operations
			simTime = simTime + deltaSimTime;

			player->Step(deltaSimTime * time_scale);

			// Fetch states of scenario objects
			int index = 0;
			ObjectStateStruct *state = player->GetState(index);
			while (state != 0)
			{
				ScenarioCar *sc = getScenarioCarById(state->id);

				// If not available, create it
				if (sc == 0)
				{
					ScenarioCar new_sc;

					LOG("Creating car %d - got state from gateway", state->id);

					new_sc.id = state->id;
					if ((new_sc.carModel = viewer->AddCar(carModelsFiles_[state->model_id], false, osg::Vec3(0.5, 0.5, 0.5), false)) == 0)
					{
						return -1;
					}

					// Add it to the list of scenario cars
					scenarioCar.push_back(new_sc);

					sc = &scenarioCar.back();
				}

				sc->pos = state->pos;

				index++;
				state = player->GetState(index);
			}

			// Visualize scenario cars
			for (size_t i=0; i<scenarioCar.size(); i++)
			{
				ScenarioCar *c = &scenarioCar[i];
				c->carModel->SetPosition(c->pos.GetX(), c->pos.GetY(), c->pos.GetZ());
				c->carModel->SetRotation(c->pos.GetH(), c->pos.GetR(), c->pos.GetP());
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
