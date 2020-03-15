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

	// use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);	

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName());
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options]\n");
	arguments.getApplicationUsage()->addCommandLineOption("-f <file.dat>", "Recording data file to replay");
	arguments.getApplicationUsage()->addCommandLineOption("--res_path <path>", "path to Resources directory, relative or absolute");
	arguments.getApplicationUsage()->addCommandLineOption("-s <factor>", "Time scale factor");

	if (arguments.argc() < 2)
	{
		arguments.getApplicationUsage()->write(std::cout, 1, 120, true);
		return -1;
	}


	std::string res_path;
	arguments.read("--res_path", res_path);

	std::string rec_filename;
	arguments.read("-f", rec_filename);

	std::string time_scale_str;
	arguments.read("-s", time_scale_str);
	double time_scale = stod(time_scale_str);



	// Create player
	try
	{
		player = new Replay(rec_filename);
	}
	catch (const std::exception& e)
	{
		LOG(e.what());
		return -1;
	}


	try
	{
		std::string odr_path = res_path;
		roadmanager::Position::LoadOpenDrive(odr_path.append("/xodr/").append(player->header_.odr_filename).c_str());
		odrManager = roadmanager::Position::GetOpenDrive();

		std::string model_path = res_path;
		viewer::Viewer *viewer = new viewer::Viewer(
			odrManager, 
			model_path.append("/models/").append(player->header_.model_filename).c_str(),
			NULL,
			arguments);

		__int64 now, lastTimeStamp = 0;

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
