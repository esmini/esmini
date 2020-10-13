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


#include "playerbase.hpp"
#include "CommonMini.cpp"

#define MIN_TIME_STEP 0.01
#define MAX_TIME_STEP 0.1

void UpdateCSV_Log(ScenarioPlayer *player)
{
	//Flag for signalling end of data line, all vehicles reported
	bool isendline = false;

	//For each vehicle (entitity) stored in the ScenarioPlayer
	for (size_t i = 0; i < player->scenarioEngine->entities.object_.size(); i++)
	{
		//Create a pointer to the object at position i in the entities vector
		Object *obj = player->scenarioEngine->entities.object_[i];

		//Create a Position object for extracting this vehicles XYZ coordinates
		roadmanager::Position pos = obj->pos_;

		//Extract the String name of the object and store in a compatable const char array
		const char *name_ = &(*obj->name_.c_str());

		if ((i + 1) == player->scenarioEngine->entities.object_.size())
		{
			isendline = true;
		}

		//Log the extracted data of ego vehicle and additonal scenario vehicles
		player->CSV_Log->LogVehicleData(isendline, player->scenarioEngine->getSimulationTime(), name_,
			obj->id_, obj->speed_, obj->wheel_angle_, obj->wheel_rot_,
			pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetS(), pos.GetT(), pos.GetH(),
			pos.GetHRelative(), pos.GetHRelativeDrivingDirection(),
			pos.GetP(), pos.GetCurvature());
	}
}

int main(int argc, char *argv[])
{
	ScenarioPlayer *player;
	__int64 time_stamp = 0;

	try
	{
		player = new ScenarioPlayer(argc, argv);
	}
	catch (const std::exception& e)
	{
		LOG(e.what());
		return -1;
	}

	while (!player->IsQuitRequested())
	{
		double dt;
		if (player->GetFixedTimestep() > 0.0)
		{
			dt = player->GetFixedTimestep();
		}
		else
		{
			dt = SE_getSimTimeStep(time_stamp, player->minStepSize, player->maxStepSize);
		}

		player->Frame(dt);

		if (player->CSV_Log)
		{
			UpdateCSV_Log(player);
		}
	}

	delete player;

	return 0;
}
