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
  * This application uses the Replay class to read and binary recordings and print content in ascii format to stdout
  */

#include "Replay.hpp"
#include "CommonMini.hpp"

using namespace scenarioengine;


int main(int argc, char** argv)
{
	Replay *player;

	if (argc < 2)
	{
		printf("Usage: %s <filename>\n", argv[0]);
		return -1;
	}

	// Create replayer object for parsing the binary data file
	try
	{
		player = new Replay(argv[1]);
	}
	catch (const std::exception& e)
	{
		printf(e.what());
		return -1;
	}

	// First output header and CSV labels
	fprintf(stdout, "OpenDRIVE: %s, 3DModel: %s\n", player->header_.odr_filename, player->header_.model_filename);
	fprintf(stdout, "timestamp, id, name, x, y, z, h, p, r, speed, wheel_angle, wheel_rot\n");

	// Then output all entries with comma separated values
	for (size_t i = 0; i < player->data_.size(); i++)
	{
		ObjectStateStruct *state = &player->data_[i];

		fprintf(stdout, "%.3f, %d, %s, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
			state->timeStamp,
			state->id,
			state->name,
			state->pos.GetX(),
			state->pos.GetY(),
			state->pos.GetZ(),
			state->pos.GetH(),
			state->pos.GetP(),
			state->pos.GetR(),
			state->speed,
			state->wheel_angle,
			state->wheel_rot);
	}

	delete player;
}