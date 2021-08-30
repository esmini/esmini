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
  * This application produces a raw data file to be used in Python for plotting the road network as simple graphs.
  *
  * This module reads an OpenDRIVE file and subsamples all roads within the road network.
  * The resulting points are saved into a data file.
  * Use separate Python program xodr.py to plot the result.
  */

#include <cmath>
#include <iostream>
#include <fstream>
#include "RoadManager.hpp"
#include "CommonMini.hpp"

using namespace roadmanager;

int main(int argc, char *argv[])
{
	std::string output_file_name = "track.csv";
	std::ofstream file;
	std::string sampling_step = "1.0";
	double step_length_target;
	static char strbuf[1024];

	if (argc < 2)
	{
		printf("Usage: ordplot openDriveFile.xodr [Output file, default=output.csv] [Sampling_step, default=1.0]\n");
		return -1;
	}
	else
	{
		if (argc > 2)
		{
			output_file_name =  argv[2];
		}

		if (argc > 3)
		{
			sampling_step = argv[3];
		}
	}

	step_length_target = std::stod(sampling_step);

	try
	{
		if (Position::LoadOpenDrive(argv[1]) == false)
		{
			printf("Failed to open OpenDRIVE file %s\n", argv[1]);
			return -1;
		}
		file.open(output_file_name);
	}
	catch (std::exception& e)
	{
		printf("exception: %s\n", e.what());
		return -1;
	}

	Position* pos = new Position();

	OpenDrive *od = Position::GetOpenDrive();

	for (int r = 0; r < od->GetNumOfRoads(); r++)
	{
		Road *road = od->GetRoadByIdx(r);

		for (int i = 0; i < road->GetNumberOfLaneSections(); i++)
		{
			LaneSection *lane_section = road->GetLaneSectionByIdx(i);
			double s_start = lane_section->GetS();
			double s_end = s_start + lane_section->GetLength();
			int steps = MAX(1, (int)((s_end - s_start) / step_length_target));
			double step_length = steps > 0 ? (s_end - s_start) / steps : s_end - s_start;

			for (int j = 0; j < lane_section->GetNumberOfLanes(); j++)
			{
				Lane *lane = lane_section->GetLaneByIdx(j);

				file << "lane, " << road->GetId() << ", " << i << ", " << lane->GetId() << (lane->IsDriving() ? ", driving" : ", no-driving") << std::endl;

				for (int k = 0; k < steps + 1; k++)
				{
					double s = MIN(s_end, s_start + k * step_length);

					// Set lane offset to half the lane width in order to mark the outer edge of the lane (laneOffset = 0 means middle of lane)
					pos->SetLanePos(road->GetId(), lane->GetId(), s, SIGN(lane->GetId())*lane_section->GetWidth(s, lane->GetId())*0.5, i);

					// Write the point to file
					snprintf(strbuf, sizeof(strbuf), "%f, %f, %f, %f\n", pos->GetX(), pos->GetY(), pos->GetZ(), pos->GetH());
					file << strbuf;
				}
			}
		}
	}
	file.close();

	delete pos;

	printf("Created %s using stepsize %.2f\n", output_file_name.c_str(), step_length_target);
	printf("To plot it, run EnvironmentSimulator/Applications/odrplot/xodr.py %s\n", output_file_name.c_str());

	return 0;
}
