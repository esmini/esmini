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

void log_callback(const char* str)
{
	printf("%s\n", str);
}

int main(int argc, char *argv[])
{
	// Use logger callback
	if (!(Logger::Inst().IsCallbackSet()))
	{
		Logger::Inst().SetCallback(log_callback);
	}

	std::string output_file_name = "track.csv";
	std::ofstream file;
	std::string sampling_step = "1.0";
	double step_length_target;
	
	if (argc < 2)
	{
		LOG("Usage: ordplot openDriveFile.xodr [Output file, default=output.csv] [Sampling_step, default=1.0]\n");
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
			LOG("Failed to open OpenDRIVE file %s", argv[1]);
			return -1;
		}
		file.open(output_file_name);
	}
	catch (std::exception& e) 
	{ 
		LOG("exception: %s", e.what()); 
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
			int steps = (int)((s_end - s_start) / step_length_target);
			double step_length = steps > 0 ? (s_end - s_start) / steps : s_end - s_start;

			for (int j = 0; j < lane_section->GetNumberOfLanes(); j++)
			{
				Lane *lane = lane_section->GetLaneByIdx(j);

				file << "lane, " << road->GetId() << ", " << i << ", " << lane->GetId() << (lane->IsDriving() ? ", driving" : ", no-driving") << std::endl;
				double s = s_start;
				for (int k = 0; k < steps + 1; k++)
				{
					s += k * step_length;
					s = MIN(s_end, s);
					pos->SetLanePos(road->GetId(), lane->GetId(), s, SIGN(lane->GetId())*lane_section->GetWidth(s, lane->GetId())*0.5, i);
					file << pos->GetX() << ", " << pos->GetY() << ", " << pos->GetZ() << ", " << pos->GetH() << std::endl;
				}
			}
		}
	}
	file.close();

	delete pos;

	LOG("Created %s using stepsize %.2f", output_file_name.c_str(), step_length_target);
	LOG("To plot it, run EnvironmentSimulator/Applications/odrplot/xodr.py %s", output_file_name.c_str());

	return 0;
}
