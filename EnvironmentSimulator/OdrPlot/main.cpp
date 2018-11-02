
#include <cmath>
#include <iostream>
#include <fstream>
#include "roadmanager.hpp"

using namespace roadmanager;

//#define REF_ONLY
#define MIN(x, y) ((x)<(y)?(x):(y))

int main(int argc, char *argv[])
{
	if (argc < 2)
	{
		printf("Usage: roadmanager <OpenDRIVE filename>\n");
	}
	try
	{
		Position::LoadOpenDrive(argv[1]);
	}
	catch (std::exception& e) 
	{ 
		std::cerr << "exception: " << e.what() << std::endl; 
	}

	std::ofstream file;
	file.open("track.csv");

	Position* pos = new Position();
	double step_length_target = 1;
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
			double step_length = (s_end - s_start) / steps;

#ifdef REF_ONLY
			Lane *lane = lane_section->GetLaneById(0);
#else
			for (int j = 0; j < lane_section->GetNumberOfLanes(); j++)
			{
				Lane *lane = lane_section->GetLaneByIdx(j);
				if (!lane->IsDriving() && lane->GetId() != 0)
				{
					continue;
				}
#endif

				file << "lane, " << road->GetId() << ", " << i << ", " << lane->GetId() << std::endl;
				for (int k = 0; k < steps + 1; k++)
				{
					pos->SetLanePos(road->GetId(), lane->GetId(), MIN(s_end, s_start + k * step_length), 0, i);
					file << pos->GetX() << ", " << pos->GetY() << ", " << pos->GetZ() << ", " << pos->GetH() << std::endl;
				}
#ifndef REF_ONLY
			}
#endif
		}
		
	}
	file.close();
//	od->Print();

	delete pos;

	return 0;
}