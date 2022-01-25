#include <iostream>

#include "RoadManager.hpp"

#define SUPPRESS_LOG

int main(int argc, char* argv[])
{
    std::string filename;
    std::cout << "Enter filename (full path) to xodr file" << std::endl;
    std::cin >> filename;

    auto openDrive = roadmanager::OpenDrive(filename.c_str());
    
    for(auto i = 0; i < openDrive.GetNumOfRoads(); i++) {
        auto road = openDrive.GetRoadByIdx(i);
        
        for(auto j = 0; j < road->GetNumberOfLaneSections(); j++) {
            auto laneSection = road->GetLaneSectionByIdx(j);
            
            for(auto k = 0; k < laneSection->GetNumberOfLanes(); k++) {
                auto lane = laneSection->GetLaneByIdx(k);
                
                for(auto l = 0; l < lane->GetNumberOfRoadMarks(); l++) {
                    auto roadMark = lane->GetLaneRoadMarkByIdx(l);
                    std::cout << "road mark for lane " 
                    << lane->GetId() << " has width " 
                    << roadMark->GetWidth() << " and color " 
                    << static_cast<std::underlying_type<roadmanager::RoadMarkColor>::type>(roadMark->GetColor()) << std::endl;                    
                }
            }
        }
    }

    std::cout << "\nGeoref tag: " << std::endl;
    std::cout << openDrive.GetGeoReferenceAsString() << std::endl;

	return 0;
}
