#include "RoadNetwork.hpp"

RoadNetwork::RoadNetwork()
{
	std::cout << "RoadNetwork: New RoadNetwork created" << std::endl;
}


void RoadNetwork::printRoadNetwork()
{
	Logics.printOSCFile(); 
	SceneGraph.printOSCFile();
}