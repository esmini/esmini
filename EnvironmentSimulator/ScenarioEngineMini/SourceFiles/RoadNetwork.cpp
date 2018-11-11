#include "RoadNetwork.hpp"
#include "CommonMini.hpp"

RoadNetwork::RoadNetwork()
{
}


void RoadNetwork::printRoadNetwork()
{
	Logics.printOSCFile(); 
	SceneGraph.printOSCFile();
}