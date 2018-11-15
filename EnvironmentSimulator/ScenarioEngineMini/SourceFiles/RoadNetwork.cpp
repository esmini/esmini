#include "RoadNetwork.hpp"
#include "CommonMini.hpp"

RoadNetwork::RoadNetwork()
{
}

void RoadNetwork::Print()
{
	LOG("Logics: ");
	Logics.Print();
	LOG("SceneGraph: %s");
	SceneGraph.Print();
}
