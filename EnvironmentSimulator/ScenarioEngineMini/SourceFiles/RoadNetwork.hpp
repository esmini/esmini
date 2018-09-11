#pragma once
#include <OSCFile.hpp>

#include <iostream>
#include <string>
#include <vector>

class RoadNetwork
{
public:
	RoadNetwork();
	void printRoadNetwork();

	OSCFile Logics;
	OSCFile SceneGraph;

};

