#pragma once
#include "OSCCatalogReference.hpp"
#include "OSCVehicle.hpp"
#include "OSCPedestrian.hpp"
#include "OSCMiscObject.hpp"
#include "OSCDriver.hpp"
#include "OSCPedestrianController.hpp"
#include "roadmanager.hpp"

#include <iostream>
#include <string>
#include <vector>

class Object
{
public:

	struct Property
	{
		std::string name_;
		std::string value_;
	};

	std::string name_;
	bool extern_control_;
	int id_;
	roadmanager::Position pos_;
	double speed_;
	bool follow_route_;

	OSCCatalogReference CatalogReference;
	OSCVehicle Vehicle;
	OSCPedestrian Pedestrian;
	OSCMiscObject MiscObject;

	std::vector<Property*> properties_;

	struct
	{
		OSCCatalogReference CatalogReference;
		OSCDriver Driver;
		OSCPedestrianController PedestrianController;
	} Controller;

	Object() : extern_control_(false), speed_(0), follow_route_(false) {}
};

class Entities
{

public:

	Entities() {}

	void Print()
	{
		LOG("");
	}

	std::vector<Object*> object_;

};

