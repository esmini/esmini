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
	roadmanager::Route *route_;
	double heading_; 
	double speed_;

	OSCCatalogReference catalog_reference_;
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

	Object() : id_(0), extern_control_(false), speed_(0), route_(0) {}
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

