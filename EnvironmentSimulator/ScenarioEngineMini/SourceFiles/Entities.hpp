#pragma once
#include "OSCCatalogReference.hpp"
#include "OSCVehicle.hpp"
#include "OSCPedestrian.hpp"
#include "OSCMiscObject.hpp"
#include "OSCDriver.hpp"
#include "OSCPedestrianController.hpp"

#include <iostream>
#include <string>
#include <vector>

class Entities
{

public:

	Entities();

	void printEntities();

//private:
	struct propertiesStruct
	{
		struct
		{
			std::string name;
			std::string value;
		} Property;
	} Properties;

	struct ObjectStruct
	{
		std::string name;

			OSCCatalogReference CatalogReference;
			OSCVehicle Vehicle;
			OSCPedestrian Pedestrian;
			OSCMiscObject MiscObject;

			/*struct
			{
				struct
				{
					std::string name;
					std::string value;
				} Property;
			} Properties;*/

			std::vector<propertiesStruct> Properties;

			struct
			{
				OSCCatalogReference CatalogReference;
				OSCDriver Driver;
				OSCPedestrianController PedestrianController;
			} Controller;
	};

	std::vector<ObjectStruct> Object;

};

