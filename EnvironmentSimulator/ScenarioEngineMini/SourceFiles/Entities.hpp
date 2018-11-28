#pragma once
#include "OSCCatalogReference.hpp"
#include "roadmanager.hpp"

#include <iostream>
#include <string>
#include <vector>


class Object
{
public:
	typedef enum
	{
		VEHICLE,
		PEDESTRIAN,
		MISC_OBJECT
	} Type;

	struct Property
	{
		std::string name_;
		std::string value_;
	};

	Type type_;
	std::string name_;
	bool extern_control_;
	int id_;
	roadmanager::Position pos_;
	roadmanager::Route *route_;
	double heading_; 
	double speed_;

	OSCCatalogReference catalog_reference_;

	struct
	{
		OSCCatalogReference CatalogReference;
	} Controller;

	Object(Type type) : type_(type), id_(0), extern_control_(false), speed_(0), route_(0) {}
};

class Vehicle : public Object
{
public:
	typedef enum
	{
		CAR,
		VAN,
		TRUCK,
		SEMITRAILER,
		BUS,
		MOTORBIKE,
		BICYCLE,
		TRAIN,
		TRAM
	} Category;
	
	Category category_;
	
	Vehicle() : Object(Object::Type::VEHICLE), category_(Category::CAR) {}

	void SetCategory(std::string category)
	{
		if (category == "car")
		{
			category_ = Vehicle::Category::CAR;
		}
		else
		{
			LOG("Vehicle category %s not supported yet", category.c_str());
		}

		return;
	}
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

