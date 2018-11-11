#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "CommonMini.hpp"

struct EntityStruct
{
	std::string name;
};

class OSCCondition
{
public:
	struct
	{
		struct
		{
			std::vector<EntityStruct> Entity;
			std::string rule; // Wrong type
		} TriggeringEntities;

		struct
		{
			struct
			{
				std::string entity;
				std::string value; // Wrong type
				std::string freespace;  // Wrong type
				std::string alongRoute;  // Wrong type
				std::string rule; // Wrong type
			}TimeHeadway;

		}EntityCondition;

	} ByEntity;

	struct
	{
		struct {}AtStart;
		struct 
		{
			std::string type; // Wrong type
			std::string name;
			std::string rule; // Wrong type
		}AfterTermination;
		struct {}Command;
		struct {}Signal;
		struct {}Controller;
	} ByState;	

	struct
	{
		struct {}Parameter;
		struct {}TimeOfDay;
		struct {
			double value;
			std::string rule; //Wrong type
		} SimulationTime;

	} ByValue;

	std::string name;
	double delay;
	std::string edge; //Wrong type;

	void printOSCCondition()
	{
		LOG("\tname = %s", name.c_str());
		LOG("\tdelay = %.2f", delay);
		LOG("\tedge = %s", edge.c_str());
		LOG("\t- TriggeringEntities");
		LOG("\trule = %s", ByEntity.TriggeringEntities.rule.c_str());

		for (size_t i = 0; i < ByEntity.TriggeringEntities.Entity.size(); i++)
		{
			LOG("- ByEntity - TriggeringEntities - Entity");
			LOG("name = %s", ByEntity.TriggeringEntities.Entity[i].name.c_str());
		}

		LOG("\t- ByEntity - EntityCondition - TimeHeadway");
		LOG("\tentity = %s", ByEntity.EntityCondition.TimeHeadway.entity.c_str());
		LOG("\tvalue = %.2f", ByEntity.EntityCondition.TimeHeadway.value);
		LOG("\tfreespace = %s", ByEntity.EntityCondition.TimeHeadway.freespace.c_str());
		LOG("\talongRoute = %s", ByEntity.EntityCondition.TimeHeadway.alongRoute.c_str());
		LOG("\trule = %s", ByEntity.EntityCondition.TimeHeadway.rule.c_str());

		}
};
