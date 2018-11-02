#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

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
		std::cout << "\t" << "name = " << name << std::endl;
		std::cout << "\t" << "delay = " << delay << std::endl;
		std::cout << "\t" << "edge = " << edge << std::endl;
		std::cout << std::endl;

		std::cout << "\t" << "- TriggeringEntities" << std::endl;
		std::cout << "\t" << "rule = " << ByEntity.TriggeringEntities.rule << std::endl;
		std::cout << std::endl;

		for (size_t i = 0; i < ByEntity.TriggeringEntities.Entity.size(); i++)
		{
			std::cout << "\t" << "- ByEntity - TriggeringEntities - Entity" << std::endl;
			std::cout << "\t" << "name = " << ByEntity.TriggeringEntities.Entity[i].name << std::endl;
			std::cout << std::endl;
		}

		std::cout << "\t" << "- ByEntity - EntityCondition - TimeHeadway" << edge << std::endl;
		std::cout << "\t" << "entity = " << ByEntity.EntityCondition.TimeHeadway.entity << std::endl;
		std::cout << "\t" << "value = " << ByEntity.EntityCondition.TimeHeadway.value << std::endl;
		std::cout << "\t" << "freespace = " << ByEntity.EntityCondition.TimeHeadway.freespace << std::endl;
		std::cout << "\t" << "alongRoute = " << ByEntity.EntityCondition.TimeHeadway.alongRoute << std::endl;
		std::cout << "\t" << "rule = " << ByEntity.EntityCondition.TimeHeadway.rule << std::endl;
		std::cout << std::endl;

		}
};
