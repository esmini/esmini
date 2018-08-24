#pragma once
#include "OSCPosition.hpp"

#include <iostream>
#include <string>
#include <math.h>

struct OSCPrivateAction
{
	bool exists = false;

	struct
	{
		bool exists = false;
		struct
		{
			bool exists = false;
			struct
			{
				bool exists = false;
				std::string shape = ""; // Wrong type
				double rate = NAN;
				double time = NAN;
				double distance = NAN;
			} Dynamics;
			struct
			{
				bool exists = false;
				struct
				{
					bool exists = false;
					std::string object = "";
					double value = NAN;
					std::string valueType = ""; // Wrong type
					bool continuous = false;

				} Relative;
				struct
				{
					bool exists = false;
					double value = NAN;
				} Absolute;
			} Target;
		} Speed;

		struct {
			bool exists = false;
		} Distance;
	} Longitudinal;

	struct
	{
		struct
		{
			double targetLaneOffset = NAN;

			struct
			{
				double time = NAN;
				double distance = NAN;
				std::string shape = ""; // Wrong type

			} Dynamics;
			struct
			{
				struct
				{
					std::string object = "";
					double value = NAN;

				} Relative;
				struct
				{
					double value = NAN;
				} Absolute;

			} Target;

		} LaneChange;
	} Lateral;

	struct {} Visibility;
	struct {} Meeting;
	struct {} Autonomous;
	struct {} Controller;
	OSCPosition Position;
	struct {} Routing;

	void printOSCPrivateAction()
	{
		std::cout << "\t" << " - Longitudinal - Speed - Dynamics" << std::endl;
		std::cout << "\t" << "shape = " << Longitudinal.Speed.Dynamics.shape << std::endl;
		std::cout << "\t" << "rate = " << Longitudinal.Speed.Dynamics.rate << std::endl;
		std::cout << "\t" << "time = " << Longitudinal.Speed.Dynamics.time << std::endl;
		std::cout << "\t" << "distance = " << Longitudinal.Speed.Dynamics.distance << std::endl;
		std::cout << std::endl;

		std::cout << "\t" << " - Longitudinal - Speed - Target - Relative" << std::endl;
		std::cout << "\t" << "object = " << Longitudinal.Speed.Target.Relative.object << std::endl;
		std::cout << "\t" << "value = " << Longitudinal.Speed.Target.Relative.value << std::endl;
		std::cout << "\t" << "valueType = " << Longitudinal.Speed.Target.Relative.valueType << std::endl;
		std::cout << "\t" << "continuous = " << Longitudinal.Speed.Target.Relative.continuous << std::endl;
		std::cout << std::endl;

		std::cout << "\t" << " - Longitudinal - Speed - Target - Absolute" << std::endl;
		std::cout << "\t" << "value = " << Longitudinal.Speed.Target.Absolute.value << std::endl;
		std::cout << std::endl;

		std::cout << "\t" << " - Position" << std::endl;
		Position.printOSCPosition();
		std::cout << std::endl;


	};

};