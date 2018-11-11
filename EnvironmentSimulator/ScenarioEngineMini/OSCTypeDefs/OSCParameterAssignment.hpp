#pragma once
#include <iostream>
#include <string>
#include "CommonMini.hpp"

class OSCParameterAssignment
{
	struct
	{
		std::string name;
		std::string value;
	}Parameter;
	

	void printOSCParameterAssignment()
	{
		LOG(" - ParameterAssignment");
		LOG("\n");
	};
};