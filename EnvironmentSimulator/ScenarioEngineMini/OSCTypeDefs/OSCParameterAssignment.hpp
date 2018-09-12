#pragma once
#include <iostream>
#include <string>

struct OSCParameterAssignment
{
	struct
	{
		std::string name;
		std::string value;
	}Parameter;
	

	void printOSCParameterAssignment()
	{
		std::cout << " - ParameterAssignment" << std::endl;
		std::cout << std::endl;
	};
};