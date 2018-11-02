#pragma once

#include <iostream>
#include <string>
#include <math.h>

class OSCOrientation
{
	bool exists;
	std::string type; //Wrong type
	double h;
	double p;
	double r;

public:
	OSCOrientation() : exists(false), type(""), h(0), p(0), r(0) {};

	void printOSCOrientation()
	{
		std::cout << "\t" << " - Orientation" << std::endl;
		std::cout << "\t" << "h = " << h << std::endl;
		std::cout << "\t" << "p = " << p << std::endl;
		std::cout << "\t" << "r = " << r << std::endl;
		std::cout << std::endl;
	};
};