#pragma once

#include <iostream>
#include <string>
#include <math.h>
#include "CommonMini.hpp"

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
		LOG("\t - Orientation");
		LOG("\th = %.2f", h);
		LOG("\tp = %.2f", p);
		LOG("\tr = %.2f", r);
		LOG("\n");
	};
};