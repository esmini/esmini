#pragma once
#include <iostream>
#include <string>
#include <math.h>
#include "CommonMini.hpp"

class OSCBoundingBox
{
public:
	struct
	{
		double x;
		double y;
		double z;
	} center;

	struct 
	{
		double width;
		double length;
		double height;
	} dimension;

	void printOSCBoundingBox()
	{
		LOG(" - center ");
		LOG("\tx = %.2f", center.x);
		LOG("\ty = %.2f", center.y);
		LOG("\tz = %.2f", center.z);
		LOG("\n");

		LOG("- dimension ");
		LOG("\twidth = %.2f", dimension.width);
		LOG("\tlength = %.2f", dimension.length);
		LOG("\theight = %.2f", dimension.height);
		LOG("\n");

	}
};