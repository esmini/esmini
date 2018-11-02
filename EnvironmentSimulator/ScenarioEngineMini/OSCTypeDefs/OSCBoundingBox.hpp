#pragma once
#include <iostream>
#include <string>
#include <math.h>

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
		std::cout << " - center " << std::endl;
		std::cout << "\t" << "x = " << center.x << std::endl;
		std::cout << "\t" << "y = " << center.y << std::endl;
		std::cout << "\t" << "z = " << center.z << std::endl;
		std::cout << std::endl;

		std::cout << "- dimension " << std::endl;
		std::cout << "\t" << "width = " << dimension.width << std::endl;
		std::cout << "\t" << "length = " << dimension.length << std::endl;
		std::cout << "\t" << "height = " << dimension.height << std::endl;
		std::cout << std::endl;

	}
};