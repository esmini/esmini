#pragma once
#include <iostream>
#include <string>
#include "CommonMini.hpp"

namespace scenarioengine
{

	class OSCProperties
	{
	public:
		class Property
		{
		public:
			std::string name_;
			std::string value_;
		};

		class File
		{
		public:
			std::string filepath_;
		};

		std::vector<Property> property_;
		File file_;  // Should be no more than one?
	};

}