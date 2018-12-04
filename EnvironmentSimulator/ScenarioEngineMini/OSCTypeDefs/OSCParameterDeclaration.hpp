#pragma once

#include <iostream>
#include <string>

#include <iostream>
#include <string>
#include <vector>

#include "CommonMini.hpp"

namespace scenarioengine
{

	struct ParameterStruct
	{
		std::string name;
		std::string type; // Wrong type
		std::string value;
	};

	class OSCParameterDeclaration
	{
	public:

		std::vector<ParameterStruct> Parameter;

		void printOSCParameterDeclaration()
		{
			LOG("\n");
		};
	};

}