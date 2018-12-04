#pragma once
#include "OSCParameterAssignment.hpp"

#include <iostream>
#include <string>
#include "CommonMini.hpp"

namespace scenarioengine
{

	class OSCCatalogReference
	{
	public:
		std::string catalog_name_;
		std::string entry_name_;

		OSCParameterAssignment ParameterAssignment;

		void Print()
		{
			LOG("CatalogReference: %s, %s", catalog_name_.c_str(), entry_name_.c_str());
		};
	};

}