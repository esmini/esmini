#pragma once
#include "OSCGlobalAction.hpp"
#include "OSCUserDefinedAction.hpp"
#include "OSCPrivateAction.hpp"

#include <iostream>
#include <string>
#include <vector>

class Init
{
	//friend class ScenarioReader;

public:
	Init();
	void printInit();

	struct PrivateStruct
	{
		bool exists = false;
		std::vector<OSCPrivateAction> Action;
		std::string object;
	};


	struct
	{
		std::vector<OSCGlobalAction> GlobalAction;
		std::vector<OSCUserDefinedAction> UserDefinedAction;
		std::vector<PrivateStruct> Private;
	} Actions;

};