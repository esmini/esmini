#pragma once
#include "OSCGlobalAction.hpp"
#include "OSCUserDefinedAction.hpp"
#include "OSCPrivateAction.hpp"

#include <iostream>
#include <string>
#include <vector>

class Init
{

public:
	Init();
	void printInit();

	typedef struct 
	{
		bool exists;
		std::vector<OSCPrivateAction> Action;
		std::string object;
	} PrivateStruct;


	struct
	{
		std::vector<OSCGlobalAction> GlobalAction;
		std::vector<OSCUserDefinedAction> UserDefinedAction;
		std::vector<PrivateStruct> Private;
	} Actions;
};