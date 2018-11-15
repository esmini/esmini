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
	void Print();

	typedef struct 
	{
		std::vector<OSCPrivateAction*> action_;
		std::string object_;
	} PrivateInitAction;

	std::vector<PrivateInitAction*> private_action_;
	std::vector<OSCGlobalAction*> global_action_;
	std::vector<OSCUserDefinedAction*> user_defined_action_;
};