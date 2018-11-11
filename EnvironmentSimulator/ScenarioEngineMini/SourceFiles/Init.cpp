#include "Init.hpp"
#include "CommonMini.hpp"

Init::Init()
{
}


void Init::printInit()
{
	for (int i = 0; i < Actions.GlobalAction.size(); i++)
	{
		Actions.GlobalAction[i].printOSCGlobalAction();
	}

	for (int i = 0; i < Actions.UserDefinedAction.size(); i++)
	{
		Actions.UserDefinedAction[i].printOSCUserDefinedAction();
	}

	for (int i = 0; i < Actions.Private.size(); i++)
	{
		LOG("\tobject[%d] = %s", i, Actions.Private[i].object.c_str());

		for (int j = 0; j < Actions.Private[i].Action.size(); j++)
		{
			LOG("\tAction %d: ", j);
			Actions.Private[i].Action[j].printOSCPrivateAction();
		}
	}
}

