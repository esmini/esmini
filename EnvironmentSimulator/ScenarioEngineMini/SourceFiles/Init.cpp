#include "Init.hpp"

Init::Init()
{
	std::cout << "Init: New Init created" << std::endl;
}


void Init::printInit()
{
	std::cout << "---------------------------------------" << std::endl;
	std::cout << "Print of Init " << "\n" << std::endl;

	std::cout << "Global:" << std::endl;
	for (int i = 0; i < Actions.GlobalAction.size(); i++)
	{
		Actions.GlobalAction[i].printOSCGlobalAction();
	}

	std::cout << "UserDefinedAction:" << std::endl;
	for (int i = 0; i < Actions.UserDefinedAction.size(); i++)
	{
		Actions.UserDefinedAction[i].printOSCUserDefinedAction();
	}

	std::cout << "Private:" << std::endl;
	for (int i = 0; i < Actions.Private.size(); i++)
	{
		std::cout << "\t" << "Private iterator counter = " << i << std::endl;

		std::cout << "\t" << "object = " << Actions.Private[i].object << std::endl;

		for (int j = 0; j < Actions.Private[i].Action.size(); j++)
		{
			std::cout << "\t" << "Action iterator counter = " << j << std::endl;
			Actions.Private[i].Action[j].printOSCPrivateAction();
		}
	}

	std::cout << "---------------------------------------" << std::endl;
	std::cout << "\n" << std::endl;
}

