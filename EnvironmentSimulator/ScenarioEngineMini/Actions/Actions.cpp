#include "Actions.hpp"


Actions::Actions()
{
}

void Actions::addAction(Action action)
{
	actions.push_back(action);
}

void Actions::setStartAction(std::vector<int> storyId, double simulationTime)
{
	for (size_t i = 0; i < actions.size(); i++)
	{
		if (actions[i].getStoryId() == storyId)
		{
			actions[i].setStartAction();
			actions[i].setStartTime(simulationTime);
		}
	}
}

void Actions::executeActions(double simulationTime)
{
	double timeStep = simulationTime - oldSimulationTime;

	for (size_t i = 0; i < actions.size(); i++)
	{
		if (actions[i].getStartAction())
		{
			actions[i].ExecuteAction(simulationTime, timeStep);
		}

		if (actions[i].getActionCompleted())
		{
			std::cout << "Actions: " << " action " << actions[i].getActionType() << " is removed from actions" << "\n" << std::endl;
			actions.erase(actions.begin() + i);
		}
	}

	oldSimulationTime = simulationTime;
}