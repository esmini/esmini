#pragma once
#include <Action.hpp>
#include <Cars.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

class Actions
{
public:

	Actions()
	{
		oldSimulationTime = 0;
	};
	void addAction(Action action);
	void executeActions(double simulationTime);
	void setStartAction(std::vector<int> storyId, double simulationTime);

private:
	double oldSimulationTime;

	std::vector<Action> actions;
};

