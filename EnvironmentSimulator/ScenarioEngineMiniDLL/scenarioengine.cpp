#include "scenarioengine.h"

ScenarioEngine *scenarioEngine;
ScenarioGateway *scenarioGateway;

extern "C"
{
	int UNITY_DLL_API step(double dt)
	{
		return 0;
	}
}