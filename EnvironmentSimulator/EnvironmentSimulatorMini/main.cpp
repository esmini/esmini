#include <iostream>
#include <string>

#include "ScenarioReader.hpp"
#include "Entities.hpp"
#include "Init.hpp"
#include "Story.hpp"
#include "ScenarioEngine.hpp"


int main(int argc, char *argv[])
{

	if (argc < 2)
	{
		printf("Usage: %s <OSC filename>\n", argv[0]);
		return 0;
	}


	// Initialization
	ScenarioReader scenarioReader;
	Entities entities;
	Init init;
	std::vector<Story> story;

	// Load and parse data
	scenarioReader.loadXmlFile(argv[1]);
	scenarioReader.parseParameterDeclaration();
	scenarioReader.parseEntities(entities);
	scenarioReader.parseInit(init);
	scenarioReader.parseStory(story);

	// Print loaded data
	entities.printEntities();
	init.printInit();
	story[0].printStory();

	// Simulate
	double endTime = 100;
	double simulationTime = 0;
	double timeStep = 1;

	// ScenarioEngine
	ScenarioEngine scenarioEngine(entities, init, story, simulationTime);
	scenarioEngine.initCarVector();
	scenarioEngine.printCarVector();
	scenarioEngine.initConditions();
	
	for (double i = 0; i < endTime; i=i+ timeStep)
	{
		std::cout << "i = " << i << std::endl;
		scenarioEngine.setSimulationTime(i);
		scenarioEngine.setTimeStep(timeStep);
		scenarioEngine.printSimulationTime();

		scenarioEngine.executeActions();

		scenarioEngine.stepObjects(1);
		scenarioEngine.printCarVector();

		scenarioEngine.checkConditions();
	}

	

	// Wait for an "Enter"
	std::cout << "\n" << "Press ENTER to quit" << std::endl;
	std::cin.get();

	return 1;
}


