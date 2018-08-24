#include <iostream>
#include <string>
#include <random>
#include <thread>
#include <chrono>

#include "ScenarioReader.hpp"
#include "Entities.hpp"
#include "Init.hpp"
#include "Story.hpp"
#include "ScenarioEngine.hpp"

#include "viewer.hpp"
#include "RoadManager.hpp"
#include "RubberbandManipulator.h"

double deltaSimTime;

static const double stepSize = 0.01;
static const double maxStepSize = 0.01;
static const double minStepSize = 0.001;
static const bool freerun = true;
static std::mt19937 mt_rand;

int main(int argc, char *argv[])
{	
	// Simulation constants
	double endTime = 100;
	double simulationTime = 0;
	double timeStep = 1;

	// use an ArgumentParser object to manage the program arguments.
	osg::ArgumentParser arguments(&argc, argv);

	arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
	arguments.getApplicationUsage()->setDescription(arguments.getApplicationName());
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options]\n");
	arguments.getApplicationUsage()->addCommandLineOption("--osc <filename>", "OpenSCENARIO filename");

	if (arguments.argc() < 2)
	{
		arguments.getApplicationUsage()->write(std::cout, 1, 80, true);
		return -1;
	}

	std::string oscFilename;
	arguments.read("--osc", oscFilename);

	std::string scenegraphFilename;
	arguments.read("--model", scenegraphFilename);

	std::string odrFilename;
	arguments.read("--odr", odrFilename);

	// Initialization
	ScenarioReader scenarioReader;
	Entities entities;
	Init init;
	std::vector<Story> story;
	viewer::Viewer *viewer = new viewer::Viewer(roadmanager::Position::GetOpenDrive(), scenegraphFilename.c_str(), arguments);

	// Load and parse data
	scenarioReader.loadXmlFile(oscFilename.c_str());
	scenarioReader.parseParameterDeclaration();
	scenarioReader.parseEntities(entities);
	scenarioReader.parseInit(init);
	scenarioReader.parseStory(story);

	// Print loaded data
	entities.printEntities();
	init.printInit();
	story[0].printStory();

	// ScenarioEngine
	ScenarioEngine scenarioEngine(entities, init, story, simulationTime);
	scenarioEngine.initCarVector();
	scenarioEngine.printCarVector();
	scenarioEngine.initConditions();

	// Init road manager
	if (!roadmanager::Position::LoadOpenDrive(odrFilename.c_str()))
	{
		printf("Failed to load ODR %s\n", odrFilename.c_str());
		return -1;
	}
	roadmanager::OpenDrive *odrManager = roadmanager::Position::GetOpenDrive();

	//  Create cars for visualization
	for (int i = 0; i < scenarioEngine.carVector.size(); i++)
	{
		int carModelID = (double(viewer->carModels_.size()) * mt_rand()) / (std::mt19937::max)();
		viewer->AddCar(carModelID);
	}

	__int64 now, lastTimeStamp = 0;
	double simTime = 0;

	while (simTime<100.0)
	{
		// Get milliseconds since Jan 1 1970
		now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		deltaSimTime = (now - lastTimeStamp) / 1000.0;  // step size in seconds
		lastTimeStamp = now;

		if (deltaSimTime > maxStepSize) // limit step size
		{
			deltaSimTime = maxStepSize;
		}
		else if (deltaSimTime < minStepSize)  // avoid CPU rush, sleep for a while
		{
			std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000 * (minStepSize - deltaSimTime))));
			deltaSimTime = minStepSize;
		}

		simTime = simTime + deltaSimTime;

		scenarioEngine.setSimulationTime(simTime);
		scenarioEngine.setTimeStep(deltaSimTime);
		scenarioEngine.executeActions();
		scenarioEngine.stepObjects(deltaSimTime);
		scenarioEngine.checkConditions();

		//scenarioEngine.printSimulationTime();
		//scenarioEngine.printCarVector();

		// Visualize cars
		for (int i = 0; i<scenarioEngine.carVector.size(); i++)
		{

			// Fulkod
			int roadId = 1;// std::stoi(scenarioEngine.carVector[i].getPosition().Lane.roadId);
			int laneId = scenarioEngine.carVector[i].getPosition().Lane.laneId;
			int s = scenarioEngine.carVector[i].getPosition().Lane.s;
			int offset = scenarioEngine.carVector[i].getPosition().Lane.offset;

			viewer::CarModel *car = viewer->cars_[i];
			roadmanager::Position p(roadId, laneId, s, offset);
			car->txNode_->setPosition(osg::Vec3(p.GetX(), p.GetY(), p.GetZ()));
			
			float roll = 0;
			float pitch = 0;
			float heading = 0;

			car->quat_.makeRotate(
				roll, osg::Vec3(1, 0, 0),
				pitch, osg::Vec3(0, 1, 0),
				heading, osg::Vec3(0, 0, 1));
			car->txNode_->setAttitude(car->quat_);

		}

		viewer->osgViewer_->frame();

	}

	

	// Wait for an "Enter"
	std::cout << "\n" << "Press ENTER to quit" << std::endl;
	std::cin.get();

	return 1;
}


