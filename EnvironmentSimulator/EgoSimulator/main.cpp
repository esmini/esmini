
#include <random>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

#include "vehicle.hpp"
#include "viewer.hpp"
#include "ScenarioEngine.hpp"
#include "ScenarioGateway.hpp"
#include "RoadManager.hpp"
#include "RubberbandManipulator.h"
#include "CommonMini.hpp"

#define USE_ROUTE 0
#define EGO_MODEL_FILENAME "../../resources/models/p1800.osgb"
#define EGO_ID 0	// need to match appearing order in the OpenSCENARIO file

#define MAX(x, y) (y > x ? y : x)
#define MIN(x, y) (y < x ? y : x)

static const double stepSize = 0.01;
static const double maxStepSize = 0.1;
static const double minStepSize = 0.01;
static const bool freerun = true;
static vehicle::Vehicle *ego;
static double egoWheelAngle = 0;
static double egoAcc = 0;

double deltaSimTime;  // external - used by Viewer::RubberBandCamera
static std::mt19937 mt_rand;

typedef struct
{
	int road_id_init;
	int lane_id_init;
	roadmanager::Position *pos;
	viewer::CarModel *graphics_model;
	vehicle::Vehicle *vehicle;
} EgoCar;

static EgoCar *egoCar;

typedef struct
{
	int id;
	viewer::CarModel *carModel;
	roadmanager::Position pos;
} ScenarioCar;

static std::vector<ScenarioCar> scenarioCar;



int SetupEgo(roadmanager::OpenDrive *odrManager, viewer::Viewer *viewer)
{
	egoCar = new EgoCar;
	egoCar->road_id_init = odrManager->GetRoadByIdx(0)->GetId();
	egoCar->lane_id_init = 1;
	egoCar->pos = new roadmanager::Position(egoCar->road_id_init, egoCar->lane_id_init, 60, 0);

	egoCar->graphics_model = viewer->AddCar(0);
 	egoCar->vehicle = new vehicle::Vehicle(egoCar->pos->GetX(), egoCar->pos->GetY(), egoCar->pos->GetH(), egoCar->graphics_model->size_x);

	return 0;
}

void UpdateEgo(double deltaTimeStep, viewer::Viewer *viewer)
{
	vehicle::THROTTLE accelerate = vehicle::THROTTLE_NONE;
	if (viewer->getKeyUp())
	{
		accelerate = vehicle::THROTTLE_ACCELERATE;
	}
	else if (viewer->getKeyDown())
	{
		accelerate = vehicle::THROTTLE_BRAKE;
	}

	vehicle::STEERING steer = vehicle::STEERING_NONE;
	if (viewer->getKeyLeft())
	{
		steer = vehicle::STEERING_LEFT;
	}
	else if (viewer->getKeyRight())
	{
		steer = vehicle::STEERING_RIGHT;
	}

	// Update vehicle motion
	egoCar->vehicle->Update(deltaSimTime, accelerate, steer);

	// Set OpenDRIVE position
	egoCar->pos->SetXYH(egoCar->vehicle->posX_, egoCar->vehicle->posY_, egoCar->vehicle->heading_);

	// Fetch Z and Pitch from OpenDRIVE position
	egoCar->vehicle->posZ_ = egoCar->pos->GetZ();
	egoCar->vehicle->pitch_ = egoCar->pos->GetP();

	// update 3D model transform
	egoCar->graphics_model->SetPosition(egoCar->vehicle->posX_, egoCar->vehicle->posY_, egoCar->vehicle->posZ_);
	egoCar->graphics_model->SetRotation(egoCar->vehicle->heading_, egoCar->vehicle->pitch_, 0.0);
	egoCar->graphics_model->UpdateWheels(egoCar->vehicle->wheelAngle_, egoCar->vehicle->wheelRotation_);
}

void UpdateDebugLines(roadmanager::Position *lane_pos, roadmanager::Position *track_pos, viewer::Viewer *viewer)
{
	track_pos->SetTrackPos(egoCar->pos->GetTrackId(), egoCar->pos->GetS(), 0);
	viewer->UpdateVPoints(track_pos->GetX(), track_pos->GetY(), lane_pos->GetX(), lane_pos->GetY(), lane_pos->GetZ());

	lane_pos->SetLanePos(egoCar->pos->GetTrackId(), egoCar->pos->GetLaneId(), egoCar->pos->GetS(), 0);
	viewer->UpdateVLine(lane_pos->GetX(), lane_pos->GetY(), lane_pos->GetZ());
}

ScenarioCar *getScenarioCarById(int id)
{
	for (size_t i=0; i<scenarioCar.size(); i++)
	{
		if (scenarioCar[i].id == id)
		{
			return &scenarioCar[i];
		}
	}

	return 0;
}

int main(int argc, char** argv)
{
	ScenarioEngine *scenarioEngine;
	ScenarioGateway *scenarioGateway;
	roadmanager::Position *lane_pos = new roadmanager::Position();
	roadmanager::Position *track_pos = new roadmanager::Position();

	double simTime = 0;

	// use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);	

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName());
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options]\n");
	arguments.getApplicationUsage()->addCommandLineOption("--osc <filename>", "OpenSCENARIO filename");

	if (arguments.argc() < 2)
	{
		arguments.getApplicationUsage()->write(std::cout, 1, 120, true);
		return -1;
	}

	std::string oscFilename;
	arguments.read("--osc", oscFilename);

	// Create scenario engine
	try
	{
		scenarioEngine = new ScenarioEngine(oscFilename, simTime);
	}
	catch (const std::exception& e)
	{
		LOG(e.what());
		return -1;
	}

	// Step scenario engine - zero time - just to reach init state
	// Report all vehicles initially - to communicate initial position for external vehicles as well
	scenarioEngine->step(0.0, true);

	// Fetch ScenarioGateway
	scenarioGateway = scenarioEngine->getScenarioGateway();


	try
	{
		roadmanager::OpenDrive *odrManager = scenarioEngine->getRoadManager();

		viewer::Viewer *viewer = new viewer::Viewer(
			odrManager, 
			scenarioEngine->getSceneGraphFilename().c_str(),
			arguments);

		SetupEgo(odrManager, viewer);

		__int64 now, lastTimeStamp = 0;

		while (!viewer->osgViewer_->done())
		{
			// Get milliseconds since Jan 1 1970
			now = SE_getSystemTime();
			deltaSimTime = (now - lastTimeStamp) / 1000.0;  // step size in seconds
			lastTimeStamp = now;
			if (deltaSimTime > maxStepSize) // limit step size
			{
				deltaSimTime = maxStepSize;
			}
			else if (deltaSimTime < minStepSize)  // avoid CPU rush, sleep for a while
			{
				SE_sleep(minStepSize - deltaSimTime);
				deltaSimTime = minStepSize;
			}

			// Update vehicle dynamics/driver model
			UpdateEgo(deltaSimTime, viewer);

			// Time operations
			simTime = simTime + deltaSimTime;
			scenarioEngine->setSimulationTime(simTime);
			scenarioEngine->setTimeStep(deltaSimTime);

			// ScenarioEngine
			scenarioEngine->step(deltaSimTime);

			// Report updated Ego state to scenario gateway
			scenarioGateway->reportObject(ObjectState(EGO_ID, std::string("Ego"), simTime, 
				egoCar->vehicle->posX_, egoCar->vehicle->posY_, egoCar->vehicle->posZ_, 
				egoCar->vehicle->heading_, egoCar->vehicle->pitch_, 0, 
				egoCar->vehicle->speed_));

			// Fetch states of scenario objects
			for (int i = 0; i < scenarioGateway->getNumberOfObjects(); i++)
			{
				ObjectState *o = scenarioGateway->getObjectStatePtrByIdx(i);

				if (o->state_.id != EGO_ID)
				{
					ScenarioCar *sc = getScenarioCarById(o->state_.id);

					// If not available, create it
					if (sc == 0)
					{
						ScenarioCar new_sc;

						LOG("Creating car %d - got state from gateway", o->state_.id);

						new_sc.id = o->state_.id;
						// Choose random model
						int carModelID = (double(viewer->carModels_.size()) * mt_rand()) / (mt_rand.max)();
						new_sc.carModel = viewer->AddCar(carModelID);

						// Add it to the list of scenario cars
						scenarioCar.push_back(new_sc);

						sc = &scenarioCar.back();
					}

					sc->pos = o->state_.pos;
				}
			}

			// Visualize scenario cars
			for (size_t i=0; i<scenarioCar.size(); i++)
			{
				ScenarioCar *c = &scenarioCar[i];
				c->carModel->SetPosition(c->pos.GetX(), c->pos.GetY(), c->pos.GetZ());
				c->carModel->SetRotation(c->pos.GetH(), c->pos.GetR(), c->pos.GetP());
			}

			// Update road debug lines 
			UpdateDebugLines(lane_pos, track_pos, viewer);

			// Find and visualize steering target point
			double steering_target_local[3];
			double steering_target_global[3];
			double angle;
			double dist = MAX(5, egoCar->vehicle->speed_);
			egoCar->pos->GetSteeringTargetPos(dist, steering_target_local, steering_target_global, &angle);
			viewer->UpdateDriverModelPoint(steering_target_global[0], steering_target_global[1], steering_target_global[2]);

			// Update graphics
			viewer->osgViewer_->frame();
		}
	}
	catch (std::logic_error &e)
	{
		printf("%s\n", e.what());
		return 2;
	}
	catch (std::runtime_error &e)
	{
		printf("%s\n", e.what());
		return 3;
	}

	delete(scenarioEngine);
	delete(egoCar);
	delete track_pos;
	delete lane_pos;

	return 0;
}
