
#include <iostream>

#include "vehicle.hpp"
#include "viewer.hpp"
#include "ScenarioEngine.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"

#include <Windows.h>
#include <process.h>

static HANDLE ghMutex;

using namespace scenarioengine;

#define EGO_ID 0	// need to match appearing order in the OpenSCENARIO file
#define MAX(x, y) (y > x ? y : x)
#define MIN(x, y) (y < x ? y : x)

static const double maxStepSize = 0.1;
static const double minStepSize = 0.01;
static vehicle::Vehicle *ego;

double deltaSimTime;  // external - used by Viewer::RubberBandCamera

static bool viewer_running = false;
static ScenarioEngine *scenarioEngine;
static viewer::Viewer *scenarioViewer;

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



int SetupEgo(roadmanager::OpenDrive *odrManager, roadmanager::Position init_pos)
{
	egoCar = new EgoCar;
	egoCar->road_id_init = init_pos.GetTrackId();
	egoCar->lane_id_init = init_pos.GetLaneId();
	egoCar->pos = new roadmanager::Position(init_pos);

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
	egoCar->pos->XYH2TrackPos(egoCar->vehicle->posX_, egoCar->vehicle->posY_, egoCar->vehicle->heading_);

	// Fetch Z and Pitch from OpenDRIVE position
	egoCar->vehicle->posZ_ = egoCar->pos->GetZ();
	egoCar->vehicle->pitch_ = egoCar->pos->GetP();
}

void viewer_thread(void *data)
{
	int firstScenarioVehicle = scenarioEngine->GetExtControl() == true ? 1 : 0;

	// Create viewer
	osg::ArgumentParser *parser = (osg::ArgumentParser *)data;
	scenarioViewer = new viewer::Viewer(roadmanager::Position::GetOpenDrive(), scenarioEngine->getSceneGraphFilename().c_str(), *parser);

	// Create Ego vehicle, 
	if (scenarioEngine->GetExtControl())
	{
		egoCar->graphics_model = scenarioViewer->AddCar(0);
		egoCar->vehicle = new vehicle::Vehicle(egoCar->pos->GetX(), egoCar->pos->GetY(), egoCar->pos->GetH(), egoCar->graphics_model->size_x);
	}

	//  Create cars for visualization
	for (size_t i = firstScenarioVehicle; i < scenarioEngine->entities.object_.size(); i++)
	{
		scenarioViewer->AddCar(scenarioEngine->entities.object_[i]->model_id_);
	}

	while (!scenarioViewer->osgViewer_->done())
	{

		WaitForSingleObject(ghMutex, INFINITE);  // no time-out interval

		// Visualize scenario cars
		for (size_t i = firstScenarioVehicle; i < scenarioEngine->entities.object_.size(); i++)
		{
			viewer::CarModel *car = scenarioViewer->cars_[i];
			roadmanager::Position pos = scenarioEngine->entities.object_[i]->pos_;

			car->SetPosition(pos.GetX(), pos.GetY(), pos.GetZ());
			car->SetRotation(pos.GetH(), pos.GetR(), pos.GetP());
		}

		// Visualize Ego car separatelly, if external control set
		if (scenarioEngine->GetExtControl())
		{
			// update 3D model transform
			egoCar->graphics_model->SetPosition(egoCar->vehicle->posX_, egoCar->vehicle->posY_, egoCar->vehicle->posZ_);
			egoCar->graphics_model->SetRotation(egoCar->vehicle->heading_, egoCar->vehicle->pitch_, 0.0);
			egoCar->graphics_model->UpdateWheels(egoCar->vehicle->wheelAngle_, egoCar->vehicle->wheelRotation_);

			// Update road and vehicle debug lines 
			scenarioViewer->UpdateVehicleLineAndPoints(egoCar->pos);

			// Visualize steering target point
			scenarioViewer->UpdateDriverModelPoint(egoCar->pos, MAX(5, egoCar->vehicle->speed_));
		}

		ReleaseMutex(ghMutex);

		scenarioViewer->osgViewer_->frame();

		viewer_running = true;
	}

	delete scenarioViewer;

	viewer_running = false;
}

void log_callback(const char *str)
{
	printf("%s\n", str);
}

int main(int argc, char** argv)
{
	// Use logger callback
	Logger::Inst().SetCallback(log_callback);

	ScenarioGateway *scenarioGateway;
	roadmanager::OpenDrive *odrManager;
	roadmanager::Position *lane_pos = new roadmanager::Position();
	roadmanager::Position *track_pos = new roadmanager::Position();

	double simTime = 0;

	// use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);	

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName());
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options]\n");
	arguments.getApplicationUsage()->addCommandLineOption("--osc <filename>", "OpenSCENARIO filename");
	arguments.getApplicationUsage()->addCommandLineOption("--ext_control <mode>", "Ego control (\"osc\", \"off\", \"on\")");
	arguments.getApplicationUsage()->addCommandLineOption("--record <file.dat>", "Record position data into a file for later replay");

	if (arguments.argc() < 2)
	{
		arguments.getApplicationUsage()->write(std::cout, 1, 120, true);
		return -1;
	}

	std::string oscFilename;
	arguments.read("--osc", oscFilename);

	std::string record_filename;
	arguments.read("--record", record_filename);

	std::string ext_control_str;
	arguments.read("--ext_control", ext_control_str);

	ExternalControlMode ext_control;
	if (ext_control_str == "osc" || ext_control_str == "") ext_control = ExternalControlMode::EXT_CONTROL_BY_OSC;
	else if(ext_control_str == "off") ext_control = ExternalControlMode::EXT_CONTROL_OFF;
	else if (ext_control_str == "on") ext_control = ExternalControlMode::EXT_CONTROL_ON;
	else
	{
		LOG("Unrecognized external control mode: %s", ext_control_str.c_str());
		ext_control = ExternalControlMode::EXT_CONTROL_BY_OSC;
	}


	// Create scenario engine
	try
	{
		scenarioEngine = new ScenarioEngine(oscFilename, simTime, ext_control);
		odrManager = scenarioEngine->getRoadManager();
	}
	catch (const std::exception& e)
	{
		LOG(e.what());
		return -1;
	}

	// Fetch ScenarioGateway
	scenarioGateway = scenarioEngine->getScenarioGateway();

	// Create a data file for later replay?
	if (!record_filename.empty())
	{
		LOG("Recording data to file %s", record_filename);
		scenarioGateway->RecordToFile(record_filename, scenarioEngine->getOdrFilename(), scenarioEngine->getSceneGraphFilename());
	}

	// Step scenario engine - zero time - just to reach init state
	// Report all vehicles initially - to communicate initial position for external vehicles as well
	scenarioEngine->step(0.0, true);

	if (scenarioEngine->GetExtControl())
	{
		// Setup Ego with initial position from the gateway
		SetupEgo(odrManager, scenarioGateway->getObjectStatePtrByIdx(0)->state_.pos);
	}

	// Launch viewer in a separate thread
	HANDLE thread_handle = (HANDLE)_beginthread(viewer_thread, 0, &arguments);

	// Wait for the viewer to launch
	while (!viewer_running) SE_sleep(100);

	try
	{
		__int64 now, lastTimeStamp = 0;

		while (viewer_running)
		{
			// Get milliseconds since Jan 1 1970
			now = SE_getSystemTime();
			deltaSimTime = (now - lastTimeStamp) / 1000.0;  // step size in seconds
			lastTimeStamp = now;
			double adjust = 0;

			if (deltaSimTime > maxStepSize) // limit step size
			{
				adjust = -(deltaSimTime - maxStepSize);
			}
			else if (deltaSimTime < minStepSize)  // avoid CPU rush, sleep for a while
			{
				adjust = minStepSize - deltaSimTime;
				SE_sleep(adjust * 1000);
				lastTimeStamp += adjust * 1000;
			}

			deltaSimTime += adjust;

			// ScenarioEngine
			WaitForSingleObject(ghMutex, INFINITE);  // no time-out interval

			if (scenarioEngine->GetExtControl())
			{
				// Update vehicle dynamics/driver model
				UpdateEgo(deltaSimTime, scenarioViewer);

				// Report updated Ego state to scenario gateway
				scenarioGateway->reportObject(ObjectState(EGO_ID, std::string("Ego"), 0, 1, simTime,
					egoCar->vehicle->posX_, egoCar->vehicle->posY_, egoCar->vehicle->posZ_,
					egoCar->vehicle->heading_, egoCar->vehicle->pitch_, 0,
					egoCar->vehicle->speed_));
			}
	
			scenarioEngine->step(deltaSimTime);

			ReleaseMutex(ghMutex);
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
