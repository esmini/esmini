
#include <thread>
#include <chrono>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

#include "vehicle.hpp"
#include "viewer.hpp"
#include "RoadManager.hpp"
#include "RubberbandManipulator.h"

#ifdef _WIN32
	#include <windows.h>
#endif

using namespace std::chrono;

#define USE_ROUTE 0
#define EGO_MODEL_FILENAME "../../resources/models/p1800.osgb"

static const double stepSize = 0.01;
static const double maxStepSize = 0.1;
static const double minStepSize = 0.01;
static const bool freerun = true;
static vehicle::Vehicle *ego;
static double egoWheelAngle = 0;
static double egoAcc = 0;

double deltaSimTime;  // external - used by Viewer::RubberBandCamera

typedef struct
{
	int road_id_init;
	int lane_id_init;
	roadmanager::Position *pos;
	double speed;  // Velocity along road reference line, m/s
	viewer::CarModel *graphics_model;
	vehicle::Vehicle *vehicle;
} Car;

Car *car;

bool KeyUpPressed()
{
#ifdef _WIN32
	return (GetKeyState(VK_UP) & 0x8000);
#else
	printf("KeyUpPressed only implemented for Windows, so far\n");
#endif
}

bool KeyDownPressed()
{
#ifdef _WIN32
	return (GetKeyState(VK_DOWN) & 0x8000);
#else
	printf("KeyDownPressed only implemented for Windows, so far\n");
#endif
}

bool KeyLeftPressed()
{
#ifdef _WIN32
	return (GetKeyState(VK_LEFT) & 0x8000);
#else
	printf("KeyLeftPressed only implemented for Windows, so far\n");
#endif
}

bool KeyRightPressed()
{
#ifdef _WIN32
	return (GetKeyState(VK_RIGHT) & 0x8000);
#else
	printf("KeyRightPressed only implemented for Windows, so far\n");
#endif
}

int SetupEgo(roadmanager::OpenDrive *odrManager, viewer::Viewer *viewer)
{
	car = new Car;
	car->road_id_init = odrManager->GetRoadByIdx(0)->GetId();
	car->lane_id_init = -1;
	car->pos = new roadmanager::Position(car->road_id_init, car->lane_id_init, 10, 0);

	car->graphics_model = viewer->AddCar(0);
	car->speed = 0;
	car->vehicle = new vehicle::Vehicle(car->pos->GetX(), car->pos->GetY(), car->pos->GetH(), car->graphics_model->size_x);

	return 0;
}

int main(int argc, char** argv)
{
	// use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);	

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName());
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options]\n");
	arguments.getApplicationUsage()->addCommandLineOption("--odr <filename>", "OpenDRIVE filename");
	arguments.getApplicationUsage()->addCommandLineOption("--model <filename>", "3D model filename");

	if (arguments.argc() < 2)
	{
		arguments.getApplicationUsage()->write(std::cout, 1, 120, true);
		return -1;
	}

	std::string odrFilename;
	arguments.read("--odr", odrFilename);

	std::string modelFilename;
	arguments.read("--model", modelFilename);

	roadmanager::Position *lane_pos = new roadmanager::Position();
	roadmanager::Position *track_pos = new roadmanager::Position();

	try
	{
		if (!roadmanager::Position::LoadOpenDrive(odrFilename.c_str()))
		{
			printf("Failed to load ODR %s\n", odrFilename.c_str());
			return -1;
		}
		roadmanager::OpenDrive *odrManager = roadmanager::Position::GetOpenDrive();

#if USE_ROUTE
		// Test route concept 
		// Specify hardcoded route on Fabriksgatan
		roadmanager::Position waypoint[2];
#if 1
		waypoint[0].SetLanePos(2, -1, 250, 0);
		waypoint[1].SetLanePos(1, -1, 10, 0);
#else
		waypoint[0].SetLanePos(0, 1, 50, 0);
		waypoint[1].SetLanePos(1, -1, 10, 0);
#endif
		roadmanager::Route route;

		route.AddWaypoint(&waypoint[0]);
		route.AddWaypoint(&waypoint[1]);
#endif


		viewer::Viewer *viewer = new viewer::Viewer(
			odrManager, 
			modelFilename.c_str(),
			arguments);

		SetupEgo(odrManager, viewer);

		__int64 now, lastTimeStamp = 0;

		while (!viewer->osgViewer_->done())
		{
			// Get milliseconds since Jan 1 1970
			now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
			deltaSimTime = (now - lastTimeStamp) / 1000.0;  // step size in seconds
			lastTimeStamp = now;
			if (deltaSimTime > maxStepSize) // limit step size
			{
				deltaSimTime = maxStepSize;
			}
			else if (deltaSimTime < minStepSize)  // avoid CPU rush, sleep for a while
			{
				std::this_thread::sleep_for(milliseconds((int)(1000 * (minStepSize - deltaSimTime))));
				deltaSimTime = minStepSize;
			}

			// Update vehicle dynamics/driver model
#if !USE_ROUTE
			vehicle::THROTTLE accelerate = vehicle::THROTTLE_NONE;
			if (KeyUpPressed()) 
			{						
				accelerate = vehicle::THROTTLE_ACCELERATE;
			}
			else if (KeyDownPressed())
			{
				accelerate = vehicle::THROTTLE_BRAKE;
			}

			vehicle::STEERING steer = vehicle::STEERING_NONE;
			if (KeyLeftPressed())
			{
				steer = vehicle::STEERING_LEFT;
			}
			else if (KeyRightPressed())
			{
				steer = vehicle::STEERING_RIGHT;
			}

			// Update vehicle motion
			car->vehicle->Update(deltaSimTime, accelerate, steer);
			
			// Set OpenDRIVE position
			car->pos->SetXYH(car->vehicle->posX_, car->vehicle->posY_, car->vehicle->heading_);
#else
			route.MoveDS(deltaSimTime * 20  / 3.6);
			route.GetPosition(car->pos);
			car->ego->SetPos(car->pos->GetX(), car->pos->GetY(), car->pos->GetZ(), car->pos->GetH());
			car->ego->SetWheelAngle(car->ego->heading_ - car->pos->GetH());
			car->ego->SetWheelRotation(route.GetS() / 0.35);
#endif
					
			// Fetch Z and Pitch from OpenDRIVE position
			car->vehicle->posZ_ = car->pos->GetZ();
			car->vehicle->pitch_ = car->pos->GetP();

			// update 3D model transform
			car->graphics_model->SetPosition(car->vehicle->posX_, car->vehicle->posY_, car->vehicle->posZ_);
			car->graphics_model->SetRotation(car->vehicle->heading_, car->vehicle->pitch_, 0.0);
			car->graphics_model->UpdateWheels(car->vehicle->wheelAngle_, car->vehicle->wheelRotation_);

			// Update road debug lines 
			track_pos->SetTrackPos(car->pos->GetTrackId(), car->pos->GetS(), 0);
			viewer->UpdateVPoints(track_pos->GetX(), track_pos->GetY(), lane_pos->GetX(), lane_pos->GetY(), lane_pos->GetZ());

			lane_pos->SetLanePos(car->pos->GetTrackId(), car->pos->GetLaneId(), car->pos->GetS(), 0);
			viewer->UpdateVLine(lane_pos->GetX(), lane_pos->GetY(), lane_pos->GetZ());

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

	delete(car);
	delete track_pos;
	delete lane_pos;

	return 0;
}
