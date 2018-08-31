
#include <thread>
#include <chrono>
#include <random>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

#include "viewer.hpp"
#include "RoadManager.hpp"
#include "RubberbandManipulator.h"
#include "vehicle.hpp"

using namespace std::chrono;

#define USE_ROUTE 1

#define DEFAULT_SPEED   70  // km/h
#define DEFAULT_DENSITY 1   // Cars per 100 m
#define ROAD_MIN_LENGTH 30
#define SIGN(X) ((X<0)?-1:1)
#define EGO_MODEL_FILENAME "../../resources/models/p1800.osgb"

static const double stepSize = 0.01;
static const double maxStepSize = 0.1;
static const double minStepSize = 0.01;
static const bool freerun = true;
static std::mt19937 mt_rand;
static double density = DEFAULT_DENSITY;
static double speed = DEFAULT_SPEED;
static Vehicle *ego;
static double egoWheelAngle = 0;
static double egoAcc = 0;

double deltaSimTime;  // external - used by Viewer::RubberBandCamera

typedef struct
{
	int road_id_init;
	int lane_id_init;
	roadmanager::Position *pos;
	double speed;  // Velocity along road reference line, m/s
	viewer::CarModel *model;
	int id;
	Vehicle *ego;
} Car;

std::vector<Car*> cars;

int SetupCars(roadmanager::OpenDrive *odrManager, viewer::Viewer *viewer)
{
	// Add one Ego car
	Car *car_ = new Car;
	car_->road_id_init = odrManager->GetRoadByIdx(0)->GetId();
	car_->lane_id_init = -1;
	car_->pos = new roadmanager::Position(car_->road_id_init, car_->lane_id_init, 10, 0);

	car_->model = viewer->AddCar(0);
	car_->speed = 0;
	car_->id = cars.size();
	car_->ego = new Vehicle(car_->pos->GetX(), car_->pos->GetY(), car_->pos->GetH(), car_->model->size_x);
	cars.push_back(car_);

	if (density < 1E-10)
	{
		// Basically no scenario vehicles
		return 0;
	}

	for (int r = 0; r < odrManager->GetNumOfRoads(); r++)
	{
		roadmanager::Road *road = odrManager->GetRoadByIdx(r);
		roadmanager::LaneSection *lane_section = road->GetLaneSectionByIdx(0);

		double average_distance = 100.0 / density;

		if (road->GetLength() > ROAD_MIN_LENGTH)
		{
			for (int l = 0; l < lane_section->GetNumberOfLanes(); l++)

			{
				int lane_id = lane_section->GetLaneIdByIdx(l);
				roadmanager::Lane *lane = lane_section->GetLaneById(lane_id);

				if (lane->IsDriving())
				{
					for (double s = 0; s < road->GetLength() - average_distance;)
					{
						int carModelID;

						// Higher speeds in lanes closer to reference lane
						double lane_speed = speed * (0.9 + 0.7*(1.0 / abs(lane_id)));

						// left lanes reverse direction
						double s_aligned = lane->GetId() > 0 ? road->GetLength() - s : s;

						// randomly choose model
						carModelID = (double(viewer->carModels_.size()) * mt_rand()) / (std::mt19937::max)();

						Car *car_ = new Car;
						car_->road_id_init = odrManager->GetRoadByIdx(r)->GetId();
						car_->lane_id_init = lane_id;						
						car_->pos = new roadmanager::Position(odrManager->GetRoadByIdx(r)->GetId(), lane_id, s_aligned, 0);
						car_->model = viewer->AddCar(carModelID);
						car_->speed = lane_speed;
						car_->id = cars.size();
						car_->ego = 0;
						cars.push_back(car_);

//						printf("Car %d rid %d lid %d\n", cars.size() - 1, car_->road_id_init, car_->lane_id_init);

						// Add space to next vehicle
						s += average_distance + (0.2 * average_distance * mt_rand()) / (std::mt19937::max)();
					}
				}
			}
		}
	}

	return 0;
}

void updateCar(roadmanager::OpenDrive *odrManager, Car *car, double deltaSimTime)
{
	double ds = -SIGN(car->pos->GetLaneId()) * car->speed * deltaSimTime; // right lane is < 0 in road dir;

	if (car->pos->MoveAlongS(ds) != 0)
	{
		// invalid move -> reset position
		double s;
		if (car->lane_id_init > 0)
		{
			s = odrManager->GetRoadById(car->road_id_init)->GetLength();
		}
		else
		{
			s = 0;
		}
//		printf("Reset pos rid: %d lid: %d\n", car->road_id_init, car->lane_id_init);
		car->pos->SetLanePos(car->road_id_init, car->lane_id_init, s, 0, 0);
	}

	if (car->model->txNode_ != 0)
	{
		double heading = car->pos->GetH();
		double pitch = car->pos->GetP();

		// Is the car going opposite direction?
		if (car->pos->GetLaneId() > 0)
		{
			// Add 180 degrees
			heading += M_PI;
			if (heading > 2 * M_PI)
			{
				heading -= 2 * M_PI;
			}
			pitch = -pitch;
		}
		car->model->txNode_->setPosition(osg::Vec3(car->pos->GetX(), car->pos->GetY(), car->pos->GetZ()));

		car->model->quat_.makeRotate(
			car->pos->GetR(), osg::Vec3(1, 0, 0),
			pitch, osg::Vec3(0, 1, 0),
			heading, osg::Vec3(0, 0, 1));

		car->model->txNode_->setAttitude(car->model->quat_);
	}
}

int main(int argc, char** argv)
{
	mt_rand.seed(time(0));

	// use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);	

    arguments.getApplicationUsage()->setApplicationName(arguments.getApplicationName());
    arguments.getApplicationUsage()->setDescription(arguments.getApplicationName());
	arguments.getApplicationUsage()->setCommandLineUsage(arguments.getApplicationName() + " [options]\n");
	arguments.getApplicationUsage()->addCommandLineOption("--odr <filename>", "OpenDRIVE filename");
	arguments.getApplicationUsage()->addCommandLineOption("--model <filename>", "3D model filename");
	arguments.getApplicationUsage()->addCommandLineOption("--density <number>", "density (cars / 100 m)", std::to_string(DEFAULT_DENSITY));
	arguments.getApplicationUsage()->addCommandLineOption("--speed <number>", "speed (km/h)", std::to_string(DEFAULT_SPEED));

	if (arguments.argc() < 2)
	{
		arguments.getApplicationUsage()->write(std::cout, 1, 120, true);
		return -1;
	}

	std::string odrFilename;
	arguments.read("--odr", odrFilename);

	std::string modelFilename;
	arguments.read("--model", modelFilename);

	arguments.read("--density", density);
	printf("density: %.2f\n", density);

	arguments.read("--speed", speed);
	printf("speed: %.2f\n", speed);
	speed /= 3.6;

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
		roadmanager::Position ego_route_pos;
		//waypoint[0].SetLanePos(2, -1, 200, 0);
		//waypoint[1].SetLanePos(1, -1, 10, 0);
		waypoint[0].SetLanePos(2, -1, 200, 0);
		waypoint[1].SetLanePos(3, -1, 10, 0);
		roadmanager::Route route;
		double route_s = 0;

		route.AddWaypoint(&waypoint[0]);
		route.AddWaypoint(&waypoint[1]);
#endif


		viewer::Viewer *viewer = new viewer::Viewer(
			odrManager, 
			modelFilename.c_str(),
			arguments);

		SetupCars(odrManager, viewer);
		printf("%d cars added\n", (int)cars.size());

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
				std::this_thread::sleep_for(std::chrono::milliseconds((int)(1000 * (minStepSize - deltaSimTime))));
				deltaSimTime = minStepSize;
			}

			for (auto &car : cars)
			{
				if(car->ego)
				{
					// Update vehicle dynamics/driver model
#if !USE_ROUTE
					car->ego->Update(deltaSimTime, viewer->driverAcceleration_, viewer->driverSteering_);
					car->pos->SetXYH(car->ego->posX_, car->ego->posY_, car->ego->heading_);
#else
					route_s += deltaSimTime * 50 / 3.6; // 50 km/h
					route.SetOffset(route_s, 0, 0);
					route.GetPosition(car->pos);
					car->ego->SetPos(car->pos->GetX(), car->pos->GetY(), car->pos->GetZ(), car->pos->GetH());
#endif
					
					// Fetch Z and Pitch from OpenDRIVE position
					car->ego->posZ_ = car->pos->GetZ();
					car->ego->pitch_ = car->pos->GetP();

					// update 3D model transform
					car->model->txNode_->setPosition(osg::Vec3(car->ego->posX_, car->ego->posY_, car->ego->posZ_));
					car->model->quat_.makeRotate(
						0, osg::Vec3(1, 0, 0), // Roll
						0, osg::Vec3(0, 1, 0), // Pitch
						car->ego->heading_, osg::Vec3(0, 0, 1)); // Heading
					car->model->txNode_->setAttitude(car->model->quat_);

					// Update wheel angles
					osg::Quat quat;
					quat.makeRotate(
						0, osg::Vec3(1, 0, 0), // Roll
						car->ego->wheelRotation_, osg::Vec3(0, 1, 0), // Pitch
						car->ego->wheelAngle_, osg::Vec3(0, 0, 1)); // Heading
					car->model->wheel_[0]->setAttitude(quat);
					car->model->wheel_[1]->setAttitude(quat);
					quat.makeRotate(
						0, osg::Vec3(1, 0, 0), // Roll
						car->ego->wheelRotation_, osg::Vec3(0, 1, 0), // Pitch
						0, osg::Vec3(0, 0, 1)); // Heading
					car->model->wheel_[2]->setAttitude(quat);
					car->model->wheel_[3]->setAttitude(quat);

					track_pos->SetTrackPos(car->pos->GetTrackId(), car->pos->GetS(), 0);
					lane_pos->SetLanePos(car->pos->GetTrackId(), car->pos->GetLaneId(), car->pos->GetS(), 0);
					//printf("Ego pos: track %d lane %d s %.2f t %.2f offset %.2f\n", car->pos->GetTrackId(), car->pos->GetLaneId(), car->pos->GetS(), car->pos->GetT(), car->pos->GetOffset());
					viewer->UpdateVPoints(track_pos->GetX(), track_pos->GetY(), lane_pos->GetX(), lane_pos->GetY(), lane_pos->GetZ());
					viewer->UpdateVLine(lane_pos->GetX(), lane_pos->GetY(), lane_pos->GetZ());
				}
				else
				{
					updateCar(odrManager, car, deltaSimTime);
				}
			}

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

	for (auto &car : cars)
	{
		delete(car);
	}

	delete track_pos;
	delete lane_pos;

	return 0;
}
