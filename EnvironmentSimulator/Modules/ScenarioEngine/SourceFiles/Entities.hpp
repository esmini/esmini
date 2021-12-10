/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "OSCBoundingBox.hpp"
#include "OSCProperties.hpp"//todo
#include <algorithm>


namespace scenarioengine
{
	class Controller;  // Forward declaration
	class OSCPrivateAction;
	class Event;

	class Object
	{
	public:
		typedef enum
		{
			TYPE_NONE = 0,
			VEHICLE = 1,
			PEDESTRIAN = 2,
			MISC_OBJECT = 3
		} Type;

		typedef enum
		{
			NONE = 0,
			LONGITUDINAL = (1 << 0),
			LATERAL = (1 << 1),
			SPEED = (1 << 2),
			WHEEL_ANGLE = (1 << 3),
			WHEEL_ROTATION = (1 << 4),
			VISIBILITY = (1 << 5),
			VELOCITY = (1 << 6),
			ANGULAR_RATE = (1 << 7),
			ACCELERATION = (1 << 8),
			ANGULAR_ACC = (1 << 9),
		} DirtyBit;

		typedef enum
		{
			GRAPHICS = (1 << 0),
			TRAFFIC = (1 << 1),
			SENSORS = (1 << 2),
		} Visibility;

		OSCProperties properties_;

		typedef enum
		{
			OVERRIDE_THROTTLE = 0,		 // Value range: [0..1]. 0 represents 0%, 1 represents 100% of pressing the throttle pedal.
			OVERRIDE_BRAKE = 1,			 // Value range: [0..1]. 0 represents 0%, 1 represents 100% of pressing the brake pedal.
			OVERRIDE_CLUTCH = 2,		 // Value range: [0..1]. 0 represents 0%, 1 represents 100% of pressing the clutch pedal.
			OVERRIDE_PARKING_BRAKE = 3,	 // Value range: [0..1]. 0 represents 0%, The value 1 represent the maximum parking brake state.
			OVERRIDE_STEERING_WHEEL = 4, // Steering wheel angle. Unit: rad. (0: Neutral position, positive: Left, negative: Right)
			OVERRIDE_GEAR = 5,			 // Gear number. (-1:Reverse, 0:Neutral, 1:Gear 1, 2:Gear 2, and so on.)
			OVERRIDE_NR_TYPES = 6,
			OVERRIDE_UNDEFINED = 7
		} OverrideType;

		typedef struct
		{
			OverrideType type;
			bool active;  // True: override; false: stop overriding
			double value; // Depends on action, see SE_OverrideActionList
		} OverrideActionStatus;


		typedef struct
		{
			double maxAcceleration;
			double maxDeceleration;
			double maxSpeed;
		} Performance;

		// Allocate vector for all possible override status
		OverrideActionStatus overrideActionList[OVERRIDE_NR_TYPES];

		Type type_;
		int category_; // specific object category in vehicle, pedestrian or misobject
		std::string typeName_;  // Name of the vehicle-, pedestrian- or misc object type
		std::string name_;
		std::string model3d_;
		int id_;
		EntityScaleMode scaleMode_;

		// Ghost following stuff
		int trail_follow_index_; // Index of closest segment
		int ghost_trail_s_;		 // closest point on ghost trail
		roadmanager::TrajVertex trail_closest_pos_;

		double sensor_pos_[3];
		Object* ghost_;
		Object* ghost_Ego_;

		double speed_;
		double wheel_angle_;
		double wheel_rot_;
		roadmanager::Position pos_;
		int model_id_;
		roadmanager::PolyLineBase trail_;
		double odometer_;
		OSCBoundingBox boundingbox_;
		double end_of_road_timestamp_;
		double off_road_timestamp_;
		double stand_still_timestamp_;
		double headstart_time_;
		int visibilityMask_;
		roadmanager::Junction::JunctionStrategyType junctionSelectorStrategy_;
		double nextJunctionSelectorAngle_;  // number between 0:2pi (circle). E.g. if 1.57 choose the leftmost road
		Performance performance_;

		int dirty_;
		bool reset_;			 // indicate discreet movement, teleporting, no odometer update
		Controller *controller_; // reference to any assigned controller object
		bool isGhost_;

		//Rel2abs Controller addition
		std::vector<Event*> objectEvents_;				//Events that contains privateactions applied to this object
		std::vector<OSCPrivateAction*> initActions_;	//initActions that is being or has been applied to this object

		struct {
			double pos_x;
			double pos_y;
			double vel_x;
			double vel_y;
			double h;
			double h_rate;
		} state_old;

		Object(Type type);
		~Object() {}
		void SetEndOfRoad(bool state, double time = 0.0);
		bool IsEndOfRoad() { return end_of_road_timestamp_ > SMALL_NUMBER; }
		double GetEndOfRoadTimestamp() { return end_of_road_timestamp_; }
		void SetOffRoad(bool state, double time = 0.0);
		bool IsOffRoad() { return off_road_timestamp_ > SMALL_NUMBER; }
		double GetOffRoadTimestamp() { return off_road_timestamp_; }
		void SetStandStill(bool state, double time = 0.0);
		bool IsStandStill() { return stand_still_timestamp_ > SMALL_NUMBER; }

		/**
			Move current position along the road or route (if assigned)
			@param ds Distance to move, negative will move backwards
			@param actualDistance if true ds will adjusted for curvature and lat offset
			@return Non zero return value indicates error of some kind
		*/
		int MoveAlongS(double ds, bool actualDistance = true);

		/**
		    Returns the timestamp from which the entity has not moved.
			@return The timestamp in seconds.
		*/
		double GetStandStillTimestamp() { return stand_still_timestamp_; }

		/**
			Check if object is colliding/overlapping with specified target object
			Definition of collision is overlapping bounding boxes
			@param target The object to check
			@return true if bounding boxes overlap else false
		*/
		bool CollisionAndRelativeDistLatLong(Object* target, double* distLat, double* distLong);

		bool Collision(Object* target) { return CollisionAndRelativeDistLatLong(target, nullptr, nullptr); }

		/**
			Check if point is colliding/overlapping with specified target object
			@param x X coordinate of target point
			@param y Y coordinate of target point
			@return true if bounding boxes overlap else false
		*/
		double PointCollision(double x, double y);

		/**
			Measure the free-space distance to provided target object
			based on closest distance between the bounding boxes
			@param target The object to check
			@param latDist Returns lateral distance to target object
			@param longDist Returns longitudinal distance to target object
			@return distance The free-space Euclidean distance between objects (0 if collision)
		*/
		double FreeSpaceDistance(Object *target, double *latDist, double *longDist);

		/**
			Measure the free-space distance to provided target 2D position
			based on closest point on object's bounding box
			@param x X coordinate of target point
			@param y Y coordinate of target point
			@param latDist Returns lateral distance
			@param longDist Returns longitudinal distance
			@return distance The free-space Euclidean distance between objects (0 if collision)
		*/
		double FreeSpaceDistancePoint(double x, double y, double *latDist, double *longDist);

		int FreeSpaceDistancePointRoadLane(double x, double y, double *latDist, double *longDist, roadmanager::CoordinateSystem cs);
		int FreeSpaceDistanceObjectRoadLane(Object *target, double *latDist, double *longDist, roadmanager::CoordinateSystem cs);

		/**
		Measure the distance to provided target object
		@param target The object to check
		@param cs CoordinateSystem, see roadmanager::CoordinateSystem
		@param relDistType, see roadmanager::RelativeDistanceType
		@param freeSpace, measure free distance between bounding boxes or just refpoint to refpoint
		@param dist Distance (output parameter)
		@return 0 if position found and parameter values are valid, else -1
		*/
		int Distance(Object *target, roadmanager::CoordinateSystem cs, roadmanager::RelativeDistanceType relDistType, bool freeSpace, double &dist, double maxDist = LARGE_NUMBER);

		/**
		Measure the distance to provided target world x, y position
		@param x X coordinate of target world position
		@param y Y coordinate of target world position
		@param cs CoordinateSystem, see roadmanager::CoordinateSystem
		@param relDistType, see roadmanager::RelativeDistanceType
		@param freeSpace, measure free distance between bounding boxes or just refpoint to refpoint
		@param dist Distance (output parameter)
		@return 0 if position found and parameter values are valid, else -1
		*/
		int Distance(double x, double y, roadmanager::CoordinateSystem cs, roadmanager::RelativeDistanceType relDistType, bool freeSpace, double &dist, double maxDist = LARGE_NUMBER);

		void SetSpeed(double speed)
		{
			speed_ = speed;
			SetDirtyBits(Object::DirtyBit::SPEED);
		}
		double GetSpeed() { return speed_; }
		void SetAssignedController(Controller *controller)
		{
			controller_ = controller;
		}
		int GetAssignedControllerType();
		int GetActivatedControllerType();
		bool IsControllerActiveOnDomains(ControlDomains domainMask);
		bool IsControllerActiveOnAnyOfDomains(ControlDomains domainMask);
		bool IsControllerActive();
		int GetControllerMode();
		int GetId() { return id_; }
		void SetHeadstartTime(double headstartTime) { headstart_time_ = headstartTime; }
		double GetHeadstartTime() { return headstart_time_; }
		void SetGhost(Object *ghost) { ghost_ = ghost; }
		Object *GetGhost() { return ghost_; }
		void SetVisibilityMask(int mask);
		bool IsGhost() { return isGhost_; }
		void SetVel(double x_vel, double y_vel, double z_vel);
		void SetAcc(double x_acc, double y_acc, double z_acc);
		void SetAngularVel(double h_vel, double p_vel, double r_vel);
		void SetAngularAcc(double h_acc, double p_acc, double r_acc);

		void SetMaxAcceleration(double maxAcceleration) { performance_.maxAcceleration = maxAcceleration; }
		double GetMaxAcceleration() { return performance_.maxAcceleration; }
		void SetMaxDeceleration(double maxDeceleration) { performance_.maxDeceleration = maxDeceleration; }
		double GetMaxDeceleration() { return performance_.maxDeceleration; }
		void SetMaxSpeed(double maxSpeed) { performance_.maxSpeed = maxSpeed; }
		double GetMaxSpeed() { return performance_.maxSpeed; }
		std::string GetName() { return name_; }
		std::string GetTypeName() { return typeName_; }
		std::string GetModelFileName() { return FileNameOf(model3d_); }
		std::string GetModelFilePath() { return model3d_; }

		/**
		Specify strategy how to choose way in next junction
		@param type Use specified angle (SetJunctionSelectorAngle*) or randomize. See roadmanager::Junction::JunctionStrategyType.
		*/
		void SetJunctionSelectorStrategy(roadmanager::Junction::JunctionStrategyType type) { junctionSelectorStrategy_ = type; }

		/**
		Get current strategy how to choose way in next junction
		@return JunctionStrategyType: Use specified angle (SetJunctionSelectorAngle*) or randomize. See roadmanager::Junction::JunctionStrategyType.
		*/
		roadmanager::Junction::JunctionStrategyType GetJunctionSelectorStrategy() { return junctionSelectorStrategy_; }

		/**
		Specify how to choose way in next junction
		@param angle Specify desired direction [0:2pi] from incoming road direction (angle = 0)
		*/
		void SetJunctionSelectorAngle(double angle);

		/**
		Choose a random angle for junction selector, in effect random choice in next junction
		*/
		void SetJunctionSelectorAngleRandom();

		double GetJunctionSelectorAngle() { return nextJunctionSelectorAngle_; }

		//Rel2abs Controller addition
		void addEvent(Event* event) { objectEvents_.push_back(event); }
		void removeEvent(Event* event);
		bool containsEvent(Event* event) { return (std::find(objectEvents_.begin(), objectEvents_.end(), event) != objectEvents_.end()); }
		//Will not get completed actions - only running/standby
		std::vector<OSCPrivateAction*> getPrivateActions();
		std::vector<Event*> getEvents() { return objectEvents_; }

		bool CheckDirtyBits(int bits)
		{
			return bool(dirty_ & bits);
		}

		void SetDirtyBits(int bits)
		{
			dirty_ |= bits;
		}

		void ClearDirtyBits(int bits)
		{
			dirty_ &= ~bits;
		}

		void ClearDirtyBits()
		{
			dirty_ = 0;
		}
	};

	class Vehicle : public Object
	{
	public:
		typedef enum
		{
			CAR = 0,
			VAN = 1,
			TRUCK = 2,
			SEMITRAILER = 3,
			BUS = 4,
			MOTORBIKE = 5,
			BICYCLE = 6,
			TRAIN = 7,
			TRAM = 8
		} Category;

		Vehicle() : Object(Object::Type::VEHICLE)
		{
			category_ = static_cast<int>(Category::CAR);
			performance_.maxAcceleration = 10.0;
			performance_.maxDeceleration= 10.0;
			performance_.maxSpeed = 100.0;
		}

		void SetCategory(std::string category)
		{
			if (category == "car")
			{
				category_ = static_cast<int>(Vehicle::Category::CAR);
			}
			else if (category == "truck")
			{
				category_ = static_cast<int>(Vehicle::Category::TRUCK);
			}
			else if (category == "bus")
			{
				category_ = static_cast<int>(Vehicle::Category::BUS);
			}
			else if (category == "bicycle")
			{
				category_ = static_cast<int>(Vehicle::Category::BICYCLE);
			}
			else if (category == "motorbike")
			{
				category_ = static_cast<int>(Vehicle::Category::MOTORBIKE);
			}
			else
			{
				LOG("Vehicle category %s not supported yet", category.c_str());
			}

			return;
		}
	};

	class Pedestrian : public Object
	{
	public:
		typedef enum
		{
			PEDESTRIAN = 0,
			WHEELCHAIR = 1,
			ANIMAL = 2
		} Category;

		double mass_;		/**< The mass of a pedestrian in kg. */

		Pedestrian() : Object(Object::Type::PEDESTRIAN), mass_(0.0)
		{
			category_ = static_cast<int>(Category::PEDESTRIAN);
			performance_.maxAcceleration = 10.0;
			performance_.maxDeceleration = 10.0;
			performance_.maxSpeed = 10.0;
		}

		void SetCategory(std::string category)
		{
			if (category == "pedestrian")
			{
				category_ = static_cast<int>(Pedestrian::Category::PEDESTRIAN);
			}
			else if (category == "wheelchair")
			{
				category_ = static_cast<int>(Pedestrian::Category::WHEELCHAIR);
			}
			else if (category == "animal")
			{
				category_ = static_cast<int>(Pedestrian::Category::ANIMAL);
			}
			else
			{
				LOG("Pedestrian category %s not supported yet", category.c_str());
			}

			return;
		}
	};

	class MiscObject : public Object
	{
	public:
		typedef enum
		{
			NONE = 0,
			OBSTACLE = 1,
			POLE = 2,
			TREE = 3,
			VEGETATION = 4,
			BARRIER = 5,
			BUILDING = 6,
			PARKINGSPACE = 7,
			PATCH = 8,
			RAILING = 9,
			TRAFFICISLAND = 10,
			CROSSWALK = 11,
			STREETLAMP = 12,
			GANTRY = 13,
			SOUNDBARRIER = 14,
			WIND = 15,
			ROADMARK = 16
		} Category;

		double mass_;
		std::string name_;

		MiscObject() : Object(Object::Type::MISC_OBJECT), mass_(0.0), name_("")
		{
			category_ = static_cast<int>(category_);
			performance_.maxAcceleration = 0.0;
			performance_.maxDeceleration = 0.0;
			performance_.maxSpeed = 0.0;
		}

		void SetCategory(std::string category)
		{
			if (category == "none")
			{
				category_ = static_cast<int>(MiscObject::Category::NONE);
			}
			else if (category == "obstacle")
			{
				category_ = static_cast<int>(MiscObject::Category::OBSTACLE);
			}
			else if (category == "pole")
			{
				category_ = static_cast<int>(MiscObject::Category::POLE);
			}
			else if (category == "tree")
			{
				category_ = static_cast<int>(MiscObject::Category::TREE);
			}
			else if (category == "vegetation")
			{
				category_ = static_cast<int>(MiscObject::Category::VEGETATION);
			}
			else if (category == "barrier")
			{
				category_ = static_cast<int>(MiscObject::Category::BARRIER);
			}
			else if (category == "building")
			{
				category_ = static_cast<int>(MiscObject::Category::BUILDING);
			}
			else if (category == "parkingSpace")
			{
				category_ = static_cast<int>(MiscObject::Category::PARKINGSPACE);
			}
			else if (category == "patch")
			{
				category_ = static_cast<int>(MiscObject::Category::PATCH);
			}
			else if (category == "railing")
			{
				category_ = static_cast<int>(MiscObject::Category::RAILING);
			}
			else if (category == "trafficIsland")
			{
				category_ = static_cast<int>(MiscObject::Category::TRAFFICISLAND);
			}
			else if (category == "crosswalk")
			{
				category_ = static_cast<int>(MiscObject::Category::CROSSWALK);
			}
			else if (category == "streetLamp")
			{
				category_ = static_cast<int>(MiscObject::Category::STREETLAMP);
			}
			else if (category == "gantry")
			{
				category_ = static_cast<int>(MiscObject::Category::GANTRY);
			}
			else if (category == "soundBarrier")
			{
				category_ = static_cast<int>(MiscObject::Category::SOUNDBARRIER);
			}
			else if (category == "wind")
			{
				category_ = static_cast<int>(MiscObject::Category::WIND);
			}
			else if (category == "roadMark")
			{
				category_ = static_cast<int>(MiscObject::Category::ROADMARK);
			}
			else
			{
				LOG("MiscObject category %s not supported yet", category.c_str());
			}

			return;
		}
	};

	class Entities
	{
	public:
		Entities() : nextId_(0) {}

		std::vector<Object*> object_;

		// create a sumo vehicle template and a sumo controller
		int addObject(Object* obj);
		void removeObject(int id);
		void removeObject(std::string name);
		int getNewId();
		bool indexExists(int id);
		bool nameExists(std::string name);
		Object* GetObjectByName(std::string name);
		Object* GetObjectById(int id);

	private:
		int nextId_;  // Is incremented for each new object created
	};

}
