#pragma once
#include "OSCPosition.hpp"

#include <iostream>
#include <string>
#include <math.h>

class OSCPrivateAction
{
public:
	bool exists;

	typedef struct
	{
		bool exists;
		std::string shape; // Wrong type
		double rate;
		double time;
		double distance;
	} SpeedDynamics;

	typedef struct
	{
		std::string object;
		double value;
		std::string valueType; // Wrong type
		bool continuous;
	} SpeedTargetRelative;

	typedef struct
	{
		double value;
	} SpeedTargetAbsolute;

	typedef struct
	{
		SpeedTargetRelative *relative_;
		SpeedTargetAbsolute *absolute_;
	} SpeedTarget;

	typedef struct
	{
		SpeedDynamics *dynamics_;
		SpeedTarget *target_;	
	} Speed;

	typedef struct
	{
		std::string object;
		double value;
	} LaneRelative;

	typedef struct
	{
		double value;
	} LaneAbsolute;

	typedef struct
	{
		double targetLaneOffset;

		struct
		{
			double time;
			double distance;
			std::string shape; // Wrong type
		} Dynamics;

		struct
		{
			LaneRelative *relative_;
			LaneAbsolute *absolute_;
		} Target;

	} LaneChange;

	typedef struct
	{
		struct
		{
			double maxLateralAcc;
			double duration;
			std::string shape; // Wrong type
		} Dynamics;

		struct
		{
			LaneRelative *relative_;
			LaneAbsolute *absolute_;
		} Target;

	} LaneOffset;

	typedef struct
	{
		std::string mode;
		std::string object;
		double offsetTime;
		std::string continuous;			//Wrong type
		OSCPosition Position;
	} MeetingRelative;

	typedef struct
	{
		std::string mode;
		OSCPosition Position;
		MeetingRelative *relative_;
	} Meeting;

	typedef struct {
		struct {
			OSCRoute Route;
			OSCCatalogReference CatalogReference;
		} FollowRoute;
	} Routing;

	LaneChange *laneChange_;
	LaneOffset *laneOffset_;
	Speed *speed_;
	Meeting *meeting_;
	Routing *routing_;
	OSCPosition *position_;

	struct {} Visibility;
	struct {} Autonomous;
	struct {} Controller;
	
	OSCPrivateAction() 
	{
		laneChange_ = 0;
		laneOffset_ = 0;
		speed_ = 0;
		meeting_ = 0;
		routing_ = 0;
		position_ = 0;
	}

	void printOSCPrivateAction()
	{
		if (laneChange_)
		{
			std::cout << "\t" << " - Lateral - Lane Change" << std::endl;
		}
		if (laneOffset_)
		{
			std::cout << "\t" << " - Lateral - Lane Offset" << std::endl;
		}
		if (speed_)
		{
			std::cout << "\t" << " - Longitudinal - speed" << std::endl;
		}
		if (meeting_)
		{
			std::cout << "\t" << " - Longitidubgal - meeting" << std::endl;
		}

		std::cout << "\t" << " - Position" << std::endl;
		if (position_)
		{
			position_->printOSCPosition();
		}
		std::cout << std::endl;
	};

};