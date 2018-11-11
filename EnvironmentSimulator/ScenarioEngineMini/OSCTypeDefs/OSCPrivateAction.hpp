#pragma once
#include "OSCPosition.hpp"
#include "CommonMini.hpp"

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
			LOG("\t - Lateral - Lane Change");
		}
		if (laneOffset_)
		{
			if (laneOffset_->Target.absolute_)
			{
				LOG("\t - Lateral - Lane Offset absolute = %.2f", laneOffset_->Target.absolute_->value);
			}
			else
			{
				LOG("\t - Lateral - Lane Offset relative %s = %.2f", 
					laneOffset_->Target.relative_->object.c_str(), laneOffset_->Target.relative_->value);
			}
			LOG("\t - Lateral - Lane Offset duration = %.2f shape = %s", 
				laneOffset_->Dynamics.duration, laneOffset_->Dynamics.shape.c_str());
		}
		if (speed_)
		{
			if (speed_->target_->absolute_)
			{
				LOG("\t - Longitudinal - speed: target absolute = %.2f dynamics.shape = %s dynamice.rate = %.2f",
					speed_->target_->absolute_->value, speed_->dynamics_->shape.c_str(), speed_->dynamics_->rate);
			}
			else
			{
				LOG("\t - Longitudinal - speed: target relative %s = %.2f dynamics.shape = %s dynamice.rate = %.2f",
					speed_->target_->relative_->object.c_str(), speed_->target_->relative_->value, 
					speed_->dynamics_->shape.c_str(), speed_->dynamics_->rate);
			}
		}
		if (meeting_)
		{
			LOG("\t - Longitidubgal - meeting");
		}

		if (position_)
		{
			position_->printOSCPosition();
		}
	};

};