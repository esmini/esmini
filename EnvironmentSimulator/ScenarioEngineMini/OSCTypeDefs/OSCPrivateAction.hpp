#pragma once
#include "OSCPosition.hpp"
#include "CommonMini.hpp"
#include "OSCAction.hpp"

#include <iostream>
#include <string>
#include <math.h>

class OSCPrivateAction: public OSCAction
{
public:
	typedef enum 
	{
		LONG_SPEED,
		LONG_DISTANCE,
		LAT_LANE_CHANGE,
		LAT_LANE_OFFSET,
		LAT_DISTANCE,
		VISIBILITY,
		MEETING_ABSOLUTE,
		MEETING_RELATIVE,
		AUTONOMOUS,
		CONTROLLER,
		POSITION,
		ROUTING
	} Type;

	typedef enum
	{
		LINEAR,
		CUBIC,
		SINUSOIDAL,
		STEP,
		UNDEFINED
	} DynamicsShape;

	Type type_;


	OSCPrivateAction(OSCPrivateAction::Type type) : OSCAction(OSCAction::BaseType::PRIVATE), type_(type)
	{
		LOG("");
	}

	virtual void Step(double dt)
	{
		LOG("Virtual method. Should be overridden!");
	}
#if 0
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

	Meeting *meeting_;
	Routing *routing_;
	OSCPosition *position_;

	struct {} Visibility;
	struct {} Autonomous;
	struct {} Controller;
	
	OSCPrivateAction() 
	{
		meeting_ = 0;
		routing_ = 0;
		position_ = 0;
	}
#endif


	void print()
	{
		LOG("Virtual, should be overridden");
	};

};

class LongSpeedAction: public OSCPrivateAction
{
public:

	class Target
	{
	public:
		typedef enum
		{
			ABSOLUTE,
			RELATIVE
		} Type;

		Type type_;
		double value_;

		Target(Type type) : type_(type) {}
	};

	class TargetAbsolute : public Target
	{
	public:
		TargetAbsolute() : Target(Type::RELATIVE) {}
	};

	class TargetRelative : public Target
	{
	public:

		typedef enum
		{
			DELTA,
			FACTOR
		} ValueType;

		std::string object_;
		ValueType valueType_;
		bool continuous_;

		TargetRelative() : Target(Type::RELATIVE) {}
	};

	struct
	{
		bool exists_;
		DynamicsShape shape_;
		double rate_;
		double time_;
		double distance_;
	} dynamics_;

	Target *target_;

	LongSpeedAction() : OSCPrivateAction(OSCPrivateAction::Type::LONG_SPEED), target_(0) 
	{
	}
	
	void Step(double dt)
	{
		LOG("Step");
	}

	void print()
	{
		LOG("");
	};
};

class LatLaneChangeAction: public OSCPrivateAction
{
public:
	struct
	{
		double time_;
		double distance_;
		DynamicsShape shape_; 
	} dynamics_;

	class Target
	{
	public:
		typedef enum
		{
			ABSOLUTE,
			RELATIVE
		} Type;

		Type type_;
		double value_;

		Target(Type type) : type_(type) {}
	};

	class TargetAbsolute : public Target
	{
	public:
		TargetAbsolute() : Target(Target::Type::ABSOLUTE) {}
	};

	class TargetRelative : public Target
	{
	public:
		std::string object_;

		TargetRelative() : Target(Target::Type::RELATIVE) {}
	};

	Target *target_;
	double target_lane_offset_;

	LatLaneChangeAction() : OSCPrivateAction(OSCPrivateAction::Type::LAT_LANE_CHANGE)
	{
		dynamics_.time_ = 0;
		dynamics_.distance_ = 0;
		dynamics_.shape_ = DynamicsShape::STEP;
	}
	
	void Step(double dt)
	{
		LOG("Step");
	}
};

class LatLaneOffsetAction : public OSCPrivateAction
{
public:
	struct
	{
		double max_lateral_acc_;
		double duration_;
		DynamicsShape shape_; 
	} dynamics_;

	class Target
	{
	public:
		typedef enum
		{
			ABSOLUTE,
			RELATIVE
		} Type;

		Type type_;
		double value_;

		Target(Type type) : type_(type) {}
	};

	class TargetAbsolute : public Target
	{
	public:
		TargetAbsolute() : Target(Target::Type::ABSOLUTE) {}
	};

	class TargetRelative : public Target
	{
	public:
		std::string object_;

		TargetRelative() : Target(Target::Type::RELATIVE) {}
	};


	Target *target_;

	LatLaneOffsetAction() : OSCPrivateAction(OSCPrivateAction::Type::LAT_LANE_OFFSET)
	{
		LOG("");
		dynamics_.max_lateral_acc_ = 0;
		dynamics_.duration_ = 0;
		dynamics_.shape_ = DynamicsShape::STEP;
	}

	void Step(double dt)
	{
		LOG("Step");
	}
};

class MeetingAbsoluteAction : public OSCPrivateAction
{
public:
	typedef enum
	{
		STRAIGHT,
		ROUTE
	} MeetingPositionMode;

	MeetingPositionMode mode_;
	OSCPosition *target_position_;
	double time_to_destination_;

	MeetingAbsoluteAction() : OSCPrivateAction(OSCPrivateAction::Type::MEETING_ABSOLUTE) {}

};

class MeetingRelativeAction : public OSCPrivateAction
{
public:
	typedef enum
	{
		STRAIGHT,
		ROUTE
	} MeetingPositionMode;

	MeetingPositionMode mode_;
	OSCPosition *target_position_;
	std::string object_;
	OSCPosition *object_target_position_;
	double offsetTime_;
	bool continuous_;

	MeetingRelativeAction() : OSCPrivateAction(OSCPrivateAction::Type::MEETING_RELATIVE) {}

};

class PositionAction : public OSCPrivateAction
{
public:
	OSCPosition *position_;

	PositionAction() : OSCPrivateAction(OSCPrivateAction::Type::POSITION) {}
};
