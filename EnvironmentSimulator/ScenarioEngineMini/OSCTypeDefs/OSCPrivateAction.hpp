#pragma once
#include "OSCPosition.hpp"
#include "CommonMini.hpp"
#include "OSCAction.hpp"
#include "Entities.hpp"

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
		virtual double GetValue() = 0;
	};

	class TargetAbsolute : public Target
	{
	public:
		TargetAbsolute() : Target(Type::RELATIVE) {}

		double GetValue()
		{
			return value_;
		}
	};

	class TargetRelative : public Target
	{
	public:

		typedef enum
		{
			DELTA,
			FACTOR
		} ValueType;

		Object *object_;
		ValueType valueType_;
		bool continuous_;

		TargetRelative() : Target(Type::RELATIVE) {}

		double GetValue() { return 0; }
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
	
	void Step(double dt, Object *object)
	{
		object->speed_ = target_->GetValue();
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
		Object *object_;

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
	
	void Step(double dt, Object *object)
	{
		LOG("Step %s", object->name_.c_str());
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
		Object *object_;

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

	void Step(double dt, Object *object)
	{
		LOG("Step %s", object->name_.c_str());
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
	roadmanager::Position *target_position_;
	double time_to_destination_;

	MeetingAbsoluteAction() : OSCPrivateAction(OSCPrivateAction::Type::MEETING_ABSOLUTE) {}

	void Step(double dt, Object *object)
	{
		LOG("Step %s", object->name_.c_str());
	}
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
	roadmanager::Position *target_position_;
	Object *object_;
	roadmanager::Position *object_target_position_;
	double offsetTime_;
	bool continuous_;

	MeetingRelativeAction() : OSCPrivateAction(OSCPrivateAction::Type::MEETING_RELATIVE) {}

	void Step(double dt, Object *object)
	{
		LOG("Step %s", object->name_.c_str());
	}
};

class PositionAction : public OSCPrivateAction
{
public:
	roadmanager::Position position_;

	PositionAction() : OSCPrivateAction(OSCPrivateAction::Type::POSITION) {}

	void Step(double dt, Object *object)
	{
		object->pos_ = position_;
		LOG("Step %s pos: ", object->name_.c_str());
		position_.Print();

		active_ = false;
	}
};
