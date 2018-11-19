#pragma once
//#include "OSCPosition.hpp"
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
		FOLLOW_ROUTE
	} Type;

	typedef enum
	{
		LINEAR,
		CUBIC,
		SINUSOIDAL,
		STEP,
		UNDEFINED
	} DynamicsShape;

	class TransitionDynamics
	{
	public:
		DynamicsShape shape_;

		double Evaluate(double factor, double start_value, double end_value);  // 0 = start_value, 1 = end_value

		TransitionDynamics() : shape_(DynamicsShape::STEP) {}
	};

	Type type_;
	Object *object_;

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

	typedef enum
	{
		RATE,
		TIME,
		DISTANCE
	} Timing;

	struct
	{
		Timing timing_type_;
		double timing_target_value_;
		TransitionDynamics transition_;
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

	Target *target_;
	double start_speed_;
	double elapsed_;

	LongSpeedAction() : OSCPrivateAction(OSCPrivateAction::Type::LONG_SPEED), target_(0) 
	{
		dynamics_.timing_type_ = Timing::TIME;  // Make default
		dynamics_.timing_target_value_ = 0.0;
		elapsed_ = 0;
	}
	
	void Trig()
	{
		OSCAction::Trig();
	}

	void Step(double dt);

	void print()
	{
		LOG("");
	}
};

class LatLaneChangeAction: public OSCPrivateAction
{
public:
	typedef enum
	{
		TIME,
		DISTANCE
	} Timing;

	struct
	{
		Timing timing_type_;
		double timing_target_value_;
		TransitionDynamics transition_; 
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
		int value_;

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
	double start_t_;
	double target_lane_offset_;
	int target_lane_id_;
	double elapsed_;

	LatLaneChangeAction() : OSCPrivateAction(OSCPrivateAction::Type::LAT_LANE_CHANGE)
	{
		dynamics_.timing_type_ = Timing::TIME;  // Make default
		dynamics_.timing_target_value_ = 0.0; 
		elapsed_ = 0;
	}
	
	void Step(double dt);

	void Trig();

};

class LatLaneOffsetAction : public OSCPrivateAction
{
public:
	struct
	{
		double max_lateral_acc_;
		double duration_;
		TransitionDynamics transition_;
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
	double elapsed_;
	double start_lane_offset_;

	LatLaneOffsetAction() : OSCPrivateAction(OSCPrivateAction::Type::LAT_LANE_OFFSET)
	{
		LOG("");
		dynamics_.max_lateral_acc_ = 0;
		dynamics_.duration_ = 0;
		elapsed_ = 0;
	}

	void Trig();
	void Step(double dt);
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

	void Step(double dt)
	{
		LOG("Step %s", object_->name_.c_str());
	}

	void Trig()
	{
		OSCAction::Trig();
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

	void Step(double dt)
	{
//		LOG("Step %s", object_->name_.c_str());
	}

	void Trig()
	{
		OSCAction::Trig();
	}
};

class PositionAction : public OSCPrivateAction
{
public:
	roadmanager::Position position_;

	PositionAction() : OSCPrivateAction(OSCPrivateAction::Type::POSITION) {}

	void Step(double dt)
	{
		object_->pos_ = position_;
		LOG("Step %s pos: ", object_->name_.c_str());
		position_.Print();

		state_ = State::DONE;
	}

	void Trig()
	{
		OSCAction::Trig();
	}
};

class FollowRouteAction : public OSCPrivateAction
{
public:
	roadmanager::Route *route_;

	FollowRouteAction() : OSCPrivateAction(OSCPrivateAction::Type::FOLLOW_ROUTE) {}

	void Step(double dt)
	{
		LOG("Step %s ", object_->name_.c_str());

//		state_ = State::DONE;
	}

	void Trig()
	{
		object_->pos_.SetRoute(route_);
		OSCAction::Trig();
	}
};
