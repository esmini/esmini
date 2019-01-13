#pragma once

#include "OSCAction.hpp"
#include "Entities.hpp"
#include "CommonMini.hpp"

#include <iostream>
#include <string>
#include <math.h>

namespace scenarioengine
{

	#define DISTANCE_TOLERANCE (0.5)  // meter
	#define IS_ZERO(x) (x < SMALL_NUMBER && x > -SMALL_NUMBER)

	class OSCPrivateAction : public OSCAction
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

		void print()
		{
			LOG("Virtual, should be overridden");
		};

	};

	class LongSpeedAction : public OSCPrivateAction
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
			TargetAbsolute() : Target(Type::ABSOLUTE) {}

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
			ValueType value_type_;
			bool continuous_;

			TargetRelative() : Target(Type::RELATIVE), consumed_(false), object_speed_(0), continuous_(false) {}

			double GetValue();

		private:
			double object_speed_;
			bool consumed_;
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

		void Trig();

		void Step(double dt);

		void print()
		{
			LOG("");
		}
	};

	class LongDistanceAction : public OSCPrivateAction
	{
	public:

		struct
		{
			bool none_;
			double max_acceleration_;
			double max_deceleration_;
			double max_speed_;
		} dynamics_;

		typedef enum
		{
			DISTANCE,
			TIME_GAP,
		} DistType;

		Object *target_object_;
		double distance_;
		DistType dist_type_;
		double freespace_;

		LongDistanceAction() : OSCPrivateAction(OSCPrivateAction::Type::LONG_DISTANCE), target_object_(0), distance_(0), dist_type_(DistType::DISTANCE), freespace_(0), acceleration_(0)
		{
		}

		void Trig();

		void Step(double dt);

		void print()
		{
			LOG("");
		}

	private:
		double acceleration_;
	};

	class LatLaneChangeAction : public OSCPrivateAction
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
			if (object_->extern_control_)
			{
				// motion control handed over 
				return;
			}

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
		roadmanager::Position *own_target_position_;

		Object *relative_object_;
		roadmanager::Position *relative_target_position_;

		double offsetTime_;
		bool continuous_;

		MeetingRelativeAction() : OSCPrivateAction(OSCPrivateAction::Type::MEETING_RELATIVE) {}

		void Step(double dt);

		void Trig()
		{
			if (object_->extern_control_)
			{
				// motion control handed over 
				return;
			}

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

			OSCAction::Stop();
		}

		void Trig()
		{
			// Allow even when externally control
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
		}

		void Trig()
		{
			if (object_->extern_control_)
			{
				// motion control handed over 
				return;
			}

			object_->pos_.SetRoute(route_);
			OSCAction::Trig();
		}
	};

}