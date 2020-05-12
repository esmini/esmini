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

#include "OSCAction.hpp"
#include "OSCPosition.hpp"
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
			AUTONOMOUS,
			CONTROLLER,
			POSITION,
			FOLLOW_ROUTE,
			FOLLOW_TRAJECTORY,
			SYNCHRONIZE
		} Type;

		typedef enum
		{
			RATE,
			TIME,
			DISTANCE,
			DIMENSION_UNDEFINED
		} DynamicsDimension;

		typedef enum
		{
			LINEAR,
			CUBIC,
			SINUSOIDAL,
			STEP,
			SHAPE_UNDEFINED
		} DynamicsShape;

		class TransitionDynamics
		{
		public:
			DynamicsShape shape_;
			DynamicsDimension dimension_;
			double target_value_;

			double Evaluate(double factor, double start_value, double end_value);  // 0 = start_value, 1 = end_value

			TransitionDynamics() : shape_(DynamicsShape::STEP), dimension_(DynamicsDimension::TIME), target_value_(0) {}
		};

		Type type_;
		Object *object_;

		OSCPrivateAction(OSCPrivateAction::Type type) : OSCAction(OSCAction::BaseType::PRIVATE), type_(type) {}

		virtual void print()
		{
			LOG("Virtual, should be overridden");
		};

		virtual OSCPrivateAction* Copy()
		{
			LOG("Virtual, should be overridden");
			return 0;
		};

	};

	class LongSpeedAction : public OSCPrivateAction
	{
	public:

		TransitionDynamics transition_dynamics_;

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

			TargetRelative() : Target(Type::RELATIVE), continuous_(false), consumed_(false), object_speed_(0) {}

			double GetValue();

		private:
			bool consumed_;
			double object_speed_;
		};

		Target *target_;
		double start_speed_;
		double elapsed_;

		LongSpeedAction() : OSCPrivateAction(OSCPrivateAction::Type::LONG_SPEED), target_(0), start_speed_(0)
		{
			elapsed_ = 0;
		}

		LongSpeedAction(const LongSpeedAction &action) : OSCPrivateAction(OSCPrivateAction::Type::LONG_SPEED)
		{
			target_ = action.target_;
			transition_dynamics_ = action.transition_dynamics_;
			elapsed_ = action.elapsed_;
			start_speed_ = action.start_speed_;
		}

		OSCPrivateAction* Copy()
		{
			LongSpeedAction *new_action = new LongSpeedAction(*this);
			return new_action;
		}
		
		void Start();

		void Step(double dt, double simTime);

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

		LongDistanceAction(const LongDistanceAction &action) : OSCPrivateAction(OSCPrivateAction::Type::LONG_DISTANCE)
		{
			target_object_ = action.target_object_;
			dynamics_ = action.dynamics_;
			distance_ = action.distance_;
			dist_type_ = action.dist_type_;
			freespace_ = action.freespace_;
			acceleration_ = action.acceleration_;
		}

		OSCPrivateAction* Copy()
		{
			LongDistanceAction *new_action = new LongDistanceAction(*this);
			return new_action;
		}

		void Start();

		void Step(double dt, double simTime);

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
		TransitionDynamics transition_dynamics_;

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

			TargetRelative() : Target(Target::Type::RELATIVE), object_(0) {}
		};

		Target *target_;
		double start_t_;
		double target_lane_offset_;
		int target_lane_id_;
		double elapsed_;

		LatLaneChangeAction(LatLaneChangeAction::DynamicsDimension timing_type = DynamicsDimension::TIME) : OSCPrivateAction(OSCPrivateAction::Type::LAT_LANE_CHANGE)
		{
			transition_dynamics_.dimension_ = timing_type;
			elapsed_ = 0;
		}

		LatLaneChangeAction(const LatLaneChangeAction &action) : OSCPrivateAction(OSCPrivateAction::Type::LAT_LANE_CHANGE)
		{
			transition_dynamics_ = action.transition_dynamics_;
			target_ = action.target_;
			start_t_ = action.start_t_;
			target_lane_offset_ = action.target_lane_offset_;
			target_lane_id_ = action.target_lane_id_;
			elapsed_ = action.elapsed_;
		}

		OSCPrivateAction* Copy()
		{
			LatLaneChangeAction *new_action = new LatLaneChangeAction(*this);
			return new_action;
		}

		void Step(double dt, double simTime);

		void Start();

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

		LatLaneOffsetAction(const LatLaneOffsetAction &action) : OSCPrivateAction(OSCPrivateAction::Type::LAT_LANE_OFFSET)
		{
			target_ = action.target_;
			elapsed_ = action.elapsed_;
			start_lane_offset_ = action.start_lane_offset_;
			dynamics_ = action.dynamics_;
		}

		OSCPrivateAction* Copy()
		{
			LatLaneOffsetAction *new_action = new LatLaneOffsetAction(*this);
			return new_action;
		}

		void Start();
		void Step(double dt, double simTime);
	};

	class SynchronizeAction : public OSCPrivateAction
	{
	public:
		roadmanager::Position *target_position_master_;
		roadmanager::Position *target_position_;
		Object *master_object_;
		LongSpeedAction::Target *final_speed_;

		SynchronizeAction() : OSCPrivateAction(OSCPrivateAction::Type::SYNCHRONIZE) 
		{
			master_object_ = 0;
			final_speed_ = 0;
			target_position_master_ = 0;
			target_position_ = 0;
			mode_ = SynchMode::MODE_NONE;
			submode_ = SynchSubmode::SUBMODE_NONE;
		}

		SynchronizeAction(const SynchronizeAction &action) : OSCPrivateAction(OSCPrivateAction::Type::SYNCHRONIZE)
		{
			master_object_ = action.master_object_;
			final_speed_ = action.final_speed_;
			target_position_master_ = action.target_position_master_;
			target_position_ = action.target_position_;
			mode_ = action.mode_;
			submode_ = action.submode_;
		}

		OSCPrivateAction* Copy()
		{
			SynchronizeAction *new_action = new SynchronizeAction(*this);
			return new_action;
		}

		void Step(double dt, double simTime);

		void Start()
		{
			if (object_->control_ == Object::Control::EXTERNAL ||
				object_->control_ == Object::Control::HYBRID_EXTERNAL)
			{
				// motion control handed over 
				return;
			}

			OSCAction::Start();
		}

	private:
		typedef enum {
			MODE_NONE,
			MODE_LINEAR,
			MODE_NON_LINEAR,
			MODE_STOPPED,
			MODE_STOP_IMMEDIATELY,
			MODE_WAITING,
		} SynchMode;

		typedef enum {
			SUBMODE_NONE,
			SUBMODE_CONVEX,
			SUBMODE_CONCAVE
		} SynchSubmode;

		SynchMode mode_;
		SynchSubmode submode_;

		double CalcSpeedForLinearProfile(double v_final, double time, double dist);
		void PrintStatus(const char* custom_msg);
		const char* Mode2Str(SynchMode mode);
		const char* SubMode2Str(SynchSubmode submode);
	};

	class PositionAction : public OSCPrivateAction
	{
	public:
		OSCPosition *position_;

		PositionAction() : OSCPrivateAction(OSCPrivateAction::Type::POSITION) {}
		
		PositionAction(const PositionAction &action) : OSCPrivateAction(OSCPrivateAction::Type::POSITION) 
		{
			position_ = action.position_;
		}

		OSCPrivateAction* Copy()
		{
			PositionAction *new_action = new PositionAction(*this);
			return new_action;
		}

		void Step(double dt, double simTime)
		{
			(void)dt;
			(void)simTime;
			object_->pos_.CopyRMPos(position_->GetRMPos());
			if (position_->type_ != OSCPosition::PositionType::ROUTE)
			{
				object_->pos_.CalcRoutePosition();
			}
			LOG("Step %s pos: ", object_->name_.c_str());
			position_->Print();

			OSCAction::Stop();
		}

		void Start();
	};

	class FollowRouteAction : public OSCPrivateAction
	{
	public:
		roadmanager::Route *route_;

		FollowRouteAction() : OSCPrivateAction(OSCPrivateAction::Type::FOLLOW_ROUTE) {}

		FollowRouteAction(const FollowRouteAction &action) : OSCPrivateAction(OSCPrivateAction::Type::FOLLOW_ROUTE)
		{
			route_ = action.route_;
		}

		OSCPrivateAction* Copy()
		{
			FollowRouteAction *new_action = new FollowRouteAction(*this);
			return new_action;
		}

		void Step(double dt, double simTime)
		{
			(void)dt;
			(void)simTime;
		}

		void Start();
	};

	class FollowTrajectoryAction : public OSCPrivateAction
	{
	public:
		typedef enum {
			NONE,
			TIMING_RELATIVE,
			TIMING_ABSOLUTE
		} TimingDomain;

		roadmanager::Trajectory* traj_;
		TimingDomain timing_domain_;
		double timing_scale_;
		double timing_offset_;
		double time_;

		FollowTrajectoryAction() : traj_(0), timing_domain_(TimingDomain::NONE), timing_scale_(1), 
			timing_offset_(0), time_(0), OSCPrivateAction(OSCPrivateAction::Type::FOLLOW_TRAJECTORY) {}

		FollowTrajectoryAction(const FollowTrajectoryAction& action) : OSCPrivateAction(OSCPrivateAction::Type::FOLLOW_TRAJECTORY)
		{
			traj_ = action.traj_;
			timing_domain_ = action.timing_domain_;
			timing_scale_ = action.timing_scale_;
			timing_offset_ = action.timing_offset_;
			time_ = 0;
		}

		OSCPrivateAction* Copy()
		{
			FollowTrajectoryAction* new_action = new FollowTrajectoryAction(*this);
			return new_action;
		}

		void Step(double dt, double simTime);

		void Start();
	};

	class AutonomousAction : public OSCPrivateAction
	{
	public:
		typedef enum
		{
			LONGITUDINAL,
			LATERAL,
			BOTH
		} DomainType;

		DomainType domain_;
		bool activate_;

		AutonomousAction() : OSCPrivateAction(OSCPrivateAction::Type::AUTONOMOUS) {}

		AutonomousAction(const AutonomousAction &action) : OSCPrivateAction(OSCPrivateAction::Type::AUTONOMOUS) 
		{
			domain_ = action.domain_;
			activate_ = action.activate_;
		}

		OSCPrivateAction* Copy()
		{
			AutonomousAction *new_action = new AutonomousAction(*this);
			return new_action;
		}

		void Step(double dt, double simTime) { }  // put driver model here

		void Start()
		{
			if (object_->control_ == Object::Control::EXTERNAL ||
				object_->control_ == Object::Control::HYBRID_EXTERNAL)
			{
				// motion control handed over 
				return;
			}

			if (activate_ == true)
			{
				// activate driver model
				LOG("Non existing driver model activated");
			}
			else
			{
				// activate driver model
				LOG("Non existing driver model deactivated");
			}

			OSCAction::Start();
		}
	};
}
