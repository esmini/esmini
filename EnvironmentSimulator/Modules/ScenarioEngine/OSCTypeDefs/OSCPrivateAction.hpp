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
#include "Controller.hpp"

#include <iostream>
#include <string>
#include <math.h>

namespace scenarioengine
{

	#define DISTANCE_TOLERANCE (0.5)  // meter
	#define SYNCH_DISTANCE_TOLERANCE (1.0)  // meter
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
			CONTROLLER,
			ASSIGN_CONTROLLER,
			ACTIVATE_CONTROLLER,
			OVERRIDE_CONTROLLER,
			TELEPORT,
			ASSIGN_ROUTE,
			FOLLOW_TRAJECTORY,
			Acquire_POSITION,
			SYNCHRONIZE
		} ActionType;

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

		ActionType type_;
		ControlDomains domain_;
		Object *object_;

		OSCPrivateAction(OSCPrivateAction::ActionType type, ControlDomains domain) :
			OSCAction(OSCAction::BaseType::PRIVATE), type_(type), domain_(domain), object_(0) {}

		virtual void print()
		{
			LOG("Virtual, should be overridden");
		};

		virtual OSCPrivateAction* Copy()
		{
			LOG("Virtual, should be overridden");
			return 0;
		};

		std::string Type2Str()
		{
			return "OSCPrivateAction base class";
		};

		ControlDomains GetDomain() { return domain_; }

		virtual void ReplaceObjectRefs(Object*, Object*) {};

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
			} TargetType;

			TargetType type_;
			double value_;

			Target(TargetType type) : type_(type), value_(0) {}
			virtual ~Target() {}
			virtual double GetValue() = 0;
		};

		class TargetAbsolute : public Target
		{
		public:
			TargetAbsolute() : Target(TargetType::ABSOLUTE) {}

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

			Object* object_;
			ValueType value_type_;
			bool continuous_;

			TargetRelative() : Target(TargetType::RELATIVE), continuous_(false), consumed_(false), object_speed_(0) {}

			double GetValue();

		private:
			bool consumed_;
			double object_speed_;
		};

		Target* target_;
		double start_speed_;
		double elapsed_;
		double sim_time_;

		LongSpeedAction() : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_SPEED, ControlDomains::DOMAIN_LONG),
			target_(0), start_speed_(0), sim_time_(0), elapsed_(0) {}

		LongSpeedAction(const LongSpeedAction& action) : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_SPEED, ControlDomains::DOMAIN_LONG)
		{
			target_ = action.target_;
			transition_dynamics_ = action.transition_dynamics_;
			elapsed_ = action.elapsed_;
			start_speed_ = action.start_speed_;
			sim_time_ = action.sim_time_;
		}

		OSCPrivateAction* Copy()
		{
			LongSpeedAction* new_action = new LongSpeedAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "SpeedAction";
		};

		void Start(double simTime, double dt);
		void Step(double simTime, double dt);

		void print()
		{
			LOG("");
		}

		void ReplaceObjectRefs(Object* obj1, Object* obj2);
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

		typedef enum
		{
			NONE,
			TRAILING,
			LEADING,
			ANY
		} DisplacementType;

		Object *target_object_;
		double distance_;
		DistType dist_type_;
		double freespace_;
		bool continuous_;
		double sim_time_;
		DisplacementType displacement_;

		LongDistanceAction() : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_DISTANCE, ControlDomains::DOMAIN_LONG),
			target_object_(0), distance_(0), dist_type_(DistType::DISTANCE), freespace_(0), acceleration_(0), sim_time_(0),
			displacement_(DisplacementType::NONE)
		{
            dynamics_.max_acceleration_ = 0;
            dynamics_.max_deceleration_ = 0;
            dynamics_.max_speed_ = 0;
            dynamics_.none_ = true;
		}

		LongDistanceAction(const LongDistanceAction &action) : OSCPrivateAction(OSCPrivateAction::ActionType::LONG_DISTANCE, ControlDomains::DOMAIN_LONG)
		{
			target_object_ = action.target_object_;
			dynamics_ = action.dynamics_;
			distance_ = action.distance_;
			dist_type_ = action.dist_type_;
			freespace_ = action.freespace_;
			acceleration_ = action.acceleration_;
			sim_time_ = action.sim_time_;
			displacement_ = action.displacement_;
		}

		OSCPrivateAction* Copy()
		{
			LongDistanceAction *new_action = new LongDistanceAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "LongitudinalDistanceAction";
		};

		void Start(double simTime, double dt);
		void Step(double simTime, double dt);

		void print()
		{
			LOG("");
		}

		void ReplaceObjectRefs(Object* obj1, Object* obj2);

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
			Object* object_;

			TargetRelative() : Target(Target::Type::RELATIVE), object_(0) {}
		};

		Target* target_;
		double start_t_;
		double target_t_;
		double target_lane_offset_;
		int target_lane_id_;
		double elapsed_;
		double t_;
		double sim_time_;

		LatLaneChangeAction(LatLaneChangeAction::DynamicsDimension timing_type = DynamicsDimension::TIME) :
			OSCPrivateAction(OSCPrivateAction::ActionType::LAT_LANE_CHANGE, ControlDomains::DOMAIN_LAT)
		{
			transition_dynamics_.dimension_ = timing_type;
			elapsed_ = 0;
			target_t_ = 0;
			t_ = 0;
			sim_time_ = 0;
		}

		LatLaneChangeAction(const LatLaneChangeAction& action) : OSCPrivateAction(OSCPrivateAction::ActionType::LAT_LANE_CHANGE, ControlDomains::DOMAIN_LAT)
		{
			transition_dynamics_ = action.transition_dynamics_;
			target_ = action.target_;
			start_t_ = action.start_t_;
			target_t_ = action.target_t_;
			target_lane_offset_ = action.target_lane_offset_;
			target_lane_id_ = action.target_lane_id_;
			elapsed_ = action.elapsed_;
			t_ = action.t_;
			sim_time_ = action.sim_time_;
		}

		OSCPrivateAction* Copy()
		{
			LatLaneChangeAction* new_action = new LatLaneChangeAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "LaneChangeAction";
		};

		void Step(double simTime, double dt);
		void Start(double simTime, double dt);

		void ReplaceObjectRefs(Object* obj1, Object* obj2);
	};

	class LatLaneOffsetAction : public OSCPrivateAction
	{
	public:
		struct
		{
			double max_lateral_acc_;
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
		double target_lane_offset_;
		double sim_time_;

		LatLaneOffsetAction() : OSCPrivateAction(OSCPrivateAction::ActionType::LAT_LANE_OFFSET, ControlDomains::DOMAIN_LAT)
		{
			dynamics_.max_lateral_acc_ = 0;
			elapsed_ = 0;
			target_ = 0;
			target_lane_offset_ = 0;
			sim_time_ = 0;
		}

		LatLaneOffsetAction(const LatLaneOffsetAction &action) : OSCPrivateAction(OSCPrivateAction::ActionType::LAT_LANE_OFFSET, ControlDomains::DOMAIN_LAT)
		{
			target_ = action.target_;
			elapsed_ = action.elapsed_;
			start_lane_offset_ = action.start_lane_offset_;
			target_lane_offset_ = action.target_lane_offset_;
			dynamics_ = action.dynamics_;
			sim_time_ = action.sim_time_;
		}

		OSCPrivateAction* Copy()
		{
			LatLaneOffsetAction *new_action = new LatLaneOffsetAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "LaneOffsetAction";
		};

		void Start(double simTime, double dt);
		void Step(double simTime, double dt);

		void ReplaceObjectRefs(Object* obj1, Object* obj2);
	};

	class SynchronizeAction : public OSCPrivateAction
	{
	public:
		typedef enum
		{
			STEADY_STATE_NONE,
			STEADY_STATE_POS,
			STEADY_STATE_DIST,
			STEADY_STATE_TIME
		} SteadyStateType;

		typedef enum {
			MODE_NONE,
			MODE_LINEAR,
			MODE_NON_LINEAR,
			MODE_STOPPED,
			MODE_STOP_IMMEDIATELY,
			MODE_WAITING,
			MODE_STEADY_STATE
		} SynchMode;

		typedef enum {
			SUBMODE_NONE,
			SUBMODE_CONVEX,
			SUBMODE_CONCAVE
		} SynchSubmode;

		struct
		{
			SteadyStateType type_;
			union
			{
				roadmanager::Position *pos_;
				double time_;
				double dist_;
			};
		} steadyState_;

		SynchMode mode_;
		SynchSubmode submode_;

		roadmanager::Position *target_position_master_;
		roadmanager::Position *target_position_;
		Object *master_object_;
		LongSpeedAction::Target *final_speed_;
		double tolerance_;
		double tolerance_master_;
		double sim_time_;

		// Store calculated distances to use for comparison
		double lastDist_;
		double lastMasterDist_;

		SynchronizeAction() : OSCPrivateAction(OSCPrivateAction::ActionType::SYNCHRONIZE, ControlDomains::DOMAIN_LONG)
		{
			master_object_ = 0;
			final_speed_ = 0;
			target_position_master_ = 0;
			target_position_ = 0;
			mode_ = SynchMode::MODE_NONE;
			submode_ = SynchSubmode::SUBMODE_NONE;
			lastDist_ = LARGE_NUMBER;
			lastMasterDist_ = LARGE_NUMBER;
			tolerance_ = SYNCH_DISTANCE_TOLERANCE;
			tolerance_master_ = SYNCH_DISTANCE_TOLERANCE;
			steadyState_.type_ = SteadyStateType::STEADY_STATE_NONE;
		}

		SynchronizeAction(const SynchronizeAction &action) : OSCPrivateAction(OSCPrivateAction::ActionType::SYNCHRONIZE, ControlDomains::DOMAIN_LONG)
		{
			master_object_ = action.master_object_;
			final_speed_ = action.final_speed_;
			target_position_master_ = action.target_position_master_;
			target_position_ = action.target_position_;
			mode_ = action.mode_;
			submode_ = action.submode_;
			lastDist_ = LARGE_NUMBER;
			lastMasterDist_ = LARGE_NUMBER;
			tolerance_ = SYNCH_DISTANCE_TOLERANCE;
			tolerance_master_ = SYNCH_DISTANCE_TOLERANCE;
			steadyState_ = action.steadyState_;
		}

		OSCPrivateAction* Copy()
		{
			SynchronizeAction *new_action = new SynchronizeAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "SynchronizeAction";
		};

		~SynchronizeAction()
		{
			if (steadyState_.pos_ != 0)
			{
				delete steadyState_.pos_;
			}
			steadyState_.pos_ = 0;
		}

		void Step(double simTime, double dt);
		void Start(double simTime, double dt);

		const char* Mode2Str(SynchMode mode);
	private:

		double CalcSpeedForLinearProfile(double v_final, double time, double dist);
		void PrintStatus(const char* custom_msg);
		//const char* Mode2Str(SynchMode mode);
		const char* SubMode2Str(SynchSubmode submode);



		void ReplaceObjectRefs(Object* obj1, Object* obj2)
		{
			if (object_ == obj1)
			{
				object_ = obj2;
			}

			if (master_object_ == obj1)
			{
				master_object_ = obj2;
			}
		}
	};

	class TeleportAction : public OSCPrivateAction
	{
	public:
		roadmanager::Position *position_;

		TeleportAction() : OSCPrivateAction(OSCPrivateAction::ActionType::TELEPORT, ControlDomains::DOMAIN_BOTH) {}

		TeleportAction(const TeleportAction &action) : OSCPrivateAction(OSCPrivateAction::ActionType::TELEPORT, ControlDomains::DOMAIN_BOTH)
		{
			position_ = action.position_;
		}

		OSCPrivateAction* Copy()
		{
			TeleportAction *new_action = new TeleportAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "TeleportAction";
		};

		void Step(double simTime, double dt);
		void Start(double simTime, double dt);

		void ReplaceObjectRefs(Object* obj1, Object* obj2);
	};

	class AssignRouteAction : public OSCPrivateAction
	{
	public:
		roadmanager::Route *route_;

		AssignRouteAction() : route_(0), OSCPrivateAction(OSCPrivateAction::ActionType::ASSIGN_ROUTE, ControlDomains::DOMAIN_NONE) {}

		AssignRouteAction(const AssignRouteAction&action) : OSCPrivateAction(OSCPrivateAction::ActionType::ASSIGN_ROUTE, ControlDomains::DOMAIN_NONE)
		{
			route_ = action.route_;
		}

		OSCPrivateAction* Copy()
		{
			AssignRouteAction* new_action = new AssignRouteAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "AssignRouteAction";
		};

		void Start(double simTime, double dt);
		void Step(double simTime, double dt);

		void ReplaceObjectRefs(Object* obj1, Object* obj2);
	};

	class FollowTrajectoryAction : public OSCPrivateAction
	{
	public:
		typedef enum {
			NONE,
			TIMING_RELATIVE,
			TIMING_ABSOLUTE
		} TimingDomain;

		roadmanager::RMTrajectory* traj_;
		TimingDomain timing_domain_;
		double timing_scale_;
		double timing_offset_;
		double time_;
		double initialDistanceOffset_;

		FollowTrajectoryAction() : traj_(0), timing_domain_(TimingDomain::NONE), timing_scale_(1),
			timing_offset_(0), time_(0), initialDistanceOffset_(0), OSCPrivateAction(OSCPrivateAction::ActionType::FOLLOW_TRAJECTORY, ControlDomains::DOMAIN_BOTH) {}

		FollowTrajectoryAction(const FollowTrajectoryAction& action) : OSCPrivateAction(OSCPrivateAction::ActionType::FOLLOW_TRAJECTORY, ControlDomains::DOMAIN_BOTH)
		{
			traj_ = action.traj_;
			timing_domain_ = action.timing_domain_;
			timing_scale_ = action.timing_scale_;
			timing_offset_ = action.timing_offset_;
			initialDistanceOffset_ = action.timing_offset_;
			time_ = 0;
		}

		OSCPrivateAction* Copy()
		{
			FollowTrajectoryAction* new_action = new FollowTrajectoryAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "FollowTrajectoryAction";
		};

		void Step(double simTime, double dt);
		void Start(double simTime, double dt);
		void End();

		void ReplaceObjectRefs(Object* obj1, Object* obj2);
	};

	class AcquirePositionAction : public OSCPrivateAction
	{
	public:
		roadmanager::Position* target_position_;
		roadmanager::Route* route_;

		AcquirePositionAction() : route_(0), target_position_(0), OSCPrivateAction(OSCPrivateAction::ActionType::Acquire_POSITION, ControlDomains::DOMAIN_LONG) {}

		AcquirePositionAction(const AcquirePositionAction& action) : OSCPrivateAction(OSCPrivateAction::ActionType::Acquire_POSITION, ControlDomains::DOMAIN_LONG)
		{
			target_position_ = action.target_position_;
			route_ = action.route_;
		}

		OSCPrivateAction* Copy()
		{
			AcquirePositionAction* new_action = new AcquirePositionAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "AcquirePositionAction";
		};

		void Start(double simTime, double dt);
		void Step(double simTime, double dt);

		void ReplaceObjectRefs(Object* obj1, Object* obj2);
	};

	class AssignControllerAction : public OSCPrivateAction
	{
	public:
		Controller* controller_;

		AssignControllerAction(Controller *controller): controller_(controller),
			OSCPrivateAction(OSCPrivateAction::ActionType::ASSIGN_CONTROLLER,
				controller != nullptr ? controller->GetDomain() : ControlDomains::DOMAIN_NONE) {}

		AssignControllerAction(const AssignControllerAction& action) : OSCPrivateAction(OSCPrivateAction::ActionType::ASSIGN_CONTROLLER,
			action.controller_ != nullptr ? action.controller_->GetDomain() : ControlDomains::DOMAIN_NONE)
		{
			controller_ = action.controller_;
		};

		OSCPrivateAction* Copy()
		{
			AssignControllerAction* new_action = new AssignControllerAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "AssignControllerAction";
		};

		void Step(double, double) { }
		void Start(double simTime, double dt);

	};

	class ActivateControllerAction : public OSCPrivateAction
	{
	public:
		ControlDomains domainMask_;

		/**
		Default constructor assuming both domains (lat/long) activated
		@param domainMask bitmask according to Controller::Domain type
		*/
		ActivateControllerAction() : domainMask_(ControlDomains::DOMAIN_BOTH),
			OSCPrivateAction(OSCPrivateAction::ActionType::ACTIVATE_CONTROLLER, domainMask_) {}

		/**
		Constructor with domain specification
		@param domainMask bitmask according to Controller::Domain type
		*/
		ActivateControllerAction(ControlDomains domainMask) : domainMask_(domainMask),
			OSCPrivateAction(OSCPrivateAction::ActionType::ACTIVATE_CONTROLLER, domainMask) {}

		ActivateControllerAction(const ActivateControllerAction& action) :
			OSCPrivateAction(OSCPrivateAction::ActionType::ACTIVATE_CONTROLLER, action.domain_)
		{
			domainMask_ = action.domainMask_;
		}

		OSCPrivateAction* Copy()
		{
			ActivateControllerAction* new_action = new ActivateControllerAction(*this);
			return new_action;
		}

		void Start(double simTime, double dt)
		{
			if (object_->GetAssignedControllerType() != 0)
			{
				if (!object_->controller_->Active())
				{
					object_->controller_->Activate(domainMask_);
					LOG("Controller %s activated, domain mask=0x%X", object_->controller_->GetName().c_str(),  domainMask_);
				}
				else
				{
					LOG("Controller %s already active (domainmask 0x%X), deactivate first in order to activate on different domain(s)",
						object_->controller_->GetName().c_str(), domainMask_);
				}
			}
			else
			{
				LOG("No controller assigned!");
			}
			OSCAction::Start(simTime, dt);
		}

		std::string Type2Str()
		{
			return "ActivateControllerAction";
		};

		void Step(double, double) {}

		void End()
		{
			if (object_->GetActivatedControllerType() != 0 && object_->controller_ != nullptr)
			{
				object_->controller_->Deactivate();
			}
			// Make sure heading is aligned with road driving direction
			object_->pos_.SetHeadingRelative((object_->pos_.GetHRelative() > M_PI_2 && object_->pos_.GetHRelative() < 3 * M_PI_2) ? M_PI : 0.0);
			OSCAction::End();
		}
	};

	class VisibilityAction : public OSCPrivateAction
	{
	public:

		bool graphics_;
		bool traffic_;
		bool sensors_;

		VisibilityAction() : graphics_(true), traffic_(true), sensors_(true),
			OSCPrivateAction(OSCPrivateAction::ActionType::VISIBILITY, ControlDomains::DOMAIN_NONE) {}

		VisibilityAction(const VisibilityAction& action) : graphics_(true), traffic_(true), sensors_(true),
			OSCPrivateAction(OSCPrivateAction::ActionType::VISIBILITY, ControlDomains::DOMAIN_NONE)
		{
			graphics_ = action.graphics_;
			traffic_ = action.traffic_;
			sensors_ = action.sensors_;
		}

		OSCPrivateAction* Copy()
		{
			VisibilityAction* new_action = new VisibilityAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "VisibilityAction";
		};

		void Step(double simTime, double dt);
		void Start(double simTime, double dt);
	};

	class OverrideControlAction : public OSCPrivateAction
	{
	public:

		std::vector<Object::OverrideActionStatus> overrideActionList;

		Object::OverrideType type_;

		// assume both domains
		OverrideControlAction(double value, bool active, Object::OverrideType type) :
			OSCPrivateAction(OSCPrivateAction::ActionType::OVERRIDE_CONTROLLER, ControlDomains::DOMAIN_BOTH), type_(type) {}
		OverrideControlAction() : OverrideControlAction(0, false, Object::OverrideType::OVERRIDE_UNDEFINED) {}
		~OverrideControlAction() {}

		void Step(double simTime, double dt);
		void Start(double simTime, double dt);

		OSCPrivateAction *Copy()
		{
			OverrideControlAction *new_action = new OverrideControlAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "OverrideControlAction";
		};

		// Input value range: [0..1] for Throttle, Brake, Clutch and ParkingBrake. [-2*PI..2*PI] for SteeringWheel. [-1,0,1,2,3,4,5,6,7,8] for Gear.
		// Function will cut the value to the near limit if the value is beyond limit and round the value in Gear case.
		double RangeCheckAndErrorLog(Object::OverrideType type, double valueCheck, double lowerLimit = 0.0, double upperLimit = 1.0, bool ifRound = false);

	};

}
