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
#include <math.h>
#include "OSCCommon.hpp"
#include "CommonMini.hpp"
#include "OSCAction.hpp"
#include "Entities.hpp"
#include "OSCPosition.hpp"

namespace scenarioengine
{
	// Forward declaration 
	class StoryBoard;

	class Timer
	{
	public:
		__int64 start_time_;

		Timer() : start_time_(0) {}
		void Start() 
		{ 
			start_time_ = SE_getSystemTime(); 
		}
		
		void Reset() { start_time_ = 0; }

		bool Started() { return start_time_ > 0 ? true : false; }
		double DurationS() { return 1E-3 * (SE_getSystemTime() - start_time_);  }
		
	};
	
	class OSCCondition
	{
	public:

		typedef enum
		{
			BY_ENTITY,
			BY_STATE,
			BY_VALUE
		} ConditionType;

		typedef enum
		{
			RISING,
			FALLING,
			RISING_OR_FALLING,
			NONE,
			UNDEFINED
		} ConditionEdge;

		ConditionType base_type_;
		std::string name_;
		double delay_;
		bool evaluated_;
		bool last_result_;  // result from last evaluation
		ConditionEdge edge_;
		Timer timer_;

		OSCCondition(ConditionType base_type) : base_type_(base_type), evaluated_(false), last_result_(false), edge_(ConditionEdge::NONE) {}

		bool Evaluate(StoryBoard *storyBoard, double sim_time);
		virtual bool CheckCondition(StoryBoard *storyBoard, double sim_time) = 0;
		bool CheckEdge(bool new_value, bool old_value, OSCCondition::ConditionEdge edge);
	};

	class ConditionGroup
	{
	public:
		std::vector<OSCCondition*> condition_;

		bool Evaluate(StoryBoard *storyBoard, double sim_time);
	};

	class Trigger
	{
	public:
		std::vector<ConditionGroup*> conditionGroup_;

		bool Evaluate(StoryBoard *storyBoard, double sim_time);
	};

	class TrigByEntity : public OSCCondition
	{
	public:
		struct Entity
		{
			Object *object_;
		};

		typedef enum
		{
			ANY,
			ALL
		} TriggeringEntitiesRule;

		struct TriggeringEntities
		{
			std::vector<Entity> entity_;
			TriggeringEntitiesRule rule_;
		};

		typedef enum
		{
			TIME_HEADWAY,
			DISTANCE,
			RELATIVE_DISTANCE,
			REACH_POSITION,
			TRAVELED_DISTANCE,
			// not complete at all
		} EntityConditionType;

		TriggeringEntitiesRule triggering_entity_rule_;
		TriggeringEntities triggering_entities_;
		EntityConditionType type_;

		TrigByEntity(EntityConditionType type) : OSCCondition(OSCCondition::ConditionType::BY_ENTITY), type_(type) {}

		void Print()
		{
			LOG("");
		}
	};

	class TrigByTimeHeadway : public TrigByEntity
	{
	public:
		Object *object_;
		double value_;
		bool freespace_;
		bool along_route_;
		Rule rule_;

		TrigByTimeHeadway() : TrigByEntity(TrigByEntity::EntityConditionType::TIME_HEADWAY) {}

		bool CheckCondition(StoryBoard *storyBoard, double sim_time);
	};

	class TrigByReachPosition : public TrigByEntity
	{
	public:
		OSCPosition *position_;
		double tolerance_;

		TrigByReachPosition() : TrigByEntity(TrigByEntity::EntityConditionType::REACH_POSITION) {}

		bool CheckCondition(StoryBoard *storyBoard, double sim_time);
	};

	class TrigByDistance : public TrigByEntity
	{
	public:
		OSCPosition *position_;
		double value_;
		bool freespace_;
		bool along_route_;
		Rule rule_;

		TrigByDistance() : TrigByEntity(TrigByEntity::EntityConditionType::DISTANCE) {}

		bool CheckCondition(StoryBoard *storyBoard, double sim_time);
	};

	class TrigByTraveledDistance : public TrigByEntity
	{
	public:
		double value_;

		TrigByTraveledDistance() : value_(0), TrigByEntity(TrigByEntity::EntityConditionType::TRAVELED_DISTANCE) {}

		bool CheckCondition(StoryBoard* storyBoard, double sim_time);
	};

	class TrigByRelativeDistance : public TrigByEntity
	{
	public:
		typedef enum
		{
			LONGITUDINAL,
			LATERAL,
			INTERIAL
		} RelativeDistanceType;

		Object *object_;
		double value_;
		bool freespace_;
		RelativeDistanceType type_;
		Rule rule_;

		TrigByRelativeDistance() : TrigByEntity(TrigByEntity::EntityConditionType::RELATIVE_DISTANCE) {}

		bool CheckCondition(StoryBoard *storyBoard, double sim_time);
	};

	class TrigByState : public OSCCondition
	{
	public:

		typedef enum
		{
			STANDBY,
			RUNNING,
			COMPLETE,
			UNDEFINED_ELEMENT_STATE,
			START_TRANSITION,  
			END_TRANSITION,
			STOP_TRANSITION,
			SKIP_TRANSITION,
			COMPLETE_TRANSITION,
			UNDEFINED_ELEMENT_TRANSITION
		} CondElementState;

		CondElementState state_;
		StoryBoardElement::ElementType element_type_;
		std::string element_name_;

		TrigByState(CondElementState state, StoryBoardElement::ElementType element_type, std::string element_name) :
			OSCCondition(BY_STATE), state_(state), element_type_(element_type), element_name_(element_name) {}

		bool CheckCondition(StoryBoard *storyBoard, double sim_time);
	};

	class TrigByValue : public OSCCondition
	{
	public:
		typedef enum
		{
			PARAMETER,
			TIME_OF_DAY,
			SIMULATION_TIME,
			UNDEFINED
		} Type;

		Type type_;
		Rule rule_;

		TrigByValue(Type type) : OSCCondition(BY_VALUE), type_(type) {}
	};

	class TrigBySimulationTime : public TrigByValue
	{
	public:
		double value_;

		TrigBySimulationTime() : TrigByValue(TrigByValue::Type::TIME_OF_DAY) {}

		bool CheckCondition(StoryBoard *storyBoard, double sim_time);
	};

}
