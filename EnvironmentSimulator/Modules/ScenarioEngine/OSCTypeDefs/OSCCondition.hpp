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
#include "Parameters.hpp"

namespace scenarioengine
{
	// Forward declaration 
	class StoryBoard;
	
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
		bool last_trig_;    // trig value from last evaluation
		ConditionEdge edge_;
		SE_SimulationTimer timer_;

		OSCCondition(ConditionType base_type) : base_type_(base_type), evaluated_(false), 
			last_result_(false), last_trig_(false), edge_(ConditionEdge::NONE) {}

		bool Evaluate(StoryBoard *storyBoard, double sim_time);
		virtual bool CheckCondition(StoryBoard *storyBoard, double sim_time, bool log = false) = 0;
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
			END_OF_ROAD,
			TIME_TO_COLLISION,
			COLLISION,
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

		bool CheckCondition(StoryBoard* storyBoard, double sim_time, bool log = false);
		TrigByTimeHeadway() : TrigByEntity(TrigByEntity::EntityConditionType::TIME_HEADWAY) {}
	};

	class TrigByTimeToCollision : public TrigByEntity
	{
	public:
		Object* object_;
		OSCPosition* position_;
		double value_;
		bool freespace_;
		bool along_route_;
		Rule rule_;

		bool CheckCondition(StoryBoard* storyBoard, double sim_time, bool log = false);
		TrigByTimeToCollision() : object_(0), position_(0), TrigByEntity(TrigByEntity::EntityConditionType::TIME_TO_COLLISION) {}
	};

	class TrigByReachPosition : public TrigByEntity
	{
	public:
		OSCPosition *position_;
		double tolerance_;

		bool CheckCondition(StoryBoard* storyBoard, double sim_time, bool log = false);
		TrigByReachPosition() : TrigByEntity(TrigByEntity::EntityConditionType::REACH_POSITION) {}
	};

	class TrigByDistance : public TrigByEntity
	{
	public:
		OSCPosition *position_;
		double value_;
		bool freespace_;
		bool along_route_;
		Rule rule_;
		
		bool CheckCondition(StoryBoard* storyBoard, double sim_time, bool log = false);
		TrigByDistance() : TrigByEntity(TrigByEntity::EntityConditionType::DISTANCE) {}
	};

	class TrigByTraveledDistance : public TrigByEntity
	{
	public:
		double value_;

		bool CheckCondition(StoryBoard* storyBoard, double sim_time, bool log = false); 
		TrigByTraveledDistance() : value_(0), TrigByEntity(TrigByEntity::EntityConditionType::TRAVELED_DISTANCE) {}
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

		bool CheckCondition(StoryBoard* storyBoard, double sim_time, bool log = false); 
		TrigByRelativeDistance() : object_(0), value_(0.0), TrigByEntity(TrigByEntity::EntityConditionType::RELATIVE_DISTANCE) {}
	};

	class TrigByCollision : public TrigByEntity
	{
	public:
		typedef enum
		{
			LONGITUDINAL,
			LATERAL,
			INTERIAL
		} RelativeDistanceType;

		Object* object_;
		Object::Type type_;
		Rule rule_;

		bool CheckCondition(StoryBoard* storyBoard, double sim_time, bool log = false);
		TrigByCollision() : object_(0), TrigByEntity(TrigByEntity::EntityConditionType::COLLISION) {}
	};

	class TrigByEndOfRoad : public TrigByEntity
	{
	public:

		Object* object_;
		double duration_;
		Rule rule_;

		bool CheckCondition(StoryBoard* storyBoard, double sim_time, bool log = false);
		TrigByEndOfRoad() : TrigByEntity(TrigByEntity::EntityConditionType::END_OF_ROAD) {}

	private:
		double elapsed_time_;
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

		bool CheckCondition(StoryBoard* storyBoard, double sim_time, bool log = false); 
		TrigByState(CondElementState state, StoryBoardElement::ElementType element_type, std::string element_name) :
			OSCCondition(BY_STATE), state_(state), element_type_(element_type), element_name_(element_name) {}
		std::string CondElementState2Str(CondElementState state);
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

		bool CheckCondition(StoryBoard* storyBoard, double sim_time, bool log = false);
		TrigBySimulationTime() : TrigByValue(TrigByValue::Type::SIMULATION_TIME) {}
	};

	class TrigByParameter : public TrigByValue
	{
	public:
		std::string name_;
		std::string value_;
		Rule rule_;
		Parameters* parameters_;


		bool CheckCondition(StoryBoard* storyBoard, double sim_time, bool log = false);
		TrigByParameter() : TrigByValue(TrigByValue::Type::PARAMETER) {}
	};

}
