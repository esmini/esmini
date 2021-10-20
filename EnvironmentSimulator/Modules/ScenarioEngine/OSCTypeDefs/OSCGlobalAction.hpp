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
#include <random>
#include "OSCAction.hpp"
#include "CommonMini.hpp"
#include "Parameters.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
//#include "ScenarioReader.hpp"
#include "OSCAABBTree.hpp"
#include <vector>
#include "OSCUtils.hpp"

namespace scenarioengine
{

	using aabbTree::Solutions;
	using std::vector;
	using namespace Utils;

	class OSCGlobalAction : public OSCAction
	{
	public:
		typedef enum
		{
			ENVIRONMENT,     // not supported yet
			ENTITY,          // not supported yet
			PARAMETER_SET,
			INFRASTRUCTURE,  // not supported yet
			SWARM_TRAFFIC,
		} Type;

		Type type_;

		OSCGlobalAction(OSCGlobalAction::Type type) : OSCAction(OSCAction::BaseType::GLOBAL), type_(type) {}

		virtual void print()
		{
			LOG("Virtual, should be overridden");
		}

		virtual OSCGlobalAction* Copy()
		{
			LOG("Virtual, should be overridden");
			return 0;
		};

		virtual std::string Type2Str()
		{
			return "OSCGlobalAction base class";
		};

	};

	class ParameterSetAction : public OSCGlobalAction
	{
	public:
		std::string name_;
		std::string value_;
		Parameters* parameters_;

		ParameterSetAction() : OSCGlobalAction(OSCGlobalAction::Type::PARAMETER_SET), name_(""), value_(""), parameters_(0) {};

		ParameterSetAction(const ParameterSetAction& action) : OSCGlobalAction(OSCGlobalAction::Type::PARAMETER_SET)
		{
			name_ = action.name_;
			value_ = action.value_;
		}

		OSCGlobalAction* Copy()
		{
			ParameterSetAction* new_action = new ParameterSetAction(*this);
			return new_action;
		}

		std::string Type2Str()
		{
			return "ParameterSetAction";
		};

		void Start(double simTime, double dt);
		void Step(double simTime, double dt);

		void print()
		{
			LOG("");
		}

	};

	class ScenarioReader;

	class SwarmTrafficAction : public OSCGlobalAction
	{
	public:

		struct SpawnInfo{
			int vehicleID;
			int outMidAreaCount;
			int roadID;
			int lane;
			double simTime;
		};

		typedef struct {
		    roadmanager::Position pos;
			roadmanager::Road *road;
		    int nLanes;
	    } SelectInfo;

		SwarmTrafficAction();

		SwarmTrafficAction(const SwarmTrafficAction& action) : OSCGlobalAction(OSCGlobalAction::Type::SWARM_TRAFFIC) {
		    spawnedV.clear();
			centralObject_ = action.centralObject_;
		}

		OSCGlobalAction* Copy() {
			SwarmTrafficAction* new_action = new SwarmTrafficAction(*this);
			return new_action;
		}

		void Start(double simTime, double dt);

		void Step(double simTime, double dt);

		void print() {
			LOG("");
		}

		void SetCentralObject(Object* centralObj) { centralObject_ = centralObj; }
		void SetInnerRadius(double innerRadius)   { innerRadius_   = innerRadius;}
		void SetSemiMajorAxes(double axes)        { semiMajorAxis_ = axes;       }
		void SetSemiMinorAxes(double axes)        { semiMinorAxis_ = axes;       }
		void SetEntities(Entities* entities)      { entities_      = entities;   }
		void SetGateway(ScenarioGateway* gateway) { gateway_ = gateway; }
		void SetReader(ScenarioReader* reader)    { reader_ = reader; }
		void SetNumberOfVehicles(int number)      { numberOfVehicles = number;   }
		void Setvelocity(double velocity)         { velocity_ = velocity;        }

    private:

		double velocity_;
		Entities *entities_;
		ScenarioGateway* gateway_;
		ScenarioReader* reader_;
		Object* centralObject_;
		aabbTree::ptTree rTree;
		unsigned long numberOfVehicles;
		std::vector<SpawnInfo> spawnedV;
		roadmanager::OpenDrive* odrManager_;
		double innerRadius_, semiMajorAxis_, semiMinorAxis_, midSMjA, midSMnA, minSize_, lastTime;
		std::vector<std::string> modelFilenames_;
		std::vector<int> modelIds_;

		int despawn(double simTime);
		void createRoadSegments(aabbTree::BBoxVec &vec);
		void spawn(Solutions sols, int replace, double simTime);
		inline bool ensureDistance(roadmanager::Position pos, int lane, double dist);
		void createEllipseSegments(aabbTree::BBoxVec &vec, double SMjA, double SMnA);
		inline void sampleRoads(int minN, int maxN, Solutions &sols, vector<SelectInfo> &info);
	};

}

