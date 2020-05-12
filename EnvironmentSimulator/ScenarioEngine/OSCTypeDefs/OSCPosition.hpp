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
#include "RoadManager.hpp"
#include "Entities.hpp"

namespace scenarioengine
{

	class OSCOrientation
	{
	public:
		typedef enum
		{
			RELATIVE,
			ABSOLUTE,
			UNDEFINED
		} OrientationType;

		OSCOrientation() : type_(OrientationType::UNDEFINED), h_(0), p_(0), r_(0) {}
		OSCOrientation(OrientationType type, double h, double p, double r) : type_(type), h_(h), p_(p), r_(r) {}

		OrientationType type_;
		double h_; 
		double p_;
		double r_;
	};

	class OSCPosition
	{
	public:

		typedef enum
		{
			WORLD,
			RELATIVE_WORLD,
			RELATIVE_OBJECT,
			ROAD,
			RELATIVE_ROAD,
			LANE,
			RELATIVE_LANE,
			ROUTE,
			UNDEFINED
		} PositionType;

		OSCPosition() : type_(PositionType::UNDEFINED) {}
		OSCPosition(PositionType type) : type_(type) {}
		virtual ~OSCPosition() {}

		PositionType type_;

		virtual double GetX() { return position_.GetX(); }
		virtual double GetY() { return position_.GetY(); }
		virtual double GetZ() { return position_.GetZ(); }
		virtual double GetH() { return position_.GetH(); }
		virtual double GetP() { return position_.GetP(); }
		virtual double GetR() { return position_.GetR(); }
		virtual void Print() = 0;
		virtual void Evaluate() = 0;
		virtual roadmanager::Position *GetRMPos() { return &position_; }

	protected:
		roadmanager::Position position_;
	};

	class OSCPositionWorld : OSCPosition
	{
	public:
		OSCPositionWorld() : OSCPosition(PositionType::WORLD) {}
		OSCPositionWorld(double x, double y, double z, double h, double p, double r);

		void Print() { position_.Print(); }
		void Evaluate() {}  // No need to evaluate, position already in cartesian coordinates
	};

	class OSCPositionLane : OSCPosition
	{
	public:
		OSCPositionLane() : OSCPosition(PositionType::LANE) {}
		OSCPositionLane(int roadId, int landId, double s, double offset, OSCOrientation orientation);

		void Print() { position_.Print(); }
		void Evaluate() {}  // No need to evaluate, position already in cartesian coordinates
	};

	class OSCPositionRelativeObject : OSCPosition
	{
	public:
		Object *object_;
		double dx_;
		double dy_;
		double dz_;
		OSCOrientation o_;

		OSCPositionRelativeObject(Object *object, double dx, double dy, double dz, OSCOrientation orientation);

		double GetX() { Evaluate(); return position_.GetX(); }
		double GetY() { Evaluate(); return position_.GetY(); }
		double GetZ() { Evaluate(); return position_.GetZ(); }
		double GetH() { Evaluate(); return position_.GetH(); }
		double GetP() { Evaluate(); return position_.GetP(); }
		double GetR() { Evaluate(); return position_.GetR(); }

		void Print();
		void Evaluate();
		roadmanager::Position *GetRMPos() 
		{ 
			Evaluate();
			return &position_;
		}
	};

	class OSCPositionRelativeWorld : OSCPosition
	{
	public:
		Object* object_;
		double dx_;
		double dy_;
		double dz_;
		OSCOrientation o_;

		OSCPositionRelativeWorld(Object* object, double dx, double dy, double dz, OSCOrientation orientation);

		double GetX() { Evaluate(); return position_.GetX(); }
		double GetY() { Evaluate(); return position_.GetY(); }
		double GetZ() { Evaluate(); return position_.GetZ(); }
		double GetH() { Evaluate(); return position_.GetH(); }
		double GetP() { Evaluate(); return position_.GetP(); }
		double GetR() { Evaluate(); return position_.GetR(); }

		void Print();
		void Evaluate();
		roadmanager::Position* GetRMPos()
		{
			Evaluate();
			return &position_;
		}
	};

	class OSCPositionRelativeLane : OSCPosition
	{
	public:
		Object *object_;
		int dLane_;
		double ds_;
		double offset_;
		OSCOrientation o_;

		OSCPositionRelativeLane(Object *object, int dLane, double ds, double offset, OSCOrientation orientation);

		void Print();
		void Evaluate();
		roadmanager::Position *GetRMPos()
		{
			Evaluate();
			return &position_;
		}
	};

	class OSCPositionRoute : OSCPosition
	{
	public:
		OSCPositionRoute() : OSCPosition(PositionType::ROUTE) {}
		OSCPositionRoute(roadmanager::Route *route, double s, int laneId, double laneOffset);

		void SetRoute(roadmanager::Route *route) { position_.SetRoute(route); }
		void SetRouteRefLaneCoord(roadmanager::Route *route, double pathS, int laneId, double laneOffset, OSCOrientation *orientation);
		void SetRouteRefLaneCoord(roadmanager::Route *route, double pathS, int laneId, double laneOffset);
		void SetRouteRelativeHeading(double h_relative) { position_.SetHeadingRelative(h_relative);  }
		
		void Print() { position_.Print(); }
		void Evaluate() {}  // No need to evaluate, position already in cartesian coordinates
	};

}
