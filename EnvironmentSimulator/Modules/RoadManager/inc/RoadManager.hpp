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

#include <cmath>
#include <list>
#include <map>
#include <string>
#include <vector>
#include "CommonMini.hpp"
#include "pugixml.hpp"
#include "Polynomial.hpp"
#include "Position.hpp"
#include "Road.hpp"
#include "OSI.hpp"

namespace roadmanager {
int GetNewGlobalLaneId();
int GetNewGlobalLaneBoundaryId();

// Forward declarations
class Route;
class RMTrajectory;

// A route is a sequence of positions, at least one per road along the route
class Route {
   public:
	Route() : invalid_route_(false), waypoint_idx_(-1), path_s_(0), length_(0) {}

	/**
	Adds a waypoint to the route. One waypoint per road. At most one junction between waypoints.
	@param position A regular position created with road, lane or world coordinates
	@return Non zero return value indicates error of some kind
	*/
	int AddWaypoint(Position* position);
	int GetWayPointDirection(int index);

	void setName(std::string name);
	std::string getName();
	double GetLength() { return length_; }
	void CheckValid();
	bool IsValid() { return !invalid_route_; }

	// Current route position data
	// Actual object position might differ, e.g. laneId or even trackId in junctions
	double GetTrackS() { return currentPos_.GetS(); }
	double GetPathS() { return path_s_; }
	int GetLaneId() { return currentPos_.GetLaneId(); }
	int GetTrackId() { return currentPos_.GetTrackId(); }
	Position* GetWaypoint(int index = -1);	// -1 means current
	Road* GetRoadAtOtherEndOfConnectingRoad(Road* incoming_road);
	int GetDirectionRelativeRoad();
	Position* GetCurrentPosition() { return &currentPos_; }

	/**
	Specify route position in terms of a track ID and track S value
	@return Non zero return value indicates error of some kind
	*/
	Position::ErrorCode SetTrackS(int trackId, double s);

	/**
	Move current position forward, or backwards, ds meters along the route
	@param ds Distance to move, negative will move backwards
	@param actualDistance Distance considering lateral offset and curvature (true/default) or along centerline
	(false)
	@return Non zero return value indicates error of some kind, most likely End Of Route
	*/
	Position::ErrorCode MovePathDS(double ds);

	/**
	Move current position to specified S-value along the route
	@param route_s Distance to move, negative will move backwards
	@return Non zero return value indicates error of some kind, most likely End Of Route
	*/
	Position::ErrorCode SetPathS(double s);

	Position::ErrorCode CopySFractionOfLength(Position* pos);

	std::vector<Position> minimal_waypoints_;  // used only for the default controllers
	std::vector<Position> all_waypoints_;	   // used for user-defined controllers
	std::string name_;
	bool invalid_route_;
	double path_s_;
	Position currentPos_;
	double length_;
	int waypoint_idx_;
};

// A Road Path is a linked list of road links (road connections or junctions)
// between a starting position and a target position
// The path can be calculated automatically
class RoadPath {
   public:
	typedef struct PathNode {
		RoadLink* link;
		double dist;
		Road* fromRoad;
		int fromLaneId;
		ContactPointType contactPoint;
		PathNode* previous;
	} PathNode;

	std::vector<PathNode*> visited_;
	std::vector<PathNode*> unvisited_;
	const Position* startPos_;
	const Position* targetPos_;
	int direction_;	 // direction of path from starting pos. 0==not set, 1==forward, 2==backward

	RoadPath(const Position* startPos, const Position* targetPos)
		: startPos_(startPos), targetPos_(targetPos){};
	~RoadPath();

	/**
	Calculate shortest path between starting position and target position,
	using Dijkstra's algorithm https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
	it also calculates the length of the path, or distance between the positions
	positive distance means that the shortest path was found in forward direction
	negative distance means that the shortest path goes in opposite direction from the heading of the starting
	position
	@param dist A reference parameter into which the calculated path distance is stored
	@param bothDirections Set to true in order to search also backwards from object
	@param maxDist If set the search along each path branch will terminate after reaching this distance
	@return 0 on success, -1 on failure e.g. path not found
	*/
	int Calculate(double& dist, bool bothDirections = true, double maxDist = LARGE_NUMBER);

   private:
	bool CheckRoad(Road* checkRoad, RoadPath::PathNode* srcNode, Road* fromRoad, int fromLaneId);
};


class PolyLineBase {
   public:
	PolyLineBase()
		: length_(0), vIndex_(0), currentPos_({0, 0, 0, 0, 0, 0, 0, 0, false}), interpolateHeading_(false) {}
	TrajVertex* AddVertex(TrajVertex p);
	TrajVertex* AddVertex(double x, double y, double z, double h);
	TrajVertex* AddVertex(double x, double y, double z);

	/**
	 * Update vertex position and recalculate dependent values, e.g. length and heading
	 * NOTE: Need to be called in order, starting from i=0
	 * @param i Index of vertex to update
	 * @param x X coordinate of new position
	 * @param y Y coordinate of new position
	 * @param z Z coordinate of new position
	 * @param h Heading
	 */
	TrajVertex* UpdateVertex(int i, double x, double y, double z, double h);

	/**
	 * Update vertex position and recalculate dependent values, e.g. length and heading
	 * NOTE: Need to be called in order, starting from i=0
	 * @param i Index of vertex to update
	 * @param x X coordinate of new position
	 * @param y Y coordinate of new position
	 * @param z Z coordinate of new position
	 */
	TrajVertex* UpdateVertex(int i, double x, double y, double z);

	void reset() { length_ = 0.0; }
	int Evaluate(double s, TrajVertex& pos, double cornerRadius, int startAtIndex);
	int Evaluate(double s, TrajVertex& pos, double cornerRadius);
	int Evaluate(double s, TrajVertex& pos, int startAtIndex);
	int Evaluate(double s, TrajVertex& pos);
	int FindClosestPoint(double xin, double yin, TrajVertex& pos, int& index, int startAtIndex = 0);
	int FindPointAhead(double s_start, double distance, TrajVertex& pos, int& index, int startAtIndex = 0);
	int GetNumberOfVertices() { return (int)vertex_.size(); }
	TrajVertex* GetVertex(int index);
	void Reset();
	int Time2S(double time, double& s);

	std::vector<TrajVertex> vertex_;
	TrajVertex currentPos_;
	double length_;
	int vIndex_;
	bool interpolateHeading_;

   protected:
	int EvaluateSegmentByLocalS(int i, double local_s, double cornerRadius, TrajVertex& pos);
};

// Trajectory stuff
class Shape {
   public:
	typedef enum { POLYLINE, CLOTHOID, NURBS, SHAPE_TYPE_UNDEFINED } ShapeType;

	typedef enum { TRAJ_PARAM_TYPE_S, TRAJ_PARAM_TYPE_TIME } TrajectoryParamType;

	Shape(ShapeType type) : type_(type) {}
	virtual int Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos) { return -1; };
	int FindClosestPoint(double xin, double yin, TrajVertex& pos, int& index, int startAtIndex = 0) {
		return -1;
	};
	virtual double GetLength() { return 0.0; }
	virtual double GetStartTime() { return 0.0; }
	virtual double GetDuration() { return 0.0; }
	ShapeType type_;

	PolyLineBase pline_;  // approximation of shape, used for calculations and visualization
};

class PolyLineShape : public Shape {
   public:
	class Vertex {
	   public:
		Position pos_;
	};

	PolyLineShape() : Shape(ShapeType::POLYLINE) {}
	void AddVertex(Position pos, double time, bool calculateHeading);
	int Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos);
	double GetLength() { return pline_.length_; }
	double GetStartTime();
	double GetDuration();

	std::vector<Vertex*> vertex_;
};

class ClothoidShape : public Shape {
   public:
	ClothoidShape(roadmanager::Position pos,
				  double curv,
				  double curvDot,
				  double len,
				  double tStart,
				  double tEnd);

	int Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos);
	int EvaluateInternal(double s, TrajVertex& pos);
	void CalculatePolyLine();
	double GetLength() { return spiral_->GetLength(); }
	double GetStartTime();
	double GetDuration();

	Position pos_;
	roadmanager::Spiral* spiral_;  // make use of the OpenDRIVE clothoid definition
	double t_start_;
	double t_end_;
};

/**
	This nurbs implementation is strongly inspired by the "Nurbs Curve Example" at:
	https://nccastaff.bournemouth.ac.uk/jmacey/OldWeb/RobTheBloke/www/opengl_programming.html
*/
class NurbsShape : public Shape {
	class ControlPoint {
	   public:
		Position pos_;
		double time_;
		double weight_;
		double t_;
		bool calcHeading_;

		ControlPoint(Position pos, double time, double weight, bool calcHeading)
			: pos_(pos), time_(time), weight_(weight), calcHeading_(calcHeading) {}
	};

   public:
	NurbsShape(int order) : order_(order), Shape(ShapeType::NURBS), length_(0) {
		pline_.interpolateHeading_ = true;
	}

	void AddControlPoint(Position pos, double time, double weight, bool calcHeading);
	void AddKnots(std::vector<double> knots);
	int Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos);
	int EvaluateInternal(double s, TrajVertex& pos);

	int order_;
	std::vector<ControlPoint> ctrlPoint_;
	std::vector<double> knot_;
	std::vector<double> d_;	 // used for temporary storage of CoxDeBoor weigthed control points
	std::vector<double>
		dPeakT_;  // used for storage of at what t value the corresponding ctrlPoint contribution peaks
	std::vector<double>
		dPeakValue_;  // used for storage of at what t value the corresponding ctrlPoint contribution peaks

	void CalculatePolyLine();
	double GetLength() { return length_; }
	double GetStartTime();
	double GetDuration();

   private:
	double CoxDeBoor(double x, int i, int p, const std::vector<double>& t);
	double length_;
};

class RMTrajectory {
   public:
	Shape* shape_;

	RMTrajectory(Shape* shape, std::string name, bool closed) : shape_(shape), name_(name), closed_(closed) {}
	RMTrajectory() : shape_(0), closed_(false) {}
	void Freeze();
	double GetLength() { return shape_ ? shape_->GetLength() : 0.0; }
	double GetTimeAtS(double s);
	double GetStartTime();
	double GetDuration();

	std::string name_;
	bool closed_;
};

}  // namespace roadmanager
