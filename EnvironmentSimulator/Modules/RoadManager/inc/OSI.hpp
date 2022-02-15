#ifndef OSI_HPP
#define OSI_HPP

#include <vector>
#include "StructsandDefines.hpp"

class OSIPoints {
   public:
	OSIPoints() {}
	OSIPoints(std::vector<PointStruct> points) : point_(points) {}
	void Set(std::vector<PointStruct> points) { point_ = points; }
	std::vector<PointStruct>& GetPoints() { return point_; }
	PointStruct& GetPoint(int i);
	double GetXfromIdx(int i);
	double GetYfromIdx(int i);
	double GetZfromIdx(int i);
	int GetNumOfOSIPoints();
	double GetLength();

   private:
	std::vector<PointStruct> point_;
};
/**
	function that checks if two sets of osi points has the same start/end
	@return the number of points that are within tolerance (0,1 or 2)
*/
int CheckOverlapingOSIPoints(OSIPoints* first_set, OSIPoints* second_set, double tolerance);

class LaneBoundaryOSI {
   public:
	LaneBoundaryOSI(int gbid) : global_id_(gbid) {}
	~LaneBoundaryOSI(){};
	void SetGlobalId();
	int GetGlobalId() { return global_id_; }
	OSIPoints* GetOSIPoints() { return &osi_points_; }
	OSIPoints osi_points_;

   private:
	int global_id_;	 // Unique ID for OSI
};

#endif