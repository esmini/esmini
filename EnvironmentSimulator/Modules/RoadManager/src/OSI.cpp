#include "OSI.hpp"

int CheckOverlapingOSIPoints(OSIPoints* first_set, OSIPoints* second_set, double tolerance) {
	std::vector<double> distances;
	int retvalue = 0;
	distances.push_back(PointDistance2D(first_set->GetPoint(0).x, first_set->GetPoint(0).y,
										second_set->GetPoint(0).x, second_set->GetPoint(0).y));
	distances.push_back(PointDistance2D(first_set->GetPoint(0).x, first_set->GetPoint(0).y,
										second_set->GetPoint(second_set->GetNumOfOSIPoints() - 1).x,
										second_set->GetPoint(second_set->GetNumOfOSIPoints() - 1).y));
	distances.push_back(PointDistance2D(first_set->GetPoint(first_set->GetNumOfOSIPoints() - 1).x,
										first_set->GetPoint(first_set->GetNumOfOSIPoints() - 1).y,
										second_set->GetPoint(0).x, second_set->GetPoint(0).y));
	distances.push_back(PointDistance2D(first_set->GetPoint(first_set->GetNumOfOSIPoints() - 1).x,
										first_set->GetPoint(first_set->GetNumOfOSIPoints() - 1).y,
										second_set->GetPoint(second_set->GetNumOfOSIPoints() - 1).x,
										second_set->GetPoint(second_set->GetNumOfOSIPoints() - 1).y));

	for (int i = 0; i < distances.size(); i++) {
		if (distances[i] < tolerance) {
			retvalue++;
		}
	}

	return retvalue;
}

PointStruct& OSIPoints::GetPoint(int i) {
	if (point_.size() <= i || point_.size() == 0) {
		throw std::runtime_error("OSIPoints::GetPoint(int i) -> exceeds index");
	} else if (i < 0) {
		throw std::runtime_error("OSIPoints::GetXFromIdx(int i) -> index must be larger than 0");
	} else {
		return point_[i];
	}
}

double OSIPoints::GetXfromIdx(int i) {
	if (point_.size() <= i || point_.size() == 0) {
		throw std::runtime_error("OSIPoints::GetXFromIdx(int i) -> exceeds index");
	} else if (i < 0) {
		throw std::runtime_error("OSIPoints::GetXFromIdx(int i) -> index must be larger than 0");
	} else {
		return point_[i].x;
	}
}

double OSIPoints::GetYfromIdx(int i) {
	if (point_.size() <= i || point_.size() == 0) {
		throw std::runtime_error("OSIPoints::GetYFromIdx(int i) -> exceeds index");
	} else if (i < 0) {
		throw std::runtime_error("OSIPoints::GetYFromIdx(int i) -> index must be larger than 0");
	} else {
		return point_[i].y;
	}
}

double OSIPoints::GetZfromIdx(int i) {
	if (point_.size() <= i || point_.size() == 0) {
		throw std::runtime_error("OSIPoints::GetZFromIdx(int i) -> exceeds index");
	} else if (i < 0) {
		throw std::runtime_error("OSIPoints::GetZFromIdx(int i) -> index must be larger than 0");
	} else {
		return point_[i].z;
	}
}

int OSIPoints::GetNumOfOSIPoints() {
	return (int)point_.size();
}

double OSIPoints::GetLength() {
	double length = 0;
	for (int i = 0; i < point_.size() - 1; i++) {
		length += PointDistance2D(point_[i].x, point_[i].y, point_[i + 1].x, point_[i + 1].y);
	}
	return length;
}

void LaneBoundaryOSI::SetGlobalId() {
	global_id_ = GetNewGlobalLaneBoundaryId();
}
