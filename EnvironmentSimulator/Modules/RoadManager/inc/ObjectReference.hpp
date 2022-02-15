#ifndef OBJECTREFERENCE_HPP
#define OBJECTREFERENCE_HPP

#include "RoadObject.hpp"
#include "pugixml.hpp"

class ObjectReference : public RoadObject {
   public:
	ObjectReference(double s, double t, int id, double zOffset, double validLength, Orientation orientation)
		: s_(s), t_(t), id_(id), z_offset_(zOffset), valid_length_(validLength), orientation_(orientation) {}

	virtual ~ObjectReference() = default;

	double GetS() const { return s_; }
	double GetT() const { return t_; }
	int GetId() const { return id_; }
	double GetZOffset() const { return z_offset_; }
	double GetValidLength() const { return valid_length_; }
	Orientation GetOrientation() const { return orientation_; }

	void Save(pugi::xml_node& objects);

   private:
	double s_;
	double t_;
	int id_;
	double z_offset_;
	double valid_length_;
	Orientation orientation_;
};
#endif
