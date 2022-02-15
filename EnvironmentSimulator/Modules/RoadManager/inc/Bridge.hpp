#ifndef BRIDGE_HPP
#define BRIDGE_HPP

#include <cassert>
#include <string>
#include "RoadObject.hpp"
#include "pugixml.hpp"
class Bridge : public RoadObject {
   public:
	enum Type { CONCRETE, STEEL, BRICK, WOOD, UNKNOWN };

	Bridge(double s, double length, std::string name, int id, Type type)
		: s_(s), length_(length), name_(name), id_(id), type_(type) {}

	virtual ~Bridge() = default;

	double GetS() const { return s_; }
	double GetLength() const { return length_; }
	int GetId() const { return id_; }
	std::string GetName() const { return name_; }
	Type GetType() const { return type_; }

	void Save(pugi::xml_node& objects);

   private:
	double s_;
	double length_;
	std::string name_;
	Type type_;
	int id_;
};
#endif