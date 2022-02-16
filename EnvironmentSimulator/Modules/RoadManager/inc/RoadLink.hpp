#pragma once

#include <cassert>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "CommonMini.hpp"
#include "StructsandDefines.hpp"
#include "Userdata.hpp"
#include "pugixml.hpp"
class RoadLink {
   public:
	typedef enum {
		ELEMENT_TYPE_UNKNOWN,
		ELEMENT_TYPE_ROAD,
		ELEMENT_TYPE_JUNCTION,
	} ElementType;

	RoadLink()
		: type_(NONE),
		  element_id_(-1),
		  element_type_(ELEMENT_TYPE_UNKNOWN),
		  contact_point_type_(CONTACT_POINT_UNDEFINED) {}
	RoadLink(LinkType type, ElementType element_type, int element_id, ContactPointType contact_point_type)
		: type_(type),
		  element_id_(element_id),
		  element_type_(element_type),
		  contact_point_type_(contact_point_type) {}
	RoadLink(LinkType type, pugi::xml_node node);
	bool operator==(RoadLink& rhs);

	int GetElementId() { return element_id_; }
	LinkType GetType() { return type_; }
	RoadLink::ElementType GetElementType() { return element_type_; }
	ContactPointType GetContactPointType() { return contact_point_type_; }

	void Print();
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&);

   protected:
	std::vector<std::shared_ptr<UserData>> user_data_;	// TODO check if I can use unique pointers

   private:
	LinkType type_;
	int element_id_;
	ElementType element_type_;
	ContactPointType contact_point_type_;
};