#ifndef LANELINK_HPP
#define LANELINK_HPP

#include <memory>
#include <vector>
#include "StructsandDefines.hpp"
#include "Userdata.hpp"
#include "pugixml.hpp"

class LaneLink {
   public:
	LaneLink(LinkType type, int id) : type_(type), id_(id) {}

	LinkType GetType() { return type_; }
	int GetId() { return id_; }
	void Print();
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&);

   protected:
	std::vector<std::shared_ptr<UserData>> user_data_;

   private:
	LinkType type_;
	int id_;
};
#endif