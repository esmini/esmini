#ifndef LANEMATERIAL_HPP
#define LANEMATERIAL_HPP

#include <memory>
#include <string>
#include <vector>
#include "Userdata.hpp"
#include "pugixml.hpp"
class LaneMaterial {
   public:
	LaneMaterial(double sOffset, std::string surface, double friction, double roughness)
		: s_offset_(sOffset), friction_(friction), roughness_(roughness), surface_(surface){};

	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node& lane);

   protected:
	std::vector<std::shared_ptr<UserData>> user_data_;

   private:
	double s_offset_;
	std::string surface_;
	double friction_;
	double roughness_;
};

#endif