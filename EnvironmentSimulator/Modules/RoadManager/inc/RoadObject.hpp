#ifndef ROADOBJECT_HPP
#define ROADOBJECT_HPP

#include <memory>
#include <vector>
#include "StructsandDefines.hpp"
#include "Userdata.hpp"

class RoadObject {
   public:
	enum Orientation {
		POSITIVE,
		NEGATIVE,
		NONE,
	};
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	std::vector<ValidityRecord> validity_;

   private:
	std::vector<std::shared_ptr<UserData>> user_data_;
};

#endif