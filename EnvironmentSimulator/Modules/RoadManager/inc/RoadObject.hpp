#pragma once

#include <memory>
#include <vector>
#include "Userdata.hpp"

typedef struct {
	int fromLane_;
	int toLane_;
	std::vector<std::shared_ptr<UserData>> user_data_;
	void Save(pugi::xml_node& object);
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
} ValidityRecord;


class RoadObject {
   public:
	enum Orientation {
		POSITIVE,
		NEGATIVE,
		NONE,
	};
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	std::vector<ValidityRecord> validity_;
	std::vector<std::shared_ptr<UserData>> getUserDataVector() { return user_data_; }

   protected:
	std::vector<std::shared_ptr<UserData>> user_data_;
};
