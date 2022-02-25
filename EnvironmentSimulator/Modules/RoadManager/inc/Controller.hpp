#pragma once

#include <memory>
#include <vector>
#include "Userdata.hpp"

typedef struct {
	std::vector<std::shared_ptr<UserData>> user_data_;
	int signalId_;
	std::string type_;
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&);
} Control;

class Controller {
   public:
	Controller() : id_(0), name_(""), sequence_(0) {}
	Controller(int id, std::string name, int sequence = 0) : id_(id), name_(name), sequence_(sequence) {}

	void AddControl(Control ctrl) { control_.push_back(ctrl); }
	int GetNumberOfControls() { return (int)control_.size(); }
	Control* GetControl(int index) {
		return ((index >= 0 && index < control_.size()) ? &control_[index] : nullptr);
	}

	int GetId() { return id_; }
	std::string GetName() { return name_; }
	int GetSequence() { return sequence_; }

	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node& root);
	std::vector<std::shared_ptr<UserData>> getUserDataVector() { return user_data_; }

   protected:
	std::vector<std::shared_ptr<UserData>> user_data_;
	std::vector<Control> control_;

   private:
	int id_;
	std::string name_;
	int sequence_;
};