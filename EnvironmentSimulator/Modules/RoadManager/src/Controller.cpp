#include "Controller.hpp"

void Controller::Save(pugi::xml_node& root) {
	auto controller = root.append_child("controller");
	controller.append_attribute("id").set_value(id_);
	controller.append_attribute("name").set_value(name_.c_str());
	controller.append_attribute("sequence").set_value(sequence_);

	for (auto control : control_) {
		control.Save(controller);
	}

	for (auto userData : user_data_) {
		userData->Save(controller);
	}
}

void Control::Save(pugi::xml_node& controller) {
	auto control = controller.append_child("control");
	control.append_attribute("signalId").set_value(signalId_);
	control.append_attribute("type").set_value(type_.c_str());
	for (auto userData : user_data_) {
		userData->Save(control);
	}
}
