
#ifndef USERDATA_HPP
#define USERDATA_HPP

#include <string>
#include "pugixml.hpp"
class UserData {
   public:
	UserData(std::string code, std::string value, pugi::xml_node userDataNode) : code_(code), value_(value) {
		origin_node_ = pugi::xml_document();
		origin_node_.append_copy(userDataNode);
	}

	UserData(pugi::xml_node userDataNode) {
		code_ = userDataNode.attribute("code").as_string();
		value_ = userDataNode.attribute("value").as_string();
		origin_node_ = pugi::xml_document();
		origin_node_.append_copy(userDataNode);
	}

	std::string GetCode() const { return code_; }
	std::string GetValue() const { return value_; }

	void Save(pugi::xml_node& parent);

   private:
	std::string code_;
	std::string value_;
	pugi::xml_document origin_node_;
};
#endif