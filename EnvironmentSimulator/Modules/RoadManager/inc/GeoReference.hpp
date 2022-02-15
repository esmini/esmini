#ifndef GEOREFERENCE_HPP
#define GEOREFERENCE_HPP

#include <memory>
#include <string>
#include <vector>
#include "Userdata.hpp"
#include "pugixml.hpp"

struct GeoReference {
	double a_;
	double axis_;
	double b_;
	std::string ellps_;
	double k_;
	double k_0_;
	double lat_0_;
	double lon_0_;
	double lon_wrap_;
	double over_;
	std::string pm_;
	std::string proj_;
	std::string units_;
	std::string vunits_;
	double x_0_;
	double y_0_;
	std::string datum_;
	std::string geo_id_grids_;
	double zone_;
	int towgs84_;
	std::vector<std::shared_ptr<UserData>> user_data_;

	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&) const;
};
#endif