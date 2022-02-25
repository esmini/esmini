#pragma once
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>
#include "Bridge.hpp"
#include "CommonMini.hpp"
#include "Controller.hpp"
#include "Junction.hpp"
#include "Outline.hpp"
#include "RMObject.hpp"
#include "Road.hpp"
#include "Userdata.hpp"
#include "pugixml.hpp"
#include "Signal.hpp"

#define ROADMARK_WIDTH_STANDARD 0.15
#define ROADMARK_WIDTH_BOLD 0.20
/**
 * @brief  This class has as responsibility to read and parse opendrive files(check that is accurate). this
 * includes populate subclasses and give access to them Should be easy to inherit for other classes that want
 * to add top level functionality or lower class level functionality.
 */
struct OpenDriveOffset {
	OpenDriveOffset() : x_(0), y_(0), z_(0), hdg_(0){};
	double x_;
	double y_;
	double z_;
	double hdg_;

	std::vector<std::shared_ptr<UserData>> user_data_;
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	void Save(pugi::xml_node&) const;
	bool isValid() const { return x_ || y_ || z_ || hdg_; };
};

struct OpenDriveHeader {
	void Save(pugi::xml_node&) const;
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }

	uint revMajor_;
	uint revMinor_;
	std::string name_;
	float version_;
	std::string date_;
	double north_;
	double south_;
	double east_;
	double west_;
	std::string vendor_;
	OpenDriveOffset offset_;
	GeoReference georeference_;
	std::vector<std::shared_ptr<UserData>> user_data_;
};

class OpenDrive {
   public:
	OpenDrive(){};
	OpenDrive(const char* filename);
	~OpenDrive();

	/**
		Load a road network, specified in the OpenDRIVE file format
		@param filename OpenDRIVE file
		@param replace If true any old road data will be erased, else new will be added to the old
	*/
	bool LoadOpenDriveFile(const char* filename, bool replace = true);

	std::string GetOpenDriveFilename() { return odr_filename_; }

	/**
		Setting information based on the OSI standards for OpenDrive elements
	*/

	// This is OSI stuff should be moved seperated move to osi class that inhereits odr class
	// bool SetRoadOSI();
	// bool CheckLaneOSIRequirement(std::vector<double> x0,
	//  std::vector<double> y0,
	//  std::vector<double> x1,
	//  std::vector<double> y1);
	// void SetLaneOSIPoints();
	// void SetRoadMarkOSIPoints();

	/**
		Checks all lanes - if a lane has RoadMarks it does nothing. If a lane does not have roadmarks
		then it creates a LaneBoundary following the lane border (left border for left lanes, right border for
	   right lanes)
	*/
	// void SetLaneBoundaryPoints(); check for osi stuff move

	/**
		Retrieve a road segment specified by road ID
		@param id road ID as specified in the OpenDRIVE file
	*/
	std::shared_ptr<Road> GetRoadById(int id);

	/**
		Retrieve a road segment specified by road vector element index
		useful for iterating over all available road segments, e.g:
		for (int i = 0; i < GetNumOfRoads(); i++)
		{
			int n_lanes = GetRoadyIdx(i)->GetNumberOfLanes();
		...
		@param idx index into the vector of roads
	*/
	std::shared_ptr<Road> GetRoadByIdx(int idx);
	// if this is added this class will be huge becouse you will need to have getters for all subclass private
	// variabels Geometry* GetGeometryByIdx(int road_idx, int geom_idx);
	int GetRoadIdxById(int id);
	int GetRoadIdByIdx(int idx);
	int GetNumOfRoads() { return (int)road_.size(); }
	// Return the whole protected subtag vector instead of index and one object, if you don't want to
	// inherit(recommend), and do multiple manipulations on multiple roads at the smae time.
	std::vector<std::shared_ptr<Road>> getRoadVector() { return road_; }
	std::vector<std::shared_ptr<Junction>> getJunctionsVector() { return junction_; }
	std::vector<Controller> getControllerVector() { return controller_; }
	std::vector<std::shared_ptr<UserData>> getUserDataVector() { return user_data_; }

	std::shared_ptr<Junction> GetJunctionById(int id);
	std::shared_ptr<Junction> GetJunctionByIdx(int idx);

	int GetNumOfJunctions() { return (int)junction_.size(); }

	//  Move to other class
	// bool IsIndirectlyConnected(int road1_id,
	//    int road2_id,
	//    int*& connecting_road_id,
	//    int*& connecting_lane_id,
	//    int lane1_id = 0,
	//    int lane2_id = 0);
	/**
		Add any missing connections so that road connectivity is two-ways
		Look at all road connections, and make sure they are defined both ways
		@param idx index into the vector of roads
		@return number of added connections
	*/
	int CheckConnections();
	int CheckLink(std::shared_ptr<Road> road,
				  std::shared_ptr<RoadLink> link,
				  ContactPointType expected_contact_point_type);
	int CheckConnectedRoad(std::shared_ptr<Road> road,
						   std::shared_ptr<RoadLink> link,
						   ContactPointType expected_contact_point_type,
						   std::shared_ptr<RoadLink> link2);
	int CheckJunctionConnection(std::shared_ptr<Junction> junction, std::shared_ptr<Connection> connection);
	static std::string ContactPointType2Str(ContactPointType type);
	static std::string ElementType2Str(RoadLink::ElementType type);

	int GetNumberOfControllers() { return (int)controller_.size(); }
	std::shared_ptr<Controller> GetControllerByIdx(int index);
	std::shared_ptr<Controller> GetControllerById(int id);
	void AddController(Controller controller) { controller_.push_back(controller); }

	std::shared_ptr<GeoReference> GetGeoReference();
	std::string GetGeoReferenceAsString();
	void ParseGeoLocalization(const std::string& geoLocalization);

	bool LoadSignalsByCountry(const std::string& country);

	void Print();

	void Save(const std::string fileName) const;

	OpenDriveHeader GetHeader() const { return header_; };
	void AddUserData(std::shared_ptr<UserData> userData) { user_data_.push_back(userData); }
	/**
		Initialize the global ids
	*/
	// void InitGlobalLaneIds(); should not be in this class

	/**
		Get the filename of currently loaded OpenDRIVE file
	*/
   protected:
	// implement share pointers we have , I am guessing we are leaking memory
	// classes that inherits opendriveparser should have a easy way to access subclasses
	std::vector<std::shared_ptr<Road>> road_;			// TODO check if it can be unique ptr
	std::vector<std::shared_ptr<Junction>> junction_;	// TODO check if it can be unique ptr
	std::vector<Controller> controller_;				// TODO check if it can be unique ptr
	std::vector<std::shared_ptr<UserData>> user_data_;	// TODO check if it can be unique ptr
	std::map<std::string, std::string> signals_types_;	// TODO check if it can be unique ptr
	GeoReference geo_ref_;	// TODO: Remove this and use the header container instead.

   private:
	pugi::xml_node root_node_;
	OpenDriveHeader header_;
	std::string odr_filename_;
};


