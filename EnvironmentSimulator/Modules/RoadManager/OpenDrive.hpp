#ifndef OPENDRIVE_HPP
#define OPENDRIVE_HPP
#include "RoadManager.hpp"
#include <memory>
// #include "Road.hpp"
// #include "Junction.hpp"
// #include "Controller.hpp"
// #include "Userdata.hpp"
/**
 * @brief  This class has as responsibility to read and parse opendrive files. this includes populate subclasses and give access to them 
 * Should be easy to inherit for other classes that want to add top level functionality or lower class level functionality. 
 * 
 */
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

	/**
		Initialize the global ids 
	*/ 
	// void InitGlobalLaneIds(); should not be in this class

	/**
		Get the filename of currently loaded OpenDRIVE file
	*/
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
	Road* GetRoadById(int id);
    
	/**
		Retrieve a road segment specified by road vector element index
		useful for iterating over all available road segments, e.g:
		for (int i = 0; i < GetNumOfRoads(); i++)
		{
			int n_lanes = GetRoadyIdx(i)->GetNumberOfLanes();
		...
		@param idx index into the vector of roads
	*/
	Road* GetRoadByIdx(int idx);
	// if this is added this class will be huge becouse you will need to have getters for all subclass private variabels
    // Geometry* GetGeometryByIdx(int road_idx, int geom_idx);
    int GetRoadIdxById(int id);
	int GetTrackIdByIdx(int idx);
	int GetNumOfRoads() { return (int)road_.size(); }
	Junction* GetJunctionById(int id);
	Junction* GetJunctionByIdx(int idx);

	int GetNumOfJunctions() { return (int)junction_.size(); }

	bool IsIndirectlyConnected(int road1_id,
							   int road2_id,
							   int*& connecting_road_id,
							   int*& connecting_lane_id,
							   int lane1_id = 0,
							   int lane2_id = 0);

	/**
		Add any missing connections so that road connectivity is two-ways
		Look at all road connections, and make sure they are defined both ways
		@param idx index into the vector of roads
		@return number of added connections
	*/
	int CheckConnections();
	int CheckLink(Road* road, RoadLink* link, ContactPointType expected_contact_point_type);
	int CheckConnectedRoad(Road* road,
						   RoadLink* link,
						   ContactPointType expected_contact_point_type,
						   RoadLink* link2);
	int CheckJunctionConnection(Junction* junction, Connection* connection);
	static std::string ContactPointType2Str(ContactPointType type);
	static std::string ElementType2Str(RoadLink::ElementType type);

	int GetNumberOfControllers() { return (int)controller_.size(); }
	Controller* GetControllerByIdx(int index);
	Controller* GetControllerById(int id);
	void AddController(Controller controller) { controller_.push_back(controller); }

	GeoReference* GetGeoReference();
	std::string GetGeoReferenceAsString();
	void ParseGeoLocalization(const std::string& geoLocalization);

	bool LoadSignalsByCountry(const std::string& country);

	void Print();

	void Save(const std::string fileName) const;

	OpenDriveHeader GetHeader() const { return header_; };
	void AddUserData(UserData* userData) { user_data_.push_back(userData); }

   protected:
    // implement share pointers we have , I am guessing we are leaking memory
    // classes that inherits opendriveparser should have a easy way to access subclasses
    std::vector<std::shared_ptr<Road>> road_;
	std::vector<std::shared_ptr<Junction>> junction_;
	std::vector<std::shared_ptr<Controler>> controller_;
	std::vector<std::shared_ptr<UserData>> user_data_;
	std::map<std::string, std::string> signals_types_;
    GeoReference geo_ref_;	// TODO: Remove this and use the header container instead.

   private:
	pugi::xml_node root_node_;
	OpenDriveHeader header_;
	std::string odr_filename_;
};
#endif