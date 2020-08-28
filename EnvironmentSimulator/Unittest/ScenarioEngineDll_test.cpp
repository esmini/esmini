#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "osi_common.pb.h"
#include "osi_object.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_version.pb.h"
#include "scenarioenginedll.hpp"
#include <vector>
#include <stdexcept>
#include <fstream>

class GetNumberOfObjectsTest :public ::testing::TestWithParam<std::tuple<std::string,int>> {};
// inp: scenario file
// expected: number of objects in the scenario

TEST_P(GetNumberOfObjectsTest, number_of_objects) {

	std::string scenario_file = std::get<0>(GetParam()); 
	//std::string scenario_file = "../../esmini/resources/xosc/cut-in.xosc"; 
	
	const char * Scenario_file = scenario_file.c_str();
	SE_Init(Scenario_file, 1, 0, 1, 0, 0);

	int n_Objects = SE_GetNumberOfObjects();	
	SE_Close();	

    EXPECT_EQ(n_Objects, std::get<1>(GetParam()));    
    //EXPECT_EQ(n_Objects, 2);    
}

INSTANTIATE_TEST_CASE_P(EsminiAPITests,GetNumberOfObjectsTest,::testing::Values(
    std::make_tuple("../../../resources/xosc/cut-in.xosc", 2),
    std::make_tuple("../../../resources/xosc/highway_merge.xosc", 6), 
	std::make_tuple("../../../resources/xosc/full_e6mini.xosc", 14)));



TEST(GetNumberOfObjectsTest, number_of_objects_no_init) {

	int n_Objects = SE_GetNumberOfObjects();	

    EXPECT_EQ(n_Objects, 0);       
}



TEST(GetOSILaneBoundaryIdsTest, lane_boundary_ids) {

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc"; 
	//std::string scenario_file = std::get<0>(GetParam()); 
	const char * Scenario_file = scenario_file.c_str();
	SE_Init(Scenario_file, 1, 0, 1, 0, 0);

	int n_Objects = SE_GetNumberOfObjects();
	//std::cout << "NUMBER OBJECTS IS " << n_Objects << std::endl; 	
	
	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView(); 
	
	std::vector<int> lane_bound = {-1, 8, 9, 10, 0, 1, 2, 3, 11, 4, 5, 6, 7, 12, 13, 14, -1}; 
	for (int i=0; i<n_Objects; i++)
	{
		SE_LaneBoundaryId right_lanes_id;
		right_lanes_id.far_left_lb_id = lane_bound[lane_bound.size() - 4 - i];
		right_lanes_id.left_lb_id = lane_bound[lane_bound.size() - 3 - i];
		right_lanes_id.right_lb_id = lane_bound[lane_bound.size() - 2 - i];
		right_lanes_id.far_right_lb_id = lane_bound[lane_bound.size() - 1 - i];
		SE_LaneBoundaryId ids;

		SE_GetOSILaneBoundaryIds(i, &ids);

		EXPECT_EQ(ids.far_left_lb_id, right_lanes_id.far_left_lb_id);
		EXPECT_EQ(ids.far_right_lb_id, right_lanes_id.far_right_lb_id);
		EXPECT_EQ(ids.left_lb_id, right_lanes_id.left_lb_id);
		EXPECT_EQ(ids.right_lb_id, right_lanes_id.right_lb_id);
	}

	SE_Close();
}



TEST(GetOSILaneBoundaryIdsTest, lane_boundary_ids_no_obj) {

	std::string scenario_file = "../../../resources/xosc/cut-in.xosc";  
	const char * Scenario_file = scenario_file.c_str();
	SE_Init(Scenario_file, 1, 0, 1, 0, 0);	
	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView(); 

	SE_LaneBoundaryId ids;
	SE_LaneBoundaryId right_lanes_id = { -1, -1, -1, -1 };

	SE_GetOSILaneBoundaryIds(10, &ids);
	EXPECT_EQ(ids.far_left_lb_id, right_lanes_id.far_left_lb_id);
	EXPECT_EQ(ids.far_right_lb_id, right_lanes_id.far_right_lb_id);
	EXPECT_EQ(ids.left_lb_id, right_lanes_id.left_lb_id);
	EXPECT_EQ(ids.right_lb_id, right_lanes_id.right_lb_id);

	SE_Close();
}


TEST(GetOSIRoadLaneTest, lane_no_obj) {

	std::string scenario_file = "../../../resources/xosc/cut-in.xosc";  
	const char * Scenario_file = scenario_file.c_str();
	
	SE_Init(Scenario_file, 1, 0, 1, 0, 0);	
	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView();   	 
 
	int road_lane_size; 

	const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, 15);

	EXPECT_EQ(road_lane_size, 0); 

	SE_Close();
}


TEST(GetOSIRoadLaneTest, lane_id) {

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";  
	const char * Scenario_file = scenario_file.c_str();
	
	SE_Init(Scenario_file, 1, 0, 1, 0, 0);	
	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView();   	

	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};  
 
	int obj_id = 0;
	int road_lane_size; 
	osi3::Lane osi_lane; 

	for (int i=0; i<lanes.size(); i++)
	{		
		int lane_id = lanes[14-i]; 
		if (lane_id == 7) // no vehicle in central lane
		{
			continue;
		}

		const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, obj_id);
		osi_lane.ParseFromArray(road_lane, road_lane_size);

		EXPECT_EQ(osi_lane.id().value(), lane_id); 
		obj_id++; 			
	}
	SE_Close();
}




TEST(GetOSIRoadLaneTest, left_lane_id) {

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";  
	const char * Scenario_file = scenario_file.c_str();
	SE_Init(Scenario_file, 1, 0, 1, 0, 0);

	int n_Objects = SE_GetNumberOfObjects();	
	
	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView();  
	int road_lane_size; 
	osi3::Lane osi_lane;

	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future 
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};  
 
	int obj_id = 0; 

	for (int i=0; i<lanes.size(); i++)
	{
		 
		int lane_id = lanes[14-i]; 
		if (lane_id == 7) // no vehicle in central lane
		{
			continue;
		}

		std::vector<int>::const_iterator last_left = lanes.end()-i-1;
		std::vector<int>::const_iterator first_left = lanes.begin();
		std::vector<int> left_lanes_id(first_left, last_left); 
		std::reverse(left_lanes_id.begin(),left_lanes_id.end()); 
		

		const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, obj_id);
		osi_lane.ParseFromArray(road_lane, road_lane_size);

		for (int j = 0; j<osi_lane.classification().left_adjacent_lane_id_size(); j++)
		{
		  	EXPECT_EQ(osi_lane.classification().left_adjacent_lane_id(j).value(), left_lanes_id[j]); 
		}	
 
		obj_id++; 
		
	}

	SE_Close();
}




TEST(GetOSIRoadLaneTest, right_lane_id) {

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";  
	const char * Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 1, 0, 1, 0, 0);
	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView();  
	int road_lane_size; 
	osi3::Lane osi_lane;
 
	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future 
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};  

	int obj_id = 0; 

	for (int i=0; i<lanes.size(); i++)
	{
		int lane_id = lanes[14-i]; 
		if (lane_id == 7) // no vehicle in central lane
		{
			continue;
		}

		std::vector<int>::const_iterator last_right = lanes.end();
		std::vector<int>::const_iterator first_right = lanes.end()-i;
		std::vector<int> right_lanes_id(first_right, last_right); 		

		const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, obj_id);
		osi_lane.ParseFromArray(road_lane, road_lane_size);


		for (int j = 0; j<osi_lane.classification().right_adjacent_lane_id_size(); j++)
		{
			//std::cout << "RIGHT LANE ID " << osi_lane.classification().right_adjacent_lane_id(j).value() << std::endl;
		  	EXPECT_EQ(osi_lane.classification().right_adjacent_lane_id(j).value(), right_lanes_id[j]); 
		}

		obj_id++; 	
		
	}

	SE_Close();
}


TEST(GetOSIRoadLaneTest, right_lane_boundary_id) {

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";  
	const char * Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 1, 0, 1, 0, 0);
	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView();  
	int road_lane_size; 
	osi3::Lane osi_lane;
 
	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future 
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};  
	std::vector<int> lane_bound = {-1, 8, 9, 10, 0, 1, 2, 3, 11, 4, 5, 6, 7, 12, 13, 14, -1}; 

	int obj_id = 0; 
	int ii = 0; 

	for (int i=0; i<lanes.size(); i++)
	{
		int lane_id = lanes[14-i]; 
		if (lane_id == 7) // no vehicle in central lane
		{
			continue;
		}
		 
		size_t n_lb = lane_bound.size(); 
		size_t lb_id = lane_bound[n_lb-2-ii]; 		

		const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, obj_id);
		osi_lane.ParseFromArray(road_lane, road_lane_size);

		EXPECT_EQ(osi_lane.classification().right_lane_boundary_id(0).value(), lb_id); 

		obj_id++; 
		ii++; 
		
	}

	SE_Close();
}



TEST(GetOSIRoadLaneTest, left_lane_boundary_id) {

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";  
	const char * Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 1, 0, 1, 0, 0);
	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView();  
	int road_lane_size; 
	osi3::Lane osi_lane;
 
	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future 
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};  
	std::vector<int> lane_bound = {-1, 8, 9, 10, 0, 1, 2, 3, 11, 4, 5, 6, 7, 12, 13, 14, -1}; 

	int obj_id = 0; 
	int ii = 0; 

	for (int i=0; i<lanes.size(); i++)
	{
		int lane_id = lanes[14-i]; 
		if (lane_id == 7) // no vehicle in central lane
		{
			continue;
		}
		size_t n_lb = lane_bound.size(); 
		size_t lb_id = lane_bound[n_lb-3-ii];

		const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, obj_id);
		osi_lane.ParseFromArray(road_lane, road_lane_size);

		EXPECT_EQ(osi_lane.classification().left_lane_boundary_id(0).value(), lb_id);	

		obj_id++; 
		ii++; 
		
	}

	SE_Close();
}


class GetOSIRoadLaneTest :public ::testing::TestWithParam<std::tuple<std::string,bool, bool>> {};
// inp: scenario file
// expected: bool defining if driving direction is the same of road definition

TEST_P(GetOSIRoadLaneTest, centerline_is_driving_direction) {

	std::string scenario_file = std::get<0>(GetParam());   
	const char * Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 1, 0, 1, 0, 0);
	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView();  

	int road_lane_size; 
	osi3::Lane osi_lane; 
	int obj_id = 0; 
 
	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future 
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};  

	for (int i=0; i<lanes.size(); i++)
	{
		int lane_id = lanes[14-i]; 
		if (lane_id == 7) // no vehicle in central lane
		{
			continue;
		}	

		const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, obj_id);
		osi_lane.ParseFromArray(road_lane, road_lane_size);
		if (i <= 7)
		{
			EXPECT_EQ(osi_lane.classification().centerline_is_driving_direction(), std::get<1>(GetParam()) );
		}
		else
		{
			EXPECT_EQ(osi_lane.classification().centerline_is_driving_direction(), std::get<2>(GetParam()) );
		}			

		obj_id++; 
	}

	SE_Close();
}


INSTANTIATE_TEST_CASE_P(EsminiAPITests,GetOSIRoadLaneTest,::testing::Values(
    std::make_tuple("../../../resources/xosc/full_e6mini.xosc", true, false ),
    std::make_tuple("../../../resources/xosc/full_e6mini_reverse.xosc", true, false )));





TEST(GetOSIRoadLaneTest, is_host_vehicle_lane) {

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";  
	const char * Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 1, 0, 1, 0, 0);
	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView();  
	int road_lane_size; 
	osi3::Lane osi_lane;
 
	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future 
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};   

	int obj_id = 0; 

	for (int i=0; i<lanes.size(); i++)
	{
		int lane_id = lanes[14-i]; 
		if (lane_id == 7) // no vehicle in central lane
		{
			continue;
		}

		const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, obj_id);
		osi_lane.ParseFromArray(road_lane, road_lane_size);

		EXPECT_EQ(osi_lane.classification().is_host_vehicle_lane(), false);	

		obj_id++; 
		
	}

	SE_Close();
}


TEST(GetOSIRoadLaneTest, lane_classification) {

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";  
	const char * Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 1, 0, 1, 0, 0);
	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView();  
	int road_lane_size; 
	osi3::Lane osi_lane;
 
	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future 
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};   

	int obj_id = 0; 

	for (int i=0; i<lanes.size(); i++)
	{
		int lane_id = lanes[14-i]; 
		if (lane_id == 7) // no vehicle in central lane
		{
			continue;
		}

		const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, obj_id);
		osi_lane.ParseFromArray(road_lane, road_lane_size);

		osi3::Lane_Classification_Type lane_type = osi_lane.classification().type();  
 
		if (lane_id == 3 || lane_id == 4 || lane_id == 5 || lane_id == 9 ||lane_id == 10 ||lane_id == 11 )
		{
			EXPECT_EQ(lane_type , osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING);	
		}	
		else
		{
			EXPECT_EQ(lane_type , osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING);
		}

		obj_id++; 
		
	}

	SE_Close();
}


TEST(GetOSILaneBoundaryTests, lane_boundary_id_existing) {

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";  
	const char * Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 1, 0, 1, 0, 0);
	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView();  
	int lb_size; 
	osi3::LaneBoundary osi_lb;
 
	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future 
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14}; 
	std::vector<int> lane_bound = {-1, 8, 9, 10, 0, 1, 2, 3, 11, 4, 5, 6, 7, 12, 13, 14, -1};   

	int obj_id = 0; 

	for (int i=0; i<lane_bound.size(); i++)
	{
		int lb_global_id = lane_bound[i]; 
		if (lb_global_id == -1)
		{
			continue; 
		}
		const char* lb = SE_GetOSILaneBoundary(&lb_size, lb_global_id);
		osi_lb.ParseFromArray(lb, lb_size);

		EXPECT_EQ(osi_lb.id().value(), lb_global_id); 
		
	}

	SE_Close();
}



class GetOSILaneBoundaryTests :public ::testing::TestWithParam<std::tuple<int, int>> {};
// inp:  excisting lane boundary global id 
// expected: size of osi lane boundary message = 0

TEST_P(GetOSILaneBoundaryTests, lane_boundary_id_not_existing) {

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";  
	const char * Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 1, 0, 1, 0, 0);
	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView();  
	int lb_size = 0; 

	const char* lb = SE_GetOSILaneBoundary(&lb_size, std::get<0>(GetParam()) );

	EXPECT_EQ(lb_size, std::get<1>(GetParam()) ); 

	SE_Close();
}


INSTANTIATE_TEST_CASE_P(EsminiAPITests,GetOSILaneBoundaryTests,::testing::Values(
    std::make_tuple(15, 0 ),
    std::make_tuple(-15, 0 )));



TEST(OSIFile, writeosifile_two_step) {

	std::string scenario_file = "../../../resources/xosc/cut-in.xosc";  
	const char * Scenario_file = scenario_file.c_str();
	std::streamoff file_size1, file_size2, file_sizeend;

	SE_Init(Scenario_file, 1, 0, 1, 0, 0);

	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView(); 	
	SE_OSIFileOpen(); 
	SE_OSIFileWrite();

	std::ifstream in_file("move_obj.osi", std::ios::binary);
   	in_file.seekg(0, std::ios::end);
   	file_size1 = in_file.tellg();
   	std::cout <<"Size of the file at first step "<< file_size1 << " bytes" << std::endl; 


	SE_StepDT(0.001f);		
	SE_UpdateOSISensorView(); 	 
	SE_OSIFileWrite(); 

	in_file.seekg(0, std::ios::end);
   	file_size2 = in_file.tellg();
   	std::cout <<"Size of the file at second step "<< file_size2 << " bytes" << std::endl; 

	SE_Close();

	in_file.seekg(0, std::ios::end);
   	file_sizeend = in_file.tellg();
   	std::cout <<"Size of closing step "<< file_sizeend << " bytes" << std::endl; 
	
//	EXPECT_EQ(file_size2, file_sizeend);  // File might not be flushed until it's closed, unless it is done explicitly
	EXPECT_LT(file_size1, file_size2); 
}

TEST(OSIFile, writeosifile_no_init) {

	bool open = SE_OSIFileOpen(); 
	bool write = SE_OSIFileWrite();

	EXPECT_EQ(open, false);
	EXPECT_EQ(write, false);	 
}

class GetSensorViewTests :public ::testing::TestWithParam<std::tuple<std::string,int>> {};
// inp: nto excisting lane boundary global id 
// expected: size of osi lane boundary message = 0

TEST_P(GetSensorViewTests, receive_SensorView) {

	std::string scenario_file = std::get<0>(GetParam());  
	const char * Scenario_file = scenario_file.c_str();
	int sv_size = 0; 
	osi3::SensorView osi_sv;

	SE_Init(Scenario_file, 1, 0, 1, 0, 0);

	//SE_OSIFileOpen(); 

	SE_StepDT(0.001f);	

	SE_UpdateOSISensorView(); 

	const char* sv = SE_GetOSISensorView(&sv_size);
	osi_sv.ParseFromArray(sv, sv_size);	

	int n_lanes = osi_sv.mutable_global_ground_truth()->lane_size();

	EXPECT_EQ(n_lanes, std::get<1>(GetParam())); 
 
}

INSTANTIATE_TEST_CASE_P(EsminiAPITests,GetSensorViewTests,::testing::Values(
    std::make_tuple("../../../resources/xosc/cut-in.xosc", 15 ),
    std::make_tuple("../../../resources/xosc/straight_500m.xosc", 7 ),
    std::make_tuple("../../../resources/xosc/highway_merge.xosc", 40 )));



TEST(GetSensorViewTests, receive_SensorView_no_init) {

	int sv_size = 0; 
	osi3::SensorView osi_sv; 

	const char* sv = SE_GetOSISensorView(&sv_size);

	EXPECT_EQ(sv_size, 0);  
}




int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}