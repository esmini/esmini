#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "osi_common.pb.h"
#include "osi_object.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_version.pb.h"
#include "esminiLib.hpp"
#include "RoadManager.hpp"
#include <vector>
#include <stdexcept>
#include <fstream>

#define _USE_MATH_DEFINES
#include <math.h>

class GetNumberOfObjectsTest : public ::testing::TestWithParam<std::tuple<std::string, int>>
{
};
// inp: scenario file
// expected: number of objects in the scenario

TEST_P(GetNumberOfObjectsTest, number_of_objects)
{
	std::string scenario_file = std::get<0>(GetParam());
	//std::string scenario_file = "../../esmini/resources/xosc/cut-in.xosc";

	const char *Scenario_file = scenario_file.c_str();
	SE_Init(Scenario_file, 0, 0, 0, 0);

	int n_Objects = SE_GetNumberOfObjects();
	SE_Close();

	EXPECT_EQ(n_Objects, std::get<1>(GetParam()));
	//EXPECT_EQ(n_Objects, 2);
}

INSTANTIATE_TEST_SUITE_P(EsminiAPITests, GetNumberOfObjectsTest, ::testing::Values(std::make_tuple("../../../resources/xosc/cut-in.xosc", 2), std::make_tuple("../../../resources/xosc/highway_merge.xosc", 6), std::make_tuple("../../../resources/xosc/full_e6mini.xosc", 15)));

TEST(GetNumberOfObjectsTest, number_of_objects_no_init)
{
	int n_Objects = SE_GetNumberOfObjects();

	EXPECT_EQ(n_Objects, -1);
}

// OSI tests

#ifdef _USE_OSI

TEST(GetOSILaneBoundaryIdsTest, lane_boundary_ids)
{

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";

	SE_Init(scenario_file.c_str(), 0, 0, 0, 0);

	int n_Objects = SE_GetNumberOfObjects();
	EXPECT_EQ(n_Objects, 15);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();

	std::vector<std::vector<int>> lane_bound = {{-1, 8, 9, 10},
												{8, 9, 10, 0},
												{9, 10, 0, 1},
												{10, 0, 1, 2},
												{0, 1, 2, 3},
												{1, 2, 3, 11},
												{2, 3, 11, 4},
												{3, 11, 4, 5}, //right side start
												{11, 4, 5, 6},
												{4, 5, 6, 7},
												{5, 6, 7, 12},
												{6, 7, 12, 13},
												{7, 12, 13, 14},
												{12, 13, 14, -1}};

	std::vector<int> veh_id = {14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 2, 1, 0};
	for (int i = 0; i < lane_bound.size(); i++)
	{
		SE_LaneBoundaryId lanes_id;
		lanes_id.far_left_lb_id = lane_bound[i][0];
		lanes_id.left_lb_id = lane_bound[i][1];
		lanes_id.right_lb_id = lane_bound[i][2];
		lanes_id.far_right_lb_id = lane_bound[i][3];
		SE_LaneBoundaryId ids;

		SE_GetOSILaneBoundaryIds(veh_id[i], &ids);

		EXPECT_EQ(ids.far_left_lb_id, lanes_id.far_left_lb_id);
		EXPECT_EQ(ids.left_lb_id, lanes_id.left_lb_id);
		EXPECT_EQ(ids.right_lb_id, lanes_id.right_lb_id);
		EXPECT_EQ(ids.far_right_lb_id, lanes_id.far_right_lb_id);
	}

	SE_Close();
}

TEST(GetOSILaneBoundaryIdsTest, lane_boundary_ids_no_obj)
{

	std::string scenario_file = "../../../resources/xosc/cut-in.xosc";
	const char *Scenario_file = scenario_file.c_str();
	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();

	SE_LaneBoundaryId ids;
	SE_LaneBoundaryId right_lanes_id = {-1, -1, -1, -1};

	SE_GetOSILaneBoundaryIds(10, &ids);
	EXPECT_EQ(ids.far_left_lb_id, right_lanes_id.far_left_lb_id);
	EXPECT_EQ(ids.far_right_lb_id, right_lanes_id.far_right_lb_id);
	EXPECT_EQ(ids.left_lb_id, right_lanes_id.left_lb_id);
	EXPECT_EQ(ids.right_lb_id, right_lanes_id.right_lb_id);

	SE_Close();
}

TEST(OSIintersections, threeway)
{

	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/simple_3_way_intersection_osi.xosc";
	const char *Scenario_file = scenario_file.c_str();
	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	int lanes_found = 0;
	bool intersection_found = false;
	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->type() == osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION)
		{
			intersection_found = true;
			// should not exist a centerline
			ASSERT_EQ(osi_gt.mutable_lane(i)->mutable_classification()->centerline_size(), 0);
			ASSERT_EQ(osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(), 6);
			ASSERT_EQ(osi_gt.mutable_lane(i)->mutable_classification()->free_lane_boundary_id_size(), 3);
		}
		else
		{
			lanes_found++;
		}
	}
	ASSERT_TRUE(intersection_found);
	ASSERT_EQ(lanes_found, 6);
	SE_Close();
}

TEST(OSIintersections, fourway)
{

	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/simple_4_way_intersection_osi.xosc";
	const char *Scenario_file = scenario_file.c_str();
	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	int lanes_found = 0;
	bool intersection_found = false;
	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->type() == osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION)
		{
			intersection_found = true;
			// should not exist a centerline
			ASSERT_EQ(osi_gt.mutable_lane(i)->mutable_classification()->centerline_size(), 0);
			ASSERT_EQ(osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(), 12);
			ASSERT_EQ(osi_gt.mutable_lane(i)->mutable_classification()->free_lane_boundary_id_size(), 4);
		}
		else
		{
			lanes_found++;
		}
	}
	ASSERT_TRUE(intersection_found);
	ASSERT_EQ(lanes_found, 8);
	SE_Close();
}

TEST(OSIintersections, motorway)
{

	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/simple_motorway_osi_intersection.xosc";
	const char *Scenario_file = scenario_file.c_str();
	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	int lanes_found = 0;
	bool intersection_found = false;
	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->type() == osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION)
		{
			intersection_found = true;
		}
		else
		{
			lanes_found++;
		}
	}
	ASSERT_FALSE(intersection_found);
	ASSERT_EQ(lanes_found, 12);
	SE_Close();
}

TEST(OSIintersections, multilane)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/multilane_3way_intersection_osi.xosc";
	const char *Scenario_file = scenario_file.c_str();
	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	int lanes_found = 0;
	bool intersection_found = false;
	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->type() == osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_INTERSECTION)
		{
			intersection_found = true;
			// should not exist a centerline
			ASSERT_EQ(osi_gt.mutable_lane(i)->mutable_classification()->centerline_size(), 0);
			ASSERT_EQ(osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(), 18);
			ASSERT_EQ(osi_gt.mutable_lane(i)->mutable_classification()->free_lane_boundary_id_size(), 3);
		}
		else
		{
			lanes_found++;
		}
	}
	ASSERT_TRUE(intersection_found);
	ASSERT_EQ(lanes_found, 18);

	SE_Close();
}

TEST(GetOSIRoadLaneTest, lane_no_obj)
{

	std::string scenario_file = "../../../resources/xosc/cut-in.xosc";
	const char *Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();

	int road_lane_size;

	const char *road_lane = SE_GetOSIRoadLane(&road_lane_size, 15);

	EXPECT_EQ(road_lane_size, 0);
	EXPECT_EQ(road_lane, nullptr);

	SE_Close();
}

TEST(GetOSIRoadLaneTest, lane_id)
{

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";
	const char *Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();

	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
	std::vector<int> veh_id = {14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 2, 1, 0};

	int road_lane_size;
	osi3::Lane osi_lane;

	for (int i = 0; i < lanes.size(); i++)
	{
		int lane_id = lanes[i];

		const char *road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
		osi_lane.ParseFromArray(road_lane, road_lane_size);

		EXPECT_EQ(osi_lane.id().value(), lane_id);
	}
	SE_Close();
}

TEST(GetOSIRoadLaneTest, left_lane_id)
{

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";
	const char *Scenario_file = scenario_file.c_str();
	SE_Init(Scenario_file, 0, 0, 0, 0);

	int n_Objects = SE_GetNumberOfObjects();
	EXPECT_EQ(n_Objects, 15);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	int road_lane_size;
	osi3::Lane osi_lane;

	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
	std::vector<int> veh_id = {14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 2, 1, 0};

	for (int i = 0; i < lanes.size(); i++)
	{
		const char *road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
		osi_lane.ParseFromArray(road_lane, road_lane_size);
		if (lanes[i] == 0)
		{
			EXPECT_EQ(osi_lane.classification().left_adjacent_lane_id_size(), 0);
		}
		else if (lanes[i] <= 6)
		{
			EXPECT_EQ(osi_lane.classification().left_adjacent_lane_id(0).value(), lanes[i - 1]);
		}
		else if (lanes[i] == 8)
		{
			EXPECT_EQ(osi_lane.classification().left_adjacent_lane_id(0).value(), 6);
		}
		else
		{
			EXPECT_EQ(osi_lane.classification().left_adjacent_lane_id(0).value(), lanes[i - 1]);
		}
	}

	SE_Close();
}

TEST(GetOSIRoadLaneTest, right_lane_id)
{

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";
	const char *Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	int road_lane_size;
	osi3::Lane osi_lane;

	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
	std::vector<int> veh_id = {14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 2, 1, 0};

	for (int i = 0; i < lanes.size(); i++)
	{
		const char *road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
		osi_lane.ParseFromArray(road_lane, road_lane_size);

		if (lanes[i] == 14)
		{
			EXPECT_EQ(osi_lane.classification().right_adjacent_lane_id_size(), 0);
		}
		else if (lanes[i] < 6)
		{
			EXPECT_EQ(osi_lane.classification().right_adjacent_lane_id(0).value(), lanes[i + 1]);
		}
		else if (lanes[i] == 6)
		{
			EXPECT_EQ(osi_lane.classification().right_adjacent_lane_id(0).value(), 8);
		}
		else
		{
			EXPECT_EQ(osi_lane.classification().right_adjacent_lane_id(0).value(), lanes[i + 1]);
		}
	}

	SE_Close();
}

TEST(GetOSIRoadLaneTest, right_lane_boundary_id)
{

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";
	const char *Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	int road_lane_size;
	osi3::Lane osi_lane;

	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
	std::vector<int> lane_bound = {8, 9, 10, 0, 1, 2, 3, 11, 4, 5, 6, 7, 12, 13, 14};
	std::vector<int> veh_id = {14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 2, 1, 0};

	for (int i = 0; i < lane_bound.size() - 1; i++)
	{
		const char *road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
		osi_lane.ParseFromArray(road_lane, road_lane_size);

		EXPECT_EQ(osi_lane.classification().right_lane_boundary_id(0).value(), lane_bound[i + 1]);
	}

	SE_Close();
}

TEST(GetOSIRoadLaneTest, left_lane_boundary_id)
{

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";
	const char *Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	int road_lane_size;
	osi3::Lane osi_lane;

	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
	std::vector<int> lane_bound = {8, 9, 10, 0, 1, 2, 3, 11, 4, 5, 6, 7, 12, 13, 14};
	std::vector<int> veh_id = {14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 2, 1, 0};

	for (int i = 0; i < veh_id.size(); i++)
	{
		const char *road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
		osi_lane.ParseFromArray(road_lane, road_lane_size);

		EXPECT_EQ(osi_lane.classification().left_lane_boundary_id(0).value(), lane_bound[i]);
	}

	SE_Close();
}

class GetOSIRoadLaneTest : public ::testing::TestWithParam<std::tuple<std::string, bool, bool>>
{
};
// inp: scenario file
// expected: bool defining if driving direction is the same of road definition

TEST_P(GetOSIRoadLaneTest, centerline_is_driving_direction)
{

	std::string scenario_file = std::get<0>(GetParam());
	const char *Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();

	int road_lane_size;
	osi3::Lane osi_lane;

	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
	std::vector<int> veh_id = {14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 2, 1, 0};

	for (int i = 0; i < lanes.size(); i++)
	{
		const char *road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
		osi_lane.ParseFromArray(road_lane, road_lane_size);
		if (veh_id[i] <= 7)
		{
			EXPECT_EQ(osi_lane.classification().centerline_is_driving_direction(), std::get<1>(GetParam()));
		}
		else
		{
			EXPECT_EQ(osi_lane.classification().centerline_is_driving_direction(), std::get<2>(GetParam()));
		}
	}

	SE_Close();
}

INSTANTIATE_TEST_SUITE_P(EsminiAPITests, GetOSIRoadLaneTest, ::testing::Values(std::make_tuple("../../../resources/xosc/full_e6mini.xosc", true, false), std::make_tuple("../../../resources/xosc/full_e6mini_reverse.xosc", true, false)));

TEST(GetOSIRoadLaneTest, is_host_vehicle_lane)
{

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";
	const char *Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	int road_lane_size;
	osi3::Lane osi_lane;

	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
	std::vector<int> veh_id = {14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 2, 1, 0};

	for (int i = 0; i < lanes.size(); i++)
	{
		const char *road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
		osi_lane.ParseFromArray(road_lane, road_lane_size);

		EXPECT_EQ(osi_lane.classification().is_host_vehicle_lane(), false);
	}

	SE_Close();
}

TEST(GetOSIRoadLaneTest, lane_classification)
{

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";
	const char *Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	int road_lane_size;
	osi3::Lane osi_lane;

	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
	std::vector<int> veh_id = {14, 13, 12, 11, 10, 9, 8, 6, 5, 4, 3, 2, 1, 0};

	int obj_id = 0;

	for (int i = 0; i < lanes.size(); i++)
	{
		int lane_id = lanes[i];

		const char *road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
		osi_lane.ParseFromArray(road_lane, road_lane_size);

		osi3::Lane_Classification_Type lane_type = osi_lane.classification().type();

		if (lane_id == 3 || lane_id == 4 || lane_id == 5 || lane_id == 9 || lane_id == 10 || lane_id == 11)
		{
			EXPECT_EQ(lane_type, osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_DRIVING);
		}
		else
		{
			EXPECT_EQ(lane_type, osi3::Lane_Classification_Type::Lane_Classification_Type_TYPE_NONDRIVING);
		}

		obj_id++;
	}

	SE_Close();
}

TEST(GetOSILaneBoundaryTests, lane_boundary_id_existing)
{

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";
	const char *Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	int lb_size;
	osi3::LaneBoundary osi_lb;

	// explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
	std::vector<int> lanes = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
	std::vector<int> lane_bound = {-1, 8, 9, 10, 0, 1, 2, 3, 11, 4, 5, 6, 7, 12, 13, 14, -1};

	for (int i = 0; i < lane_bound.size(); i++)
	{
		int lb_global_id = lane_bound[i];
		if (lb_global_id == -1)
		{
			continue;
		}
		const char *lb = SE_GetOSILaneBoundary(&lb_size, lb_global_id);
		osi_lb.ParseFromArray(lb, lb_size);

		EXPECT_EQ(osi_lb.id().value(), lb_global_id);
	}

	SE_Close();
}

class GetOSILaneBoundaryTests : public ::testing::TestWithParam<std::tuple<int, int>>
{
};
// inp:  excisting lane boundary global id
// expected: size of osi lane boundary message = 0

TEST_P(GetOSILaneBoundaryTests, lane_boundary_id_not_existing)
{

	std::string scenario_file = "../../../resources/xosc/full_e6mini.xosc";
	const char *Scenario_file = scenario_file.c_str();

	SE_Init(Scenario_file, 0, 0, 0, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	int lb_size = 0;

	const char *lb = SE_GetOSILaneBoundary(&lb_size, std::get<0>(GetParam()));

	EXPECT_EQ(lb_size, std::get<1>(GetParam()));
	EXPECT_EQ(lb, nullptr);

	SE_Close();
}

INSTANTIATE_TEST_SUITE_P(EsminiAPITests, GetOSILaneBoundaryTests, ::testing::Values(std::make_tuple(15, 0), std::make_tuple(-15, 0)));

TEST(OSIFile, writeosifile_two_step)
{

	std::string scenario_file = "../../../resources/xosc/cut-in.xosc";
	const char *Scenario_file = scenario_file.c_str();
	std::streamoff file_size1, file_size2, file_sizeend;

	SE_Init(Scenario_file, 0, 0, 0, 0);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	SE_OSIFileOpen(0);
	SE_OSIFileWrite(true);

	std::ifstream in_file("ground_truth.osi", std::ios::binary);
	in_file.seekg(0, std::ios::end);
	file_size1 = in_file.tellg();
	//std::cout <<"Size of the file at first step "<< file_size1 << " bytes" << std::endl;

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	SE_OSIFileWrite(true);

	in_file.seekg(0, std::ios::end);
	file_size2 = in_file.tellg();
	//std::cout <<"Size of the file at second step "<< file_size2 << " bytes" << std::endl;

	SE_Close();

	in_file.seekg(0, std::ios::end);
	file_sizeend = in_file.tellg();
	//std::cout <<"Size of closing step "<< file_sizeend << " bytes" << std::endl;

	//	EXPECT_EQ(file_size2, file_sizeend);  // File might not be flushed until it's closed, unless it is done explicitly
	EXPECT_LT(file_size1, file_size2);
}

TEST(OSIFile, writeosifile_no_init)
{

	bool open = SE_OSIFileOpen(0);
	bool write = SE_OSIFileWrite();

	EXPECT_EQ(open, false);
	EXPECT_EQ(write, false);
}

typedef struct
{
	float length;
	float width;
	float height;
	float centerOffsetX;
	float centerOffsetY;
	float centerOffsetZ;
} bounding_box;

class GetGroundTruthTests : public ::testing::TestWithParam<std::tuple<std::string, int, int, bounding_box, std::string>>
{
};
// inp: nto excisting lane boundary global id
// expected: size of osi lane boundary message = 0

TEST_P(GetGroundTruthTests, receive_GroundTruth)
{

	std::string scenario_file = std::get<0>(GetParam());
	const char *Scenario_file = scenario_file.c_str();
	int sv_size = 0;
	osi3::GroundTruth osi_gt;

	SE_Init(Scenario_file, 0, 0, 0, 0);

	//SE_OSIFileOpen(0);

	SE_StepDT(0.001f);

	SE_UpdateOSIGroundTruth();

	const char *sv = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(sv, sv_size);

	int n_lanes = osi_gt.lane_size();

	int n_objects = osi_gt.mutable_moving_object()->size();

	int ego_index = 0; // ego vehicle are always first in tested scenarios

	float ego_length = (float)osi_gt.mutable_moving_object(ego_index)->mutable_base()->mutable_dimension()->length();
	float ego_width = (float)osi_gt.mutable_moving_object(ego_index)->mutable_base()->mutable_dimension()->width();
	float ego_height = (float)osi_gt.mutable_moving_object(ego_index)->mutable_base()->mutable_dimension()->height();
	float ego_xoffset = (float)osi_gt.mutable_moving_object(ego_index)->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->x();
	float ego_yoffset = (float)osi_gt.mutable_moving_object(ego_index)->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->y();
	float ego_zoffset = (float)osi_gt.mutable_moving_object(ego_index)->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->z();
	std::string map_reference = osi_gt.map_reference();

	EXPECT_EQ(n_lanes, std::get<1>(GetParam()));
	EXPECT_EQ(n_objects, std::get<2>(GetParam()));
	EXPECT_EQ(ego_length, std::get<3>(GetParam()).length);
	EXPECT_EQ(ego_width, std::get<3>(GetParam()).width);
	EXPECT_EQ(ego_height, std::get<3>(GetParam()).height);
	EXPECT_EQ(ego_xoffset, std::get<3>(GetParam()).centerOffsetX);
	EXPECT_EQ(ego_yoffset, std::get<3>(GetParam()).centerOffsetY);
	EXPECT_EQ(ego_zoffset, std::get<3>(GetParam()).centerOffsetZ);
	EXPECT_EQ(map_reference, std::get<4>(GetParam()));
}

INSTANTIATE_TEST_SUITE_P(EsminiAPITests, GetGroundTruthTests, ::testing::Values(std::make_tuple("../../../resources/xosc/cut-in.xosc", 14, 2, bounding_box{5.04f, 2.0f, 1.5f, 1.4f, 0.0f, 0.75f}, "+proj=utm +lat_0=37.3542934123933 +lon_0=-122.0859797650754"), std::make_tuple("../../../resources/xosc/straight_500m.xosc", 6, 2, bounding_box{5.0f, 2.0f, 1.8f, 1.4f, 0.0f, 0.9f}, "+proj=utm +lat_0=37.3542934123933 +lon_0=-122.0859797650754"), std::make_tuple("../../../resources/xosc/highway_merge.xosc", 33, 6, bounding_box{5.04f, 2.0f, 1.5f, 1.4f, 0.0f, 0.75f}, "+proj=utm +lat_0=37.3542934123933 +lon_0=-122.0859797650754")));
// scenario_file_name, number_of_lanes, number_of_objects, ego_bounding_box

TEST(GetGroundTruthTests, receive_GroundTruth_no_init)
{

	int sv_size = 0;
	osi3::GroundTruth osi_gt;

	const char *sv = SE_GetOSIGroundTruth(&sv_size);

	EXPECT_EQ(sv_size, 0);
	EXPECT_EQ(sv, nullptr);
}

TEST(GetMiscObjFromGroundTruth, receive_miscobj)
{

	int sv_size = 0;
	osi3::GroundTruth osi_gt;

	SE_Init("../../../EnvironmentSimulator/Unittest/xosc/miscobj_basic.xosc", 0, 0, 0, 0);

	SE_StepDT(0.001f);

	SE_UpdateOSIGroundTruth();

	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);

	int n_miscobjects = osi_gt.mutable_stationary_object()->size();

	uint64_t miscobj_id = osi_gt.mutable_stationary_object(0)->mutable_id()->value();
	osi3::StationaryObject_Classification_Type miscobj_type = osi_gt.mutable_stationary_object(0)->mutable_classification()->type();

	double miscobj_length = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_dimension()->length();
	double miscobj_width = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_dimension()->width();
	double miscobj_height = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_dimension()->height();

	double miscobj_x = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_position()->x();
	double miscobj_y = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_position()->y();
	double miscobj_z = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_position()->z();

	double miscobj_roll = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_orientation()->roll();
	double miscobj_pitch = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_orientation()->pitch();
	double miscobj_yaw = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_orientation()->yaw();

	EXPECT_EQ(n_miscobjects, 1);
	EXPECT_EQ(miscobj_id, 0);

	EXPECT_EQ(miscobj_type, osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_BARRIER);

	EXPECT_EQ(miscobj_length, 200);
	EXPECT_EQ(miscobj_width, 100);
	EXPECT_EQ(miscobj_height, 5);

	EXPECT_EQ(miscobj_x, 10);
	EXPECT_EQ(miscobj_y, 10);
	EXPECT_EQ(miscobj_z, 0.0); // adjusted to the road z

	// Angles in OSI should be in range [-PI, PI]
	EXPECT_EQ(miscobj_roll, 5.0 - 2 * M_PI);  // Aligned to the road (so if road roll is 1.0 total roll will be 6.0)
	EXPECT_EQ(miscobj_pitch, 5.0 - 2 * M_PI); // Aligned to the road (so if road pitch is 1.0 total pitch will be 6.0)
	EXPECT_EQ(miscobj_yaw, 5.0 - 2 * M_PI);
}

TEST(SetOSITimestampTest, TestGetAndSet)
{
	osi3::GroundTruth *osi_gt;

	std::string scenario_file = "../../../resources/xosc/cut-in.xosc";

	EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);

	int n_Objects = SE_GetNumberOfObjects();
	EXPECT_EQ(n_Objects, 2);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();

	osi_gt = (osi3::GroundTruth *)SE_GetOSIGroundTruthRaw();

	EXPECT_EQ(osi_gt->mutable_moving_object()->size(), 2);

	double seconds = osi_gt->mutable_timestamp()->seconds() + 1E-9 * osi_gt->mutable_timestamp()->nanos();
	EXPECT_DOUBLE_EQ(seconds, 0.001);

	SE_OSISetTimeStamp(1234543210);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	EXPECT_EQ(osi_gt->mutable_timestamp()->seconds(), 1);
	EXPECT_EQ(osi_gt->mutable_timestamp()->nanos(), (unsigned int)234543210);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	// Expect no change as timestamp has been set explicitly only once
	EXPECT_EQ(osi_gt->mutable_timestamp()->seconds(), 1);
	EXPECT_EQ(osi_gt->mutable_timestamp()->nanos(), (unsigned int)234543210);

	SE_OSISetTimeStamp(5234543229);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	// Expect updated timestamp
	EXPECT_EQ(osi_gt->mutable_timestamp()->seconds(), 5);
	EXPECT_EQ(osi_gt->mutable_timestamp()->nanos(), (unsigned int)234543229);

	SE_Close();
}

TEST(ReportObjectAcc, TestGetAndSet)
{
	osi3::GroundTruth *osi_gt;

	std::string scenario_file = "../../../resources/xosc/cut-in_simple.xosc";

	EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);

	int n_Objects = SE_GetNumberOfObjects();
	EXPECT_EQ(n_Objects, 2);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();

	osi_gt = (osi3::GroundTruth *)SE_GetOSIGroundTruthRaw();

	EXPECT_EQ(osi_gt->mutable_moving_object()->size(), 2);

	double seconds = osi_gt->mutable_timestamp()->seconds() + 1E-9 * osi_gt->mutable_timestamp()->nanos();
	EXPECT_DOUBLE_EQ(seconds, 0.001);

	SE_ReportObjectAcc(0, 0, 1, 2, 3);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	EXPECT_EQ(osi_gt->mutable_moving_object(0)->mutable_base()->mutable_acceleration()->x(), 1.0);

	SE_ReportObjectAcc(0, 0, 4, 1, 8);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	EXPECT_EQ(osi_gt->mutable_moving_object(0)->mutable_base()->mutable_acceleration()->x(), 4.0);
	EXPECT_EQ(osi_gt->mutable_moving_object(0)->mutable_base()->mutable_acceleration()->y(), 1.0);
	EXPECT_EQ(osi_gt->mutable_moving_object(0)->mutable_base()->mutable_acceleration()->z(), 8.0);

	SE_ReportObjectAngularAcc(1, 0, 5, 4, 3);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	EXPECT_EQ(osi_gt->mutable_moving_object(1)->mutable_base()->mutable_orientation_acceleration()->yaw(), 5.0);
	EXPECT_EQ(osi_gt->mutable_moving_object(1)->mutable_base()->mutable_orientation_acceleration()->pitch(), 4.0);
	EXPECT_EQ(osi_gt->mutable_moving_object(1)->mutable_base()->mutable_orientation_acceleration()->roll(), 3.0);

	SE_Close();
}

TEST(ReportObjectVel, TestGetAndSet)
{
	osi3::GroundTruth *osi_gt;

	std::string scenario_file = "../../../resources/xosc/cut-in_simple.xosc";

	EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);

	int n_Objects = SE_GetNumberOfObjects();
	EXPECT_EQ(n_Objects, 2);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();

	osi_gt = (osi3::GroundTruth *)SE_GetOSIGroundTruthRaw();

	EXPECT_EQ(osi_gt->mutable_moving_object()->size(), 2);

	double seconds = osi_gt->mutable_timestamp()->seconds() + 1E-9 * osi_gt->mutable_timestamp()->nanos();
	EXPECT_DOUBLE_EQ(seconds, 0.001);

	SE_ReportObjectVel(0, 0, 11, 12, 13);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	EXPECT_EQ(osi_gt->mutable_moving_object(0)->mutable_base()->mutable_velocity()->x(), 11.0);

	SE_ReportObjectVel(0, 0, 21, 22, 23);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	EXPECT_EQ(osi_gt->mutable_moving_object(0)->mutable_base()->mutable_velocity()->x(), 21.0);
	EXPECT_EQ(osi_gt->mutable_moving_object(0)->mutable_base()->mutable_velocity()->y(), 22.0);
	EXPECT_EQ(osi_gt->mutable_moving_object(0)->mutable_base()->mutable_velocity()->z(), 23.0);

	SE_ReportObjectAngularVel(1, 0, 25, 24, 23);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	EXPECT_EQ(osi_gt->mutable_moving_object(1)->mutable_base()->mutable_orientation_rate()->yaw(), 25.0);
	EXPECT_EQ(osi_gt->mutable_moving_object(1)->mutable_base()->mutable_orientation_rate()->pitch(), 24.0);
	EXPECT_EQ(osi_gt->mutable_moving_object(1)->mutable_base()->mutable_orientation_rate()->roll(), 23.0);

	SE_Close();
}

#endif // _USE_OSI

TEST(ParameterTest, GetTypedParameterValues)
{
	std::string scenario_file = "../../../resources/xosc/lane_change.xosc";
	SE_Init(scenario_file.c_str(), 0, 0, 0, 0);

	bool boolVar;
	int retVal;
	retVal = SE_GetParameterBool("DummyParameter2", &boolVar);
	EXPECT_EQ(retVal, 0);
	EXPECT_EQ(boolVar, true);

	// Use common SE_Parameter class
	SE_Parameter param;
	param.name = "DummyParameter2";
	bool value = false;
	param.value = (void*)&value;
	retVal = SE_GetParameter(&param);
	EXPECT_EQ(retVal, 0);
	value = *(bool*)(param.value);
	EXPECT_EQ(value, true);

	value = false;
	retVal = SE_SetParameter(param);
	EXPECT_EQ(retVal, 0);

	value = true;
	retVal = SE_GetParameter(&param);
	EXPECT_EQ(retVal, 0);
	value = *(bool*)(param.value);
	EXPECT_EQ(value, false);

	// Unavailable
	retVal = SE_GetParameterBool("DoesNotExist", &boolVar);
	EXPECT_EQ(retVal, -1);

	// Set value
	SE_SetParameterBool("DummyParameter2", false);
	retVal = SE_GetParameterBool("DummyParameter2", &boolVar);
	EXPECT_EQ(retVal, 0);
	EXPECT_EQ(boolVar, false);

	// Wrong name
	retVal = SE_SetParameterBool("DummyParameter3", false);
	EXPECT_EQ(retVal, -1);

	// Wrong type
	retVal = SE_SetParameterInt("DummyParameter2", false);
	EXPECT_EQ(retVal, -1);

	// String
	const char *strVar;
	retVal = SE_GetParameterString("DummyParameter3", &strVar);
	EXPECT_EQ(retVal, 0);
	EXPECT_STREQ(strVar, "lane_change_param");

	retVal = SE_SetParameterString("DummyParameter3", "Kalle");
	EXPECT_EQ(retVal, 0);
	retVal = SE_GetParameterString("DummyParameter3", &strVar);
	EXPECT_EQ(retVal, 0);
	EXPECT_STREQ(strVar, "Kalle");

	retVal = SE_GetParameterString("DoesNotExist", &strVar);
	EXPECT_EQ(retVal, -1);

	SE_Close();
}

static void paramDeclCallback(void*)
{
	static int counter = 0;
	double value[2] = { 1.1, 1.5 };

	if (counter < 2)
	{
		SE_SetParameterDouble("TargetSpeedFactor", value[counter]);
	}

	counter++;
}

TEST(ParameterTest, SetParameterValuesBeforeInit)
{
	double positions[3][2] = {
		{5.37060, 189.98206},  // TargetSpeedFactor = 1.1
		{9.09839, 241.55124},  // TargetSpeedFactor = 1.5
		{5.49925, 204.98148}  // TargetSpeedFactor = Default = 1.2
	};
	SE_ScenarioObjectState state;

	std::string scenario_file = "../../../resources/xosc/cut-in.xosc";

	SE_RegisterParameterDeclarationCallback(paramDeclCallback, 0);

	for (int i = 0; i < 3 && SE_GetQuitFlag() != 1; i++)
	{
		ASSERT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);
		ASSERT_EQ(SE_GetNumberOfObjects(), 2);

		while (SE_GetSimulationTime() < 5.0 && SE_GetQuitFlag() != 1)
		{
			SE_StepDT(0.1f);
		}

		// Check position of second vehicle
		SE_GetObjectState(1, &state);
		EXPECT_NEAR(state.x, positions[i][0], 1e-5);
		EXPECT_NEAR(state.y, positions[i][1], 1e-5);

		SE_Close();
	}
}

TEST(OverrideActionTest, TestGetAndSet)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/override_action.xosc";
	const char *Scenario_file = scenario_file.c_str();
	float dt = 0.1f;
	float t = 0.0f;

	SE_OverrideActionList list;

	ASSERT_EQ(SE_Init(Scenario_file, 0, 0, 0, 0), 0);

	EXPECT_EQ(SE_GetOverrideActionStatus(0, &list), 0);
	EXPECT_EQ(list.throttle.active, false);
	EXPECT_DOUBLE_EQ(list.throttle.value, 0.0);

	for (; t < 2.5; t += dt)
	{
		SE_StepDT(dt);
	}
	// Should still be no changes
	EXPECT_EQ(SE_GetOverrideActionStatus(0, &list), 0);
	EXPECT_EQ(list.throttle.active, false);
	EXPECT_DOUBLE_EQ(list.throttle.value, 0.0);

	for (; t < 3.1; t += dt)
	{
		SE_StepDT(dt);
	}
	// Now there should be some settings done
	EXPECT_EQ(SE_GetOverrideActionStatus(0, &list), 0);
	EXPECT_EQ(list.throttle.active, true);
	EXPECT_DOUBLE_EQ(list.throttle.value, 0.5);
	EXPECT_EQ(list.clutch.active, false);
	EXPECT_DOUBLE_EQ(list.clutch.value, 1.0);
	EXPECT_EQ(list.steeringWheel.active, false);
	EXPECT_DOUBLE_EQ(list.steeringWheel.value, 0.0);

	for (; t < 5.1; t += dt)
	{
		SE_StepDT(dt);
	}
	EXPECT_EQ(SE_GetOverrideActionStatus(0, &list), 0);
	EXPECT_EQ(list.throttle.active, true);
	EXPECT_DOUBLE_EQ(list.throttle.value, 0.5);
	EXPECT_EQ(list.clutch.active, true);
	EXPECT_DOUBLE_EQ(list.clutch.value, 0.7);
	EXPECT_EQ(list.steeringWheel.active, false);
	EXPECT_NEAR(list.steeringWheel.value, 2 * M_PI, 0.01);
}

TEST(PropertyTest, TestGetAndSet)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/VehiclePropertyTest.xosc";
	const char *Scenario_file = scenario_file.c_str();
	SE_Init(Scenario_file, 0, 0, 0, 0);

	//object 0, 3 properties
	EXPECT_EQ(SE_GetNumberOfProperties(0), 3);
	//    <Property name="model_id" value="1"/>
	//    <Property name="dummy0_1" value="2"/>
	//    <Property name="dummy0_2" value="3"/>
	EXPECT_STREQ(SE_GetObjectPropertyName(0, 0), "model_id");
	EXPECT_STREQ(SE_GetObjectPropertyName(0, 1), "dummy0_1");
	EXPECT_STREQ(SE_GetObjectPropertyName(0, 2), "dummy0_2");
	EXPECT_STREQ(SE_GetObjectPropertyValue(0, "model_id"), "1");
	EXPECT_STREQ(SE_GetObjectPropertyValue(0, "dummy0_1"), "2");
	EXPECT_STREQ(SE_GetObjectPropertyValue(0, "dummy0_2"), "3");
	//object 1, 4 properties
	//    <Property name="model_id" value="1"/>
	//    <Property name="dummy1_1" value="2"/>
	//    <Property name="dummy1_2" value="3"/>
	//    <Property name="dummy1_3" value="4"/>
	EXPECT_EQ(SE_GetNumberOfProperties(1), 4);
	EXPECT_STREQ(SE_GetObjectPropertyName(1, 0), "model_id");
	EXPECT_STREQ(SE_GetObjectPropertyName(1, 1), "dummy1_1");
	EXPECT_STREQ(SE_GetObjectPropertyName(1, 2), "dummy1_2");
	EXPECT_STREQ(SE_GetObjectPropertyName(1, 3), "dummy1_3");
	// propertyIndex exceeds both limits
	EXPECT_STREQ(SE_GetObjectPropertyName(1, -1), "");
	EXPECT_STREQ(SE_GetObjectPropertyName(1, 4), "");
	// propertyName does not exist
	EXPECT_STREQ(SE_GetObjectPropertyValue(1, "superdummy"), "");

	//object 2, 0 properties
	EXPECT_EQ(SE_GetNumberOfProperties(2), 0);
	EXPECT_STREQ(SE_GetObjectPropertyName(2, 0), "");
	EXPECT_STREQ(SE_GetObjectPropertyValue(2, "model_id"), "");

	//object -1 and 3, does not exist
	EXPECT_EQ(SE_GetNumberOfProperties(3), -1);
	EXPECT_EQ(SE_GetNumberOfProperties(-1), -1);
}

TEST(RoadSign, TestValidityRecord)
{
	std::string scenario_file = "../../../resources/xosc/distance_test.xosc";
	EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);

	int n_Objects = SE_GetNumberOfObjects();
	EXPECT_EQ(n_Objects, 2);

	EXPECT_EQ(SE_GetNumberOfRoadSigns(1), 12);
	EXPECT_EQ(SE_GetNumberOfRoadSignValidityRecords(1, 0), 2);

	SE_RoadObjValidity validityRec;

	EXPECT_EQ(SE_GetRoadSignValidityRecord(1, 0, 0, &validityRec), 0);
	EXPECT_EQ(validityRec.fromLane, -3);
	EXPECT_EQ(validityRec.toLane, -1);

	EXPECT_EQ(SE_GetRoadSignValidityRecord(1, 0, 1, &validityRec), 0);
	EXPECT_EQ(validityRec.fromLane, 1);
	EXPECT_EQ(validityRec.toLane, 3);

	EXPECT_EQ(SE_GetNumberOfRoadSignValidityRecords(1, 9), 3);

	EXPECT_EQ(SE_GetRoadSignValidityRecord(1, 9, 0, &validityRec), 0);
	EXPECT_EQ(validityRec.fromLane, -3);
	EXPECT_EQ(validityRec.toLane, -2);

	EXPECT_EQ(SE_GetRoadSignValidityRecord(1, 9, 1, &validityRec), 0);
	EXPECT_EQ(validityRec.fromLane, 1);
	EXPECT_EQ(validityRec.toLane, 2);

	EXPECT_EQ(SE_GetRoadSignValidityRecord(1, 9, 2, &validityRec), 0);
	EXPECT_EQ(validityRec.fromLane, 3);
	EXPECT_EQ(validityRec.toLane, 3);

	SE_Close();
}

TEST(OSILaneParing, multi_roads)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/consecutive_roads.xosc";
	const char *Scenario_file = scenario_file.c_str();
	int i_init = SE_Init(Scenario_file, 0, 0, 0, 0);
	ASSERT_EQ(i_init, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();

	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);

	// order: lane, predecessor, successor
	std::vector<std::vector<int>> lane_pairs = {{0, -1, 3},
												{2, -1, 5},
												{3, 0, 6},
												{5, 2, 8},
												{6, 3, 9},
												{8, 5, 11},
												{9, 6, -1},
												{11, 8, -1}};
	int successor;
	int predecessor;
	int gt_successor = -1;
	int gt_predecessor = -1;
	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
		{
			ASSERT_TRUE(false);
		}
		// std::cout << i << std::endl;
		// ASSERT_EQ(osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(),1);
		for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
		{
			predecessor = -1;
			successor = -1;
			for (int k = 0; k < lane_pairs.size(); k++)
			{
				if (osi_gt.mutable_lane(i)->id().value() == lane_pairs[k][0])
				{
					predecessor = lane_pairs[k][1];
					successor = lane_pairs[k][2];
				}
			}

			if (successor == -1 && predecessor == -1)
			{
				ASSERT_EQ(true, false);
			}
			if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
			{
				gt_successor = (int)osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value();
			}
			else
			{
				gt_successor = -1;
			}
			if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
			{
				gt_predecessor = (int)osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value();
			}
			else
			{
				gt_predecessor = -1;
			}
			ASSERT_EQ(gt_successor, successor);
			ASSERT_EQ(gt_predecessor, predecessor);
		}
	}

	SE_Close();
}

TEST(OSILaneParing, multi_lanesections)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/multi_lanesections.xosc";
	const char *Scenario_file = scenario_file.c_str();
	int i_init = SE_Init(Scenario_file, 0, 0, 0, 0);
	ASSERT_EQ(i_init, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();

	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	// order: lane, predecessor, successor
	std::vector<std::vector<int>> lane_pairs = {{0, -1, 3},
												{2, -1, 6},
												{4, -1, 8},
												{3, 0, 7},
												{6, 2, 10},
												{8, 4, 12},
												{7, 3, 11},
												{10, 6, 14},
												{12, 8, 17},
												{11, 7, 16},
												{14, 10, 19},
												{15, -1, 20},
												{17, 12, -1},
												{16, 11, -1},
												{19, 14, -1},
												{20, 15, -1}};
	int successor;
	int predecessor;
	int gt_successor;
	int gt_predecessor;
	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
		{
			ASSERT_TRUE(false);
		}
		for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
		{
			predecessor = -1;
			successor = -1;
			for (int k = 0; k < lane_pairs.size(); k++)
			{
				if (osi_gt.mutable_lane(i)->id().value() == lane_pairs[k][0])
				{
					predecessor = lane_pairs[k][1];
					successor = lane_pairs[k][2];
				}
			}

			if (successor == -1 && predecessor == -1)
			{
				ASSERT_EQ(true, false);
			}
			if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
			{
				gt_successor = (int)osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value();
			}
			else
			{
				gt_successor = -1;
			}
			if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
			{
				gt_predecessor = (int)osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value();
			}
			else
			{
				gt_predecessor = -1;
			}
			ASSERT_EQ(gt_successor, successor);
			ASSERT_EQ(gt_predecessor, predecessor);
		}
	}
	SE_Close();
}

TEST(OSILaneParing, highway_split)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/highway_split.xosc";
	const char *Scenario_file = scenario_file.c_str();
	int i_init = SE_Init(Scenario_file, 0, 0, 0, 0);
	ASSERT_EQ(i_init, 0);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	// order: lane, predecessor, successor
	std::vector<std::vector<int>> lane_pairs = {{1, -1, 8},
												{2, -1, 10},
												{8, 1, 4},
												{10, 2, 6},
												{4, 8, -1},
												{6, 10, -1}};

	int successor;
	int predecessor;
	int gt_successor;
	int gt_predecessor;

	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
		{
			ASSERT_TRUE(false);
		}
		for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
		{
			predecessor = -1;
			successor = -1;
			for (int k = 0; k < lane_pairs.size(); k++)
			{
				if (osi_gt.mutable_lane(i)->id().value() == lane_pairs[k][0])
				{
					predecessor = lane_pairs[k][1];
					successor = lane_pairs[k][2];
				}
			}

			if (successor == -1 && predecessor == -1)
			{
				ASSERT_EQ(true, false);
			}
			if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
			{
				gt_successor = (int)osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value();
			}
			else
			{
				gt_successor = -1;
			}
			if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
			{
				gt_predecessor = (int)osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value();
			}
			else
			{
				gt_predecessor = -1;
			}
			EXPECT_EQ(gt_successor, successor);
			EXPECT_EQ(gt_predecessor, predecessor);
		}
	}
	SE_Close();
}

//rht split is a merge with lht
TEST(OSILaneParing, highway_merge_lht)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/highway_split_lht.xosc";
	const char *Scenario_file = scenario_file.c_str();
	int i_init = SE_Init(Scenario_file, 0, 0, 0, 0);
	ASSERT_EQ(i_init, 0);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	// order: lane, predecessor, successor
	std::vector<std::vector<int>> lane_pairs = {{1, -1, 8},
												{2, -1, 10},
												{8, 1, 4},
												{10, 2, 6},
												{4, 8, -1},
												{6, 10, -1}};

	int successor;
	int predecessor;
	int gt_successor;
	int gt_predecessor;

	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
		{
			ASSERT_TRUE(false);
		}
		for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
		{
			predecessor = -1;
			successor = -1;
			for (int k = 0; k < lane_pairs.size(); k++)
			{
				if (osi_gt.mutable_lane(i)->id().value() == lane_pairs[k][0])
				{
					predecessor = lane_pairs[k][1];
					successor = lane_pairs[k][2];
				}
			}

			if (successor == -1 && predecessor == -1)
			{
				ASSERT_EQ(true, false);
			}
			if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
			{
				gt_successor = (int)osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value();
			}
			else
			{
				gt_successor = -1;
			}
			if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
			{
				gt_predecessor = (int)osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value();
			}
			else
			{
				gt_predecessor = -1;
			}
			EXPECT_EQ(gt_successor, successor);
			EXPECT_EQ(gt_predecessor, predecessor);
		}
	}
	SE_Close();
}

TEST(OSILaneParing, highway_merge)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/highway_merge.xosc";
	const char *Scenario_file = scenario_file.c_str();
	int i_init = SE_Init(Scenario_file, 0, 0, 0, 0);
	ASSERT_EQ(i_init, 0);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	// order: lane, predecessor, successor
	std::vector<std::vector<int>> lane_pairs = {{0, -1, 11},
												{2, -1, 13},
												{3, -1, 14},
												{5, -1, 16},
												{6, 11, -1},
												{8, 13, -1},
												{9, 14, -1},
												{10, 16, -1},
												{11, 0, 6},
												{13, 2, 8},
												{14, 3, 9},
												{16, 5, 10}};
	int successor;
	int predecessor;
	int gt_successor;
	int gt_predecessor;
	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
		{
			ASSERT_TRUE(false);
		}
		for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
		{
			predecessor = -1;
			successor = -1;
			for (int k = 0; k < lane_pairs.size(); k++)
			{
				if (osi_gt.mutable_lane(i)->id().value() == lane_pairs[k][0])
				{
					predecessor = lane_pairs[k][1];
					successor = lane_pairs[k][2];
				}
			}

			if (successor == -1 && predecessor == -1)
			{
				ASSERT_EQ(true, false);
			}
			if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
			{
				gt_successor = (int)osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value();
			}
			else
			{
				gt_successor = -1;
			}
			if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
			{
				gt_predecessor = (int)osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value();
			}
			else
			{
				gt_predecessor = -1;
			}
			ASSERT_EQ(gt_successor, successor);
			ASSERT_EQ(gt_predecessor, predecessor);
		}
	}
	SE_Close();
}




TEST(OSILaneParing, highway_merge_w_split)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/highway_intersection_test0.xosc";
	const char *Scenario_file = scenario_file.c_str();
	int i_init = SE_Init(Scenario_file, 0, 0, 0, 0);
	ASSERT_EQ(i_init, 0);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	// order: lane, predecessor, successor
	std::vector<std::vector<int>> lane_pairs = {{0, -1, 5},
												{1, -1, 6},
												{3, -1, 8},
												{4, -1, 9},
												{5, 0, 11},
												{6, 1, 12},
												{8, 3, 14},
												{9, 4, 15},
												{10, -1, 16},
												{11, 5, 24},
												{12, 6, 25},
												{14, 8, 27},
												{15, 9, 28},
												{16, 10, 30},
												{24, 11, 17},
												{25, 12, 18},
												{27, 14, 20},
												{28, 15, 21},
												{30, 16, 23},
												{17, 24, -1},
												{18, 25, -1},
												{20, 27, -1},
												{21, 28, -1},
												{23, 30, -1}};
	int successor;
	int predecessor;
	int gt_successor;
	int gt_predecessor;
	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
		{
			ASSERT_TRUE(false);
		}
		for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
		{
			predecessor = -1;
			successor = -1;
			for (int k = 0; k < lane_pairs.size(); k++)
			{
				if (osi_gt.mutable_lane(i)->id().value() == lane_pairs[k][0])
				{
					predecessor = lane_pairs[k][1];
					successor = lane_pairs[k][2];
				}
			}

			if (successor == -1 && predecessor == -1)
			{
				ASSERT_EQ(true, false);
			}
			if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
			{
				gt_successor = (int)osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value();
			}
			else
			{
				gt_successor = -1;
			}
			if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
			{
				gt_predecessor = (int)osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value();
			}
			else
			{
				gt_predecessor = -1;
			}
			EXPECT_EQ(gt_successor, successor);
			EXPECT_EQ(gt_predecessor, predecessor);
		}
	}
	SE_Close();
}

TEST(OSILaneParing, circular_road)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/circular_road.xosc";
	const char *Scenario_file = scenario_file.c_str();
	int i_init = SE_Init(Scenario_file, 0, 0, 0, 0);
	ASSERT_EQ(i_init, 0);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	// order: lane, predecessor, successor
	std::vector<std::vector<int>> lane_pairs = {{0, 5, 11},
												{2, 3, 9},
												{3, 2, 6},
												{5, 0, 8},
												{6, 3, 9},
												{8, 5, 11},
												{9, 6, 2},
												{11, 8, 0}};
	int successor;
	int predecessor;
	int gt_successor;
	int gt_predecessor;
	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
		{
			ASSERT_TRUE(false);
		}
		for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
		{
			predecessor = -1;
			successor = -1;
			for (int k = 0; k < lane_pairs.size(); k++)
			{
				if (osi_gt.mutable_lane(i)->id().value() == lane_pairs[k][0])
				{
					predecessor = lane_pairs[k][1];
					successor = lane_pairs[k][2];
				}
			}

			if (successor == -1 && predecessor == -1)
			{
				ASSERT_EQ(true, false);
			}
			if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
			{
				gt_successor = (int)osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value();
			}
			else
			{
				gt_successor = -1;
			}
			if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
			{
				gt_predecessor = (int)osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value();
			}
			else
			{
				gt_predecessor = -1;
			}
			ASSERT_EQ(gt_successor, successor);
			ASSERT_EQ(gt_predecessor, predecessor);
		}
	}
	SE_Close();
}

TEST(OSILaneParing, simple_3way_intersection)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/simple_3_way_intersection_osi.xosc";
	const char *Scenario_file = scenario_file.c_str();
	int i_init = SE_Init(Scenario_file, 0, 0, 0, 0);
	ASSERT_EQ(i_init, 0);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	// order: lane, predecessor, successor
	std::vector<std::vector<int>> lane_pairs = {
		{0, -1, 18},
		{2, -1, 18},
		{3, 18, -1},
		{5, 18, -1},
		{6, 18, -1},
		{8, 18, -1},
		{18, 3, 0},
		{18, 3, 8},
		{18, 2, 5},
		{18, 2, 8},
		{18, 6, 5},
		{18, 6, 0}
	};

	std::sort(lane_pairs.begin(), lane_pairs.end());

	int gt_successor;
	int gt_predecessor;
	static int counter = 0;
	static int prev_id;

	std::vector<std::vector<int>> gt_lane_pairs;

	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
		{
			ASSERT_TRUE(false);
		}
		for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
		{
			if (osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
			{
				gt_successor = (int)osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value();
			}
			else
			{
				gt_successor = -1;
			}
			if (osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
			{
				gt_predecessor = (int)osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value();
			}
			else
			{
				gt_predecessor = -1;
			}

			gt_lane_pairs.push_back({(int)osi_gt.lane(i).id().value(), gt_predecessor, gt_successor});
		}

	}
	std::sort(gt_lane_pairs.begin(), gt_lane_pairs.end());

	ASSERT_EQ(gt_lane_pairs.size(), lane_pairs.size());
	for (int i = 0; i < lane_pairs.size(); i++)
	{
		ASSERT_EQ(gt_lane_pairs[i][0], lane_pairs[i][0]);
		ASSERT_EQ(gt_lane_pairs[i][1], lane_pairs[i][1]);
		ASSERT_EQ(gt_lane_pairs[i][2], lane_pairs[i][2]);
	}

	SE_Close();
}

TEST(OSILaneParing, simple_3way_intersection_lht)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/simple_3_way_intersection_osi_lht.xosc";
	const char *Scenario_file = scenario_file.c_str();
	int i_init = SE_Init(Scenario_file, 0, 0, 0, 0);
	ASSERT_EQ(i_init, 0);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	// order: lane, predecessor, successor
	std::vector<std::vector<int>> lane_pairs = {
		{0, -1, 18},
		{2, -1, 18},
		{3, 18, -1},
		{5, 18, -1},
		{6, 18, -1},
		{8, 18, -1},
		{18, 0, 3},
		{18, 0, 6},
		{18, 5, 2},
		{18, 5, 6},
		{18, 8, 2},
		{18, 8, 3}
	};

	std::sort(lane_pairs.begin(), lane_pairs.end());

	int gt_successor;
	int gt_predecessor;
	static int counter = 0;
	static int prev_id;

	std::vector<std::vector<int>> gt_lane_pairs;

	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
		{
			ASSERT_TRUE(false);
		}
		for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
		{
			if (osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
			{
				gt_successor = (int)osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value();
			}
			else
			{
				gt_successor = -1;
			}
			if (osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
			{
				gt_predecessor = (int)osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value();
			}
			else
			{
				gt_predecessor = -1;
			}

			gt_lane_pairs.push_back({(int)osi_gt.lane(i).id().value(), gt_predecessor, gt_successor});
		}

	}
	std::sort(gt_lane_pairs.begin(), gt_lane_pairs.end());

	ASSERT_EQ(gt_lane_pairs.size(), lane_pairs.size());
	for (int i = 0; i < lane_pairs.size(); i++)
	{
		ASSERT_EQ(gt_lane_pairs[i][0], lane_pairs[i][0]);
		ASSERT_EQ(gt_lane_pairs[i][1], lane_pairs[i][1]);
		ASSERT_EQ(gt_lane_pairs[i][2], lane_pairs[i][2]);
	}

	SE_Close();
}

TEST(OSILaneParing, simple_4way_intersection)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/simple_4_way_intersection_osi.xosc";
	const char *Scenario_file = scenario_file.c_str();
	int i_init = SE_Init(Scenario_file, 0, 0, 0, 0);
	ASSERT_EQ(i_init, 0);

	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();
	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char *gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	// order: lane, predecessor, successor
	std::vector<std::vector<int>> lane_pairs = {
		{0, -1, 30},
		{2, -1, 30},
		{3, 30, -1},
		{5, 30, -1},
		{6, 30, -1},
		{8, 30, -1},
		{9, 30, -1},
		{11, 30, -1},
		{30, 2, 5},
		{30, 2, 8},
		{30, 2, 11},
		{30, 3, 0},
		{30, 3, 8},
		{30, 3, 11},
		{30, 6, 0},
		{30, 6, 5},
		{30, 6, 11},
		{30, 9, 0},
		{30, 9, 5},
		{30, 9, 8}
	};
	std::sort(lane_pairs.begin(), lane_pairs.end());

	int gt_successor;
	int gt_predecessor;
	static int counter = 0;
	static int prev_id;

	std::vector<std::vector<int>> gt_lane_pairs;

	for (int i = 0; i < osi_gt.lane_size(); i++)
	{
		if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
		{
			ASSERT_TRUE(false);
		}
		for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
		{
			if (osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
			{
				gt_successor = (int)osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value();
			}
			else
			{
				gt_successor = -1;
			}
			if (osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
			{
				gt_predecessor = (int)osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value();
			}
			else
			{
				gt_predecessor = -1;
			}

			gt_lane_pairs.push_back({(int)osi_gt.lane(i).id().value(), gt_predecessor, gt_successor});
		}

	}
	std::sort(gt_lane_pairs.begin(), gt_lane_pairs.end());

	ASSERT_EQ(gt_lane_pairs.size(), lane_pairs.size());
	for (int i = 0; i < lane_pairs.size(); i++)
	{
		ASSERT_EQ(gt_lane_pairs[i][0], lane_pairs[i][0]);
		ASSERT_EQ(gt_lane_pairs[i][1], lane_pairs[i][1]);
		ASSERT_EQ(gt_lane_pairs[i][2], lane_pairs[i][2]);
	}

	SE_Close();
}

TEST(OSILaneParing, Signs)
{
	std::string scenario_file = "../../../resources/xosc/distance_test.xosc";
	const char* Scenario_file = scenario_file.c_str();
	int i_init = SE_Init(Scenario_file, 0, 0, 0, 0);
	ASSERT_EQ(i_init, 0);
	SE_StepDT(0.001f);
	SE_UpdateOSIGroundTruth();

	osi3::GroundTruth osi_gt;
	int sv_size = 0;
	const char* gt = SE_GetOSIGroundTruth(&sv_size);
	osi_gt.ParseFromArray(gt, sv_size);
	// order: id, type, country, subtypevalue, text, pitch, roll, height, s, t, zOffset
	std::vector<std::tuple<int, osi3::TrafficSign_MainSign_Classification_Type, double, std::string, double, double, double, double, double, double>> signs = { std::make_tuple(0, osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_DANGER_SPOT, -1, "", 0.0, 0.0, 0.61, 0.0, 3.57, 1.7),
																																					std::make_tuple(1, osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_DANGER_SPOT, -1, "", 0.0, 0.0, 0.61, 0.0, 3.57, 1.7),
																																					std::make_tuple(2, osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_ZEBRA_CROSSING, -1, "", 0.0, 0.0, 0.61, 100.0, 3.57, 1.7),
																																					std::make_tuple(3, osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN, -1, "", 0.0, 0.0, 0.61, 100.0, 3.57, 1.7),
																																					std::make_tuple(4, osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_HILL_UPWARDS, -1, "", 0.0, 0.0, 0.61, 100.0, 3.57, 1.7),
																																					std::make_tuple(5, osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_DOUBLE_TURN_LEFT, -1, "", 0.0, 0.0, 0.61, 100.0, 3.57, 1.7),
																																					std::make_tuple(6, osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_DOUBLE_TURN_RIGHT, -1, "", 0.0, 0.0, 0.61, 200.0, 3.57, 1.7),
																																					std::make_tuple(7, osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN, -1, "", 0.0, 0.0, 0.61, 200.0, 3.57, 1.7),
																																					std::make_tuple(8, osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN, -1, "", 0.0, 0.0, 0.61, 200.0, 3.57, 1.7),
																																					std::make_tuple(9, osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN, -1, "", 0.0, 0.0, 0.61, 200.0, 3.57, 1.7),
																																					std::make_tuple(10, osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN, -1, "", 0.0, 0.0, 0.61, 500.0, 3.57, 1.7),
																																					std::make_tuple(11, osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN, -1, "", 0.0, 0.0, 0.61, 500.0, 3.57, 1.7) };

	int sign_id = 0;
	osi3::TrafficSign_MainSign_Classification_Type type = osi3::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN;
	double value = 0;
	std::string text = "";
	double pitch = 0;
	double roll = 0;
	double s = 0;
	double t = 0;
	double height = 0;
	double zOffset = 0;

	for (auto traffic_sign : osi_gt.traffic_sign())
	{
		for (auto sign : signs)
		{
			if (traffic_sign.id().value() == std::get<0>(sign))
			{
				sign_id = std::get<0>(sign);
				type = std::get<1>(sign);
				value = std::get<2>(sign);
				text = std::get<3>(sign);
				pitch = std::get<4>(sign);
				roll = std::get<5>(sign);
				height = std::get<6>(sign);
				s = std::get<7>(sign);
				t = std::get<8>(sign);
				zOffset = std::get<9>(sign);
			}
		}
		ASSERT_EQ(traffic_sign.id().value(), sign_id);
		ASSERT_EQ(static_cast<int>(traffic_sign.main_sign().classification().type()), static_cast<int>(type));
		ASSERT_DOUBLE_EQ(traffic_sign.main_sign().classification().value().value(), value);
		ASSERT_STREQ(traffic_sign.main_sign().classification().value().text().c_str(), text.c_str());
		ASSERT_DOUBLE_EQ(traffic_sign.main_sign().base().orientation().pitch(), pitch);
		ASSERT_DOUBLE_EQ(traffic_sign.main_sign().base().orientation().roll(), roll);
		ASSERT_DOUBLE_EQ(traffic_sign.main_sign().base().dimension().height(), height);
	}
	SE_Close();
}

void objectCallback(SE_ScenarioObjectState* state, void* my_data)
{
	SE_ReportObjectRoadPos(state->id, state->timestamp, state->roadId, 5, -2.3f, state->s, state->speed);
}

TEST(GatewayTest, TestReportToGatewayInCallback)
{
	std::string scenario_file = "../../../resources/xosc/cut-in.xosc";

	EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);

	int n_Objects = SE_GetNumberOfObjects();
	EXPECT_EQ(n_Objects, 2);

	SE_RegisterObjectCallback(0, objectCallback, 0);

	SE_ScenarioObjectState state;
	SE_GetObjectState(0, &state);
	ASSERT_EQ(state.laneId, -3);

	SE_StepDT(0.01f);
	SE_GetObjectState(0, &state);
	ASSERT_EQ(state.laneId, 5);
	ASSERT_FLOAT_EQ(state.laneOffset, -2.3f);
}

static void ghostParamDeclCB(void* user_arg)
{
	bool ghostMode = *((bool*)user_arg);

	SE_SetParameterBool("GhostMode", ghostMode);
}

TEST(ExternalController, TestExternalDriver)
{
	void* vehicleHandle = 0;
	SE_SimpleVehicleState vehicleState = { 0, 0, 0, 0, 0, 0 };
	SE_ScenarioObjectState objectState;
	SE_RoadInfo roadInfo;
	double defaultTargetSpeed = 50.0;
	double curveWeight = 30.0;
	double throttleWeight = 0.01;
	bool ghostMode[2] = { false, true };
	float dt = 0.05f;

	SE_AddPath("../../../resources/xodr");
	SE_AddPath("../../../resources/xosc/Catalogs/Vehicles");

	for (int i = 0; i < 2; i++)
	{

		SE_RegisterParameterDeclarationCallback(ghostParamDeclCB, &ghostMode[i]);

		ASSERT_EQ(SE_Init("../../../EnvironmentSimulator/code-examples/test-driver/test-driver.xosc", 0, 0, 0, 0), 0);

		// Lock object to the original lane
		// If setting to false, the object road position will snap to closest lane
		SE_SetLockOnLane(0, true);

		// Initialize the vehicle model, fetch initial state from the scenario
		SE_GetObjectState(0, &objectState);
		vehicleHandle = SE_SimpleVehicleCreate(objectState.x, objectState.y, objectState.h, 4.0, 0.0);

		// show some road features, including road sensor
		SE_ViewerShowFeature(4 + 8, true);  // NODE_MASK_TRAIL_DOTS (1 << 2) & NODE_MASK_ODR_FEATURES (1 << 3),

		// Run for 40 seconds or until 'Esc' button is pressed
		while (SE_GetSimulationTime() < 40 && SE_GetQuitFlag() != 1)
		{
			// Get road information at a point some speed dependent distance ahead
			double targetSpeed;
			if (ghostMode[i] == true)
			{
				// ghost version
				float ghost_speed;
				SE_GetRoadInfoAlongGhostTrail(0, 5 + 0.75f * vehicleState.speed, &roadInfo, &ghost_speed);
				targetSpeed = ghost_speed;

			}
			else
			{
				// Look ahead along the road, to establish target info for the driver model
				SE_GetRoadInfoAtDistance(0, 5 + 0.75f * vehicleState.speed, &roadInfo, 0, true);

				// Slow down when curve ahead - CURVE_WEIGHT is the tuning parameter
				targetSpeed = defaultTargetSpeed / (1 + curveWeight * fabs(roadInfo.angle));
			}

			// Steer towards where the point
			double steerAngle = roadInfo.angle;

			// Accelerate or decelerate towards target speed - THROTTLE_WEIGHT tunes magnitude
			double throttle = throttleWeight * (targetSpeed - vehicleState.speed);

			// Step vehicle model with driver input, but wait until time > 0
			if (SE_GetSimulationTime() > 0)
			{
				SE_SimpleVehicleControlAnalog(vehicleHandle, dt, throttle, steerAngle);
			}

			// Fetch updated state and report to scenario engine
			SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);

			if (i == 0)
			{
				if (abs(SE_GetSimulationTime() - 11.0f) < SMALL_NUMBER)
				{
					SE_GetObjectState(0, &objectState);
					EXPECT_NEAR(objectState.x, 203.401, 1e-3);
					EXPECT_NEAR(objectState.y, 78.381, 1e-3);
					EXPECT_NEAR(objectState.h, 1.100, 1e-3);
					EXPECT_NEAR(objectState.p, 6.263, 1e-3);
				}
				else if (abs(SE_GetSimulationTime() - 30.0f) < SMALL_NUMBER)
				{
					SE_GetObjectState(0, &objectState);
					EXPECT_NEAR(objectState.x, 336.029, 1e-3);
					EXPECT_NEAR(objectState.y, 341.738, 1e-3);
					EXPECT_NEAR(objectState.h, 5.880, 1e-3);
					EXPECT_NEAR(objectState.p, 0.034, 1e-3);
				}
			}
			else if (i==1)
			{
				if (abs(SE_GetSimulationTime() - 11.0f) < SMALL_NUMBER)
				{
					SE_GetObjectState(0, &objectState);
					EXPECT_NEAR(objectState.x, 188.556, 1e-3);
					EXPECT_NEAR(objectState.y, 56.787, 1e-3);
					EXPECT_NEAR(objectState.h, 1.017, 1e-3);
					EXPECT_NEAR(objectState.p, 6.261, 1e-3);
				}
				else if (abs(SE_GetSimulationTime() - 30.0f) < SMALL_NUMBER)
				{
					SE_GetObjectState(0, &objectState);
					EXPECT_NEAR(objectState.x, 319.988, 1e-3);
					EXPECT_NEAR(objectState.y, 347.143, 1e-3);
					EXPECT_NEAR(objectState.h, 6.051, 1e-3);
					EXPECT_NEAR(objectState.p, 0.010, 1e-3);
				}
			}

			// Report updated vehicle position and heading. z, pitch and roll will be aligned to the road
			SE_ReportObjectPosXYH(0, 0, vehicleState.x, vehicleState.y, vehicleState.h, vehicleState.speed);

			// Finally, update scenario using same time step as for vehicle model
			SE_StepDT(dt);
		}
		SE_Close();
	}
}

TEST(SeedTest, TestGetAndSetSeed)
{
	std::string scenario_file = "../../../resources/xosc/cut-in.xosc";

	SE_SetSeed(12345);
	EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);
	ASSERT_EQ(SE_GetNumberOfObjects(), 2);
	ASSERT_EQ(SE_GetSeed(), (unsigned int)12345);

	SE_Close();
}

TEST(SimpleVehicleTest, TestControl)
{
	float dt = 0.01f;

	std::string scenario_file = "../../../resources/xosc/parking_lot.xosc";
	SE_SimpleVehicleState vehicleState = { 0, 0, 0, 0, 0, 0 };
	SE_ScenarioObjectState objectState;
	void* vehicleHandle = 0;

	EXPECT_EQ(SE_Init(scenario_file.c_str(), 1, 0, 0, 0), 0);
	ASSERT_EQ(SE_GetNumberOfObjects(), 1);

	ASSERT_EQ(SE_GetObjectState(0, &objectState), 0);
	EXPECT_NEAR(objectState.x, 1.800, 1e-3);
	EXPECT_NEAR(objectState.y, -358.000, 1e-3);
	EXPECT_NEAR(objectState.h, 1.570, 1e-3);

	vehicleHandle = SE_SimpleVehicleCreate(objectState.x, objectState.y, objectState.h, 4.0, 0.0);

	for (int i = 0; i < 200; i++)
	{
		SE_SimpleVehicleControlTarget(vehicleHandle, dt, 30.0, 0.2);
		SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
		SE_ReportObjectPosXYH(0, 0.0f, vehicleState.x, vehicleState.y, vehicleState.h, vehicleState.speed);

		SE_StepDT(dt);
	}

	SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
	EXPECT_NEAR(vehicleState.x, -18.639, 1e-3);
	EXPECT_NEAR(vehicleState.y, -327.593, 1e-3);
	EXPECT_NEAR(vehicleState.h, 2.471, 1e-3);

	for (int i = 0; i < 200; i++)
	{
		SE_SimpleVehicleControlTarget(vehicleHandle, dt, 60.0, -0.2);
		SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
		SE_ReportObjectPosXYH(0, 0.0f, vehicleState.x, vehicleState.y, vehicleState.h, vehicleState.speed);

		SE_StepDT(dt);
	}

	SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
	EXPECT_NEAR(vehicleState.x, -70.041, 1e-3);
	EXPECT_NEAR(vehicleState.y, -245.796, 1e-3);
	EXPECT_NEAR(vehicleState.h, 1.924, 1e-3);
	EXPECT_NEAR(vehicleState.speed, 60.000, 1e-3);

	SE_SimpleVehicleSetEngineBrakeFactor(vehicleHandle, 0.001f);
	for (int i = 0; i < 200; i++)
	{
		SE_SimpleVehicleControlTarget(vehicleHandle, dt, 80.0, 0.0);
		SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
		SE_ReportObjectPosXYH(0, 0.0f, vehicleState.x, vehicleState.y, vehicleState.h, vehicleState.speed);

		SE_StepDT(dt);
	}

	SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
	EXPECT_NEAR(vehicleState.x, -117.600342, 1e-3);
	EXPECT_NEAR(vehicleState.y, -116.729, 1e-3);
	EXPECT_NEAR(vehicleState.h, 1.924, 1e-3);
	EXPECT_NEAR(vehicleState.speed, 70.0, 1e-3);  // Limited by the default speed 70 km/h

	// no drag factor
	SE_SimpleVehicleSetEngineBrakeFactor(vehicleHandle, 0.0f);  // no engine brake
	for (int i = 0; i < 200; i++)
	{
		SE_SimpleVehicleControlBinary(vehicleHandle, dt, 0, 0);
		SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
		SE_ReportObjectPosXYH(0, 0.0f, vehicleState.x, vehicleState.y, vehicleState.h, vehicleState.speed);

		SE_StepDT(dt);
	}

	SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
	EXPECT_NEAR(vehicleState.x, -166.007, 1e-3);
	EXPECT_NEAR(vehicleState.y, 14.636, 1e-3);
	EXPECT_NEAR(vehicleState.h, 1.924, 1e-3);
	EXPECT_NEAR(vehicleState.speed, 70.0, 1e-3);

	// Strong drag factor
	SE_SimpleVehicleSetEngineBrakeFactor(vehicleHandle, 0.005f);
	for (int i = 0; i < 600; i++)
	{
		SE_SimpleVehicleControlAnalog(vehicleHandle, dt, 0, 0);   // no throttle -> engine brake applied
		SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
		SE_ReportObjectPosXYH(0, 0.0f, vehicleState.x, vehicleState.y, vehicleState.h, vehicleState.speed);

		SE_StepDT(dt);
	}

	SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
	EXPECT_NEAR(vehicleState.x, -211.792, 1e-3);
	EXPECT_NEAR(vehicleState.y, 138.885, 1e-3);
	EXPECT_NEAR(vehicleState.h, 1.924, 1e-3);
	EXPECT_NEAR(vehicleState.speed, 3.459, 1e-3);

	SE_Close();
}

TEST(APITest, TestGetName)
{
	std::string scenario_file = "../../../resources/xosc/cut-in.xosc";

	EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);
	ASSERT_EQ(SE_GetNumberOfObjects(), 2);

	EXPECT_STREQ(SE_GetObjectName(0), "Ego");
	EXPECT_STREQ(SE_GetObjectTypeName(0), "car_white");
	EXPECT_STREQ(SE_GetObjectModelFileName(0), "car_white.osgb");

	EXPECT_STREQ(SE_GetObjectName(1), "OverTaker");
	EXPECT_STREQ(SE_GetObjectTypeName(1), "car_red");
	EXPECT_STREQ(SE_GetObjectModelFileName(1), "car_red.osgb");

	SE_Close();
}


TEST(APITest, TestGetRoute)
{
	std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/highway_exit_with_route.xosc";

	EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);

	int num_of_points = SE_GetNumberOfRoutePoints(0);
	EXPECT_EQ(num_of_points,4);
	
	SE_RouteInfo route_info;
	SE_GetRoutePoint(0,0,&route_info);
	EXPECT_EQ(route_info.t,-1.5);
	EXPECT_EQ(route_info.s,15);
	EXPECT_EQ(route_info.x,15);
	EXPECT_EQ(route_info.y,-1.5);
	SE_GetRoutePoint(0,1,&route_info);
	EXPECT_EQ(route_info.t,-4.5);
	EXPECT_EQ(route_info.s,150);
	EXPECT_EQ(route_info.x,150);
	EXPECT_EQ(route_info.y,-4.5);
	


	SE_Close();
}

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc, argv);

#if 0   // set to 1 and modify filter to run one single test
	testing::GTEST_FLAG(filter) = "*TestGetRoute*";
#else
	SE_LogToConsole(false);
#endif

	return RUN_ALL_TESTS();
}