#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#ifdef _USE_OSI
#include "osi_common.pb.h"
#include "osi_object.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_version.pb.h"
#endif  // _USE_OSI
#include "Replay.hpp"
#include "CommonMini.hpp"
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
    // std::string scenario_file = "../../esmini/resources/xosc/cut-in.xosc";

    SE_ClearPaths();

    const char* Scenario_file = scenario_file.c_str();
    SE_Init(Scenario_file, 0, 0, 0, 0);

    int n_Objects = SE_GetNumberOfObjects();
    SE_Close();

    EXPECT_EQ(n_Objects, std::get<1>(GetParam()));
    // EXPECT_EQ(n_Objects, 2);
}

INSTANTIATE_TEST_SUITE_P(EsminiAPITests,
                         GetNumberOfObjectsTest,
                         ::testing::Values(std::make_tuple("../../../resources/xosc/cut-in.xosc", 2),
                                           std::make_tuple("../../../resources/xosc/highway_merge.xosc", 6),
                                           std::make_tuple("../../../EnvironmentSimulator/Unittest/xosc/full_e6mini.xosc", 14)));

TEST(GetNumberOfObjectsTest, number_of_objects_no_init)
{
    int n_Objects = SE_GetNumberOfObjects();

    EXPECT_EQ(n_Objects, -1);
}

TEST(LoggerTests, check_verbosity_level_warn)
{
    const char* args[] = {"--osc", "../../../resources/xosc/cut-in_simple.xosc", "--headless", "--log_level", "warn"};

    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    SE_LogMessage("This message should not be logged");

    for (int i = 0; i < 6; i++)
    {
        if (i < 5)  // skip step of the last round
        {
            SE_StepDT(0.01f);
        }
    }

    SE_Close();

    FILE* file = FileOpen("log.txt", "r");
    ASSERT_NE(file, nullptr);
    const int max_msg_size = 10000;
    char      msg_buf[max_msg_size];

    size_t size = fread(msg_buf, 1, max_msg_size, file);
    EXPECT_NE(size, 0);

    bool found = strstr(msg_buf, "This message should not be logged") != NULL;
    EXPECT_FALSE(found);

    fclose(file);
}

TEST(LoggerTests, check_verbosity_level_info)
{
    const char* args[] = {"--osc", "../../../resources/xosc/cut-in_simple.xosc", "--headless", "--log_level", "info"};

    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    SE_LogMessage("This message should be logged");

    for (int i = 0; i < 6; i++)
    {
        if (i < 5)  // skip step of the last round
        {
            SE_StepDT(0.01f);
        }
    }

    SE_Close();

    FILE* file = FileOpen("log.txt", "r");
    ASSERT_NE(file, nullptr);
    const int max_msg_size = 10000;
    char      msg_buf[max_msg_size];
    size_t    size = fread(msg_buf, 1, max_msg_size, file);
    EXPECT_NE(size, 0);
    bool found = strstr(msg_buf, "This message should be logged") != NULL;
    EXPECT_TRUE(found);

    fclose(file);
}

TEST(LoggerTests, check_meta_data)
{
    const char* args[] = {"--osc", "../../../resources/xosc/cut-in_simple.xosc", "--headless", "--log_meta_data"};

    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);

    for (int i = 0; i < 6; i++)
    {
        if (i < 5)  // skip step of the last round
        {
            SE_StepDT(0.01f);
        }
    }

    SE_Close();

    FILE* file = FileOpen("log.txt", "r");
    ASSERT_NE(file, nullptr);
    const int max_msg_size = 10000;
    char      msg_buf[max_msg_size];
    size_t    size = fread(msg_buf, 1, max_msg_size, file);
    EXPECT_NE(size, 0);
    bool found = strstr(msg_buf, ".cpp::") != NULL;
    EXPECT_TRUE(found);

    fclose(file);
}

TEST(LoggerTests, check_log_skip_modules)
{
    const char* args[] =
        {"--osc", "../../../resources/xosc/cut-in_simple.xosc", "--headless", "--log_meta_data", "--log_skip_modules", "StoryboardElement"};

    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    SE_LogMessage("This message should be logged");

    for (int i = 0; i < 6; i++)
    {
        if (i < 5)  // skip step of the last round
        {
            SE_StepDT(0.01f);
        }
    }

    SE_Close();

    FILE* file = FileOpen("log.txt", "r");
    ASSERT_NE(file, nullptr);
    const int max_msg_size = 10000;
    char      msg_buf[max_msg_size];
    size_t    size = fread(msg_buf, 1, max_msg_size, file);
    EXPECT_NE(size, 0);
    bool found = strstr(msg_buf, "This message should be logged") != NULL;
    EXPECT_TRUE(found);

    found = strstr(msg_buf, "OverTaker New position:") != NULL;
    EXPECT_TRUE(found);

    found = strstr(msg_buf, "Loaded OpenDRIVE:") != NULL;
    EXPECT_TRUE(found);

    found = strstr(msg_buf, "Init Ego LongitudinalAction initState") != NULL;
    EXPECT_FALSE(found);

    fclose(file);
}

TEST(LoggerTests, check_log_only_modules)
{
    const char* args[] =
        {"--osc", "../../../resources/xosc/cut-in_simple.xosc", "--headless", "--log_meta_data", "--log_only_modules", "StoryboardElement"};

    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
    SE_LogMessage("This message should be not logged");

    for (int i = 0; i < 6; i++)
    {
        if (i < 5)  // skip step of the last round
        {
            SE_StepDT(0.01f);
        }
    }

    SE_Close();

    FILE* file = FileOpen("log.txt", "r");
    ASSERT_NE(file, nullptr);
    const int max_msg_size = 10000;
    char      msg_buf[max_msg_size];
    size_t    size = fread(msg_buf, 1, max_msg_size, file);
    EXPECT_NE(size, 0);
    bool found = strstr(msg_buf, "This message should be not logged") != NULL;
    EXPECT_FALSE(found);

    found = strstr(msg_buf, "OverTaker New position:") != NULL;
    EXPECT_FALSE(found);

    found = strstr(msg_buf, "Loaded OpenDRIVE:") != NULL;
    EXPECT_FALSE(found);

    found = strstr(msg_buf, "Init Ego LongitudinalAction initState") != NULL;
    EXPECT_TRUE(found);

    fclose(file);
}

TEST(LoggerTests, check_log_append)
{
    remove("log.txt");
    const char* args[] = {"--osc", "../../../resources/xosc/cut-in_simple.xosc", "--headless", "--log_meta_data", "--log_append"};

    for (int j = 0; j < 2; ++j)
    {
        ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);
        SE_LogMessage("This message should be logged twice");

        for (int i = 0; i < 6; i++)
        {
            if (i < 5)  // skip step of the last round
            {
                SE_StepDT(0.01f);
            }
        }
        SE_Close();
    }

    FILE* file = FileOpen("log.txt", "r");
    ASSERT_NE(file, nullptr);
    const int max_msg_size = 10000;
    char      msg_buf[max_msg_size];
    size_t    size = fread(msg_buf, 1, max_msg_size, file);
    EXPECT_NE(size, 0);
    char* pos   = strstr(msg_buf, "This message should be logged twice");
    bool  found = pos != NULL;
    EXPECT_TRUE(found);

    pos   = strstr(pos + 1, "This message should be logged twice");
    found = pos != NULL;
    EXPECT_TRUE(found);

    fclose(file);
}

// OSI tests

#ifdef _USE_OSI

TEST(GetOSILaneBoundaryIdsTest, lane_boundary_ids)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/full_e6mini.xosc";

    SE_Init(scenario_file.c_str(), 0, 0, 0, 0);

    int n_Objects = SE_GetNumberOfObjects();
    EXPECT_EQ(n_Objects, 14);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();

    std::vector<std::vector<int>> lane_bound = {{-1, 8, 9, 10},
                                                {8, 9, 10, 0},
                                                {9, 10, 0, 1},
                                                {10, 0, 1, 2},
                                                {0, 1, 2, 3},
                                                {1, 2, 3, 11},
                                                {2, 3, 11, 4},
                                                {3, 11, 4, 5},  // right side start
                                                {11, 4, 5, 6},
                                                {4, 5, 6, 7},
                                                {5, 6, 7, 12},
                                                {6, 7, 12, 13},
                                                {7, 12, 13, 14},
                                                {12, 13, 14, -1}};

    std::vector<int> veh_id = {13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
    for (size_t i = 0; i < lane_bound.size(); i++)
    {
        SE_LaneBoundaryId lanes_id;
        lanes_id.far_left_lb_id  = lane_bound[i][0];
        lanes_id.left_lb_id      = lane_bound[i][1];
        lanes_id.right_lb_id     = lane_bound[i][2];
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
    const char* Scenario_file = scenario_file.c_str();
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
    const char* Scenario_file = scenario_file.c_str();
    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    int               lanes_found        = 0;
    bool              intersection_found = false;
    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
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
    const char* Scenario_file = scenario_file.c_str();
    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    int               lanes_found        = 0;
    bool              intersection_found = false;
    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
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
    const char* Scenario_file = scenario_file.c_str();
    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    int               lanes_found        = 0;
    bool              intersection_found = false;
    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
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

TEST(OSIStationaryObjects, square_building)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/Junction_with_building0.xosc";
    const char* Scenario_file = scenario_file.c_str();
    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    // SE_UpdateOSIGroundTruth();

    // osi3::GroundTruth osi_gt;
    // int               sv_size = 0;
    // const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
    // osi_gt.ParseFromArray(gt, sv_size);
    // ASSERT_EQ(osi_gt.stationary_object_size(), 1);
    // for (int i = 0; i < osi_gt.stationary_object_size(); i++)
    // {
    //     EXPECT_EQ(osi_gt.stationary_object(i).base().base_polygon_size(), 0);
    //     EXPECT_EQ(osi_gt.stationary_object(i).base().position().x(), 80);
    //     EXPECT_EQ(osi_gt.stationary_object(i).base().position().y(), 20);
    //     EXPECT_EQ(osi_gt.stationary_object(i).base().dimension().length(), 30);
    //     EXPECT_EQ(osi_gt.stationary_object(i).base().dimension().width(), 20);
    //     EXPECT_EQ(osi_gt.stationary_object(i).base().dimension().height(), 4);
    //     EXPECT_EQ(osi_gt.stationary_object(i).classification().type(), osi3::StationaryObject_Classification_Type_TYPE_BUILDING);
    // }

    SE_Close();
}

class OSIStationaryObjectsOutline : public ::testing::TestWithParam<std::tuple<std::string>>
{
};

TEST_P(OSIStationaryObjectsOutline, object_with_outline)
{
    std::string scenario_file = std::get<0>(GetParam());
    const char* Scenario_file = scenario_file.c_str();
    ASSERT_EQ(SE_Init(Scenario_file, 0, 0, 0, 0), 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();

    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
    osi_gt.ParseFromArray(gt, sv_size);
    ASSERT_EQ(osi_gt.stationary_object_size(), 1);
    for (int i = 0; i < osi_gt.stationary_object_size(); i++)
    {
        EXPECT_EQ(osi_gt.stationary_object(i).base().base_polygon_size(), 8);
        EXPECT_EQ(osi_gt.stationary_object(i).base().base_polygon(0).x(), 20);
        EXPECT_EQ(osi_gt.stationary_object(i).base().base_polygon(0).y(), 0);
        EXPECT_EQ(osi_gt.stationary_object(i).base().dimension().height(), 4);
        EXPECT_NEAR(osi_gt.stationary_object(i).base().position().x(), 10.0, 0.1);
        EXPECT_NEAR(osi_gt.stationary_object(i).base().position().y(), -25.0, 0.1);
    }

    SE_Close();
}

INSTANTIATE_TEST_SUITE_P(OSIStationaryObjects,
                         OSIStationaryObjectsOutline,
                         ::testing::Values(std::make_tuple("../../../EnvironmentSimulator/Unittest/xosc/Weird_looking_building_road_coord.xosc"),
                                           std::make_tuple("../../../EnvironmentSimulator/Unittest/xosc/Weird_looking_building_local_coord.xosc")));

TEST(OSIintersections, multilane)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/multilane_3way_intersection_osi.xosc";
    const char* Scenario_file = scenario_file.c_str();
    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    int               lanes_found        = 0;
    bool              intersection_found = false;
    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
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
    struct stat fileStatus;
    std::string scenario_file = "../../../resources/xosc/cut-in.xosc";
    const char* Scenario_file = scenario_file.c_str();

    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_EnableOSIFile("gt.osi");

    ASSERT_EQ(stat("gt.osi", &fileStatus), 0);
    EXPECT_EQ(fileStatus.st_size, 0);  // so far, nothing has been saved

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    SE_FlushOSIFile();
    ASSERT_EQ(stat("gt.osi", &fileStatus), 0);
    EXPECT_EQ(fileStatus.st_size, 84957);  // initial OSI size, including static content

    int road_lane_size;

    const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, 15);

    EXPECT_EQ(road_lane_size, 0);
    EXPECT_EQ(road_lane, nullptr);

    SE_StepDT(0.001f);  // Step for write another frame to osi file
    SE_UpdateOSIGroundTruth();
    SE_FlushOSIFile();
    ASSERT_EQ(stat("gt.osi", &fileStatus), 0);
    EXPECT_EQ(fileStatus.st_size, 86250);  // slight growth due to only dynamic updates

    SE_StepDT(0.001f);  // Step for write another frame to osi file
    SE_UpdateOSIGroundTruth();
    SE_FlushOSIFile();
    ASSERT_EQ(stat("gt.osi", &fileStatus), 0);
    EXPECT_EQ(fileStatus.st_size, 87544);  // slight growth due to only dynamic updates

    SE_DisableOSIFile();
    SE_Close();
}

TEST(GetOSIRoadLaneTest, lane_id)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/full_e6mini.xosc";
    const char* Scenario_file = scenario_file.c_str();

    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();

    std::vector<int> lanes  = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
    std::vector<int> veh_id = {13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

    int        road_lane_size;
    osi3::Lane osi_lane;

    for (size_t i = 0; i < lanes.size(); i++)
    {
        int lane_id = lanes[i];

        const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
        osi_lane.ParseFromArray(road_lane, road_lane_size);

        EXPECT_EQ(osi_lane.id().value(), lane_id);
    }
    SE_Close();
}

TEST(GetOSIRoadLaneTest, left_lane_id)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/full_e6mini.xosc";
    const char* Scenario_file = scenario_file.c_str();
    SE_Init(Scenario_file, 0, 0, 0, 0);

    int n_Objects = SE_GetNumberOfObjects();
    EXPECT_EQ(n_Objects, 14);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    int        road_lane_size;
    osi3::Lane osi_lane;

    // explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
    std::vector<int> lanes  = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
    std::vector<int> veh_id = {13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

    for (size_t i = 0; i < lanes.size(); i++)
    {
        const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
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
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/full_e6mini.xosc";
    const char* Scenario_file = scenario_file.c_str();

    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    int        road_lane_size;
    osi3::Lane osi_lane;

    // explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
    std::vector<int> lanes  = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
    std::vector<int> veh_id = {13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

    for (size_t i = 0; i < lanes.size(); i++)
    {
        const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
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
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/full_e6mini.xosc";
    const char* Scenario_file = scenario_file.c_str();

    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    int        road_lane_size;
    osi3::Lane osi_lane;

    // explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
    std::vector<int> lane_bound = {8, 9, 10, 0, 1, 2, 3, 11, 4, 5, 6, 7, 12, 13, 14};
    std::vector<int> veh_id     = {13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

    for (size_t i = 0; i < lane_bound.size() - 1; i++)
    {
        const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
        osi_lane.ParseFromArray(road_lane, road_lane_size);

        EXPECT_EQ(osi_lane.classification().right_lane_boundary_id(0).value(), lane_bound[i + 1]);
    }

    SE_Close();
}

TEST(GetOSIRoadLaneTest, left_lane_boundary_id)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/full_e6mini.xosc";
    const char* Scenario_file = scenario_file.c_str();

    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    int        road_lane_size;
    osi3::Lane osi_lane;

    // explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
    std::vector<int> lane_bound = {8, 9, 10, 0, 1, 2, 3, 11, 4, 5, 6, 7, 12, 13, 14};
    std::vector<int> veh_id     = {13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

    for (size_t i = 0; i < veh_id.size(); i++)
    {
        const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
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
    const char* Scenario_file = scenario_file.c_str();

    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();

    int        road_lane_size;
    osi3::Lane osi_lane;

    // explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
    std::vector<int> lanes  = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
    std::vector<int> veh_id = {13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

    for (size_t i = 0; i < lanes.size(); i++)
    {
        const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
        osi_lane.ParseFromArray(road_lane, road_lane_size);
        if (veh_id[i] < 7)
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

INSTANTIATE_TEST_SUITE_P(EsminiAPITests,
                         GetOSIRoadLaneTest,
                         ::testing::Values(std::make_tuple("../../../EnvironmentSimulator/Unittest/xosc/full_e6mini.xosc", true, false),
                                           std::make_tuple("../../../EnvironmentSimulator/Unittest/xosc/full_e6mini_reverse.xosc", true, false)));

TEST(GetOSIRoadLaneTest, is_host_vehicle_lane)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/full_e6mini.xosc";
    const char* Scenario_file = scenario_file.c_str();

    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    int        road_lane_size;
    osi3::Lane osi_lane;

    // explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
    std::vector<int> lanes  = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
    std::vector<int> veh_id = {13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

    for (size_t i = 0; i < lanes.size(); i++)
    {
        const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
        osi_lane.ParseFromArray(road_lane, road_lane_size);

        EXPECT_EQ(osi_lane.classification().is_host_vehicle_lane(), false);
    }

    SE_Close();
}

TEST(GetOSIRoadLaneTest, lane_classification)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/full_e6mini.xosc";
    const char* Scenario_file = scenario_file.c_str();

    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    int        road_lane_size;
    osi3::Lane osi_lane;

    // explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
    std::vector<int> lanes  = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
    std::vector<int> veh_id = {13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

    int obj_id = 0;

    for (size_t i = 0; i < lanes.size(); i++)
    {
        int lane_id = lanes[i];

        const char* road_lane = SE_GetOSIRoadLane(&road_lane_size, veh_id[i]);
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
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/full_e6mini.xosc";
    const char* Scenario_file = scenario_file.c_str();

    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    int                lb_size;
    osi3::LaneBoundary osi_lb;

    // explicitly writing lanes ID so that it will be easy to adapt the test for more complex roads in the future
    std::vector<int> lanes      = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
    std::vector<int> lane_bound = {-1, 8, 9, 10, 0, 1, 2, 3, 11, 4, 5, 6, 7, 12, 13, 14, -1};

    for (size_t i = 0; i < lane_bound.size(); i++)
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

class GetOSILaneBoundaryTests : public ::testing::TestWithParam<std::tuple<int, int>>
{
};
// inp:  excisting lane boundary global id
// expected: size of osi lane boundary message = 0

TEST_P(GetOSILaneBoundaryTests, lane_boundary_id_not_existing)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/full_e6mini.xosc";
    const char* Scenario_file = scenario_file.c_str();

    SE_Init(Scenario_file, 0, 0, 0, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    int lb_size = 0;

    const char* lb = SE_GetOSILaneBoundary(&lb_size, std::get<0>(GetParam()));

    EXPECT_EQ(lb_size, std::get<1>(GetParam()));
    EXPECT_EQ(lb, nullptr);

    SE_Close();
}

INSTANTIATE_TEST_SUITE_P(EsminiAPITests, GetOSILaneBoundaryTests, ::testing::Values(std::make_tuple(15, 0), std::make_tuple(-15, 0)));

TEST(OSIFile, writeosifile_two_step)
{
    std::string    scenario_file = "../../../resources/xosc/cut-in.xosc";
    const char*    Scenario_file = scenario_file.c_str();
    std::streamoff file_size1, file_size2;

    SE_EnableOSIFile("");
    SE_Init(Scenario_file, 0, 0, 0, 0);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    SE_FlushOSIFile();

    std::ifstream in_file("ground_truth.osi", std::ios::binary);
    in_file.seekg(0, std::ios::end);
    file_size1 = in_file.tellg();
    // std::cout <<"Size of the file at first step "<< file_size1 << " bytes" << std::endl;

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    SE_FlushOSIFile();

    in_file.seekg(0, std::ios::end);
    file_size2 = in_file.tellg();
    // std::cout <<"Size of the file at second step "<< file_size2 << " bytes" << std::endl;

    SE_Close();
    SE_DisableOSIFile();

    EXPECT_LT(file_size1, file_size2);
    EXPECT_GT(file_size1, file_size2 - file_size1);  // first package larger since including static stuff
}

TEST(OSIFile, updateosi_no_init)
{
    SE_EnableOSIFile(0);

    EXPECT_EQ(SE_UpdateOSIGroundTruth(), false);

    SE_DisableOSIFile();
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
    std::string       scenario_file = std::get<0>(GetParam());
    const char*       Scenario_file = scenario_file.c_str();
    int               sv_size       = 0;
    osi3::GroundTruth osi_gt;

    SE_ClearPaths();
    SE_Init(Scenario_file, 0, 0, 0, 0);

    // SE_OSIFileOpen(0);

    SE_StepDT(0.001f);

    SE_UpdateOSIGroundTruth();

    const char* sv = SE_GetOSIGroundTruth(&sv_size);
    osi_gt.ParseFromArray(sv, sv_size);

    int n_lanes = osi_gt.lane_size();

    int n_objects = osi_gt.mutable_moving_object()->size();

    int ego_index = 0;  // ego vehicle are always first in tested scenarios

    float ego_length  = static_cast<float>(osi_gt.mutable_moving_object(ego_index)->mutable_base()->mutable_dimension()->length());
    float ego_width   = static_cast<float>(osi_gt.mutable_moving_object(ego_index)->mutable_base()->mutable_dimension()->width());
    float ego_height  = static_cast<float>(osi_gt.mutable_moving_object(ego_index)->mutable_base()->mutable_dimension()->height());
    float ego_xoffset = static_cast<float>(osi_gt.mutable_moving_object(ego_index)->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->x());
    float ego_yoffset = static_cast<float>(osi_gt.mutable_moving_object(ego_index)->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->y());
    float ego_zoffset = static_cast<float>(osi_gt.mutable_moving_object(ego_index)->mutable_vehicle_attributes()->mutable_bbcenter_to_rear()->z());
    std::string map_reference = osi_gt.map_reference();

    EXPECT_EQ(n_lanes, std::get<1>(GetParam()));
    EXPECT_EQ(n_objects, std::get<2>(GetParam()));
    EXPECT_EQ(ego_length, std::get<3>(GetParam()).length);
    EXPECT_EQ(ego_width, std::get<3>(GetParam()).width);
    EXPECT_EQ(ego_height, std::get<3>(GetParam()).height);
    EXPECT_NEAR(ego_xoffset, std::get<3>(GetParam()).centerOffsetX, 1e-5);
    EXPECT_NEAR(ego_yoffset, std::get<3>(GetParam()).centerOffsetY, 1e-5);
    EXPECT_NEAR(ego_zoffset, std::get<3>(GetParam()).centerOffsetZ, 1e-5);
    EXPECT_EQ(map_reference, std::get<4>(GetParam()));

    SE_Close();
}

INSTANTIATE_TEST_SUITE_P(EsminiAPITests,
                         GetGroundTruthTests,
                         ::testing::Values(std::make_tuple("../../../resources/xosc/cut-in.xosc",
                                                           14,
                                                           2,
                                                           bounding_box{5.04f, 2.0f, 1.5f, -1.4f, 0.0f, -0.35f},
                                                           "+proj=utm +lat_0=37.3542934123933 +lon_0=-122.0859797650754"),
                                           std::make_tuple("../../../resources/xosc/straight_500m.xosc",
                                                           6,
                                                           2,
                                                           bounding_box{5.0f, 2.0f, 1.8f, -1.4f, 0.0f, -0.5f},
                                                           "+proj=utm +lat_0=37.3542934123933 +lon_0=-122.0859797650754"),
                                           std::make_tuple("../../../resources/xosc/highway_merge.xosc",
                                                           33,
                                                           6,
                                                           bounding_box{5.04f, 2.0f, 1.5f, -1.4f, 0.0f, -0.35f},
                                                           "+proj=utm +lat_0=37.3542934123933 +lon_0=-122.0859797650754")));
// scenario_file_name, number_of_lanes, number_of_objects, ego_bounding_box

TEST(GetGroundTruthTests, receive_GroundTruth_no_init)
{
    int               sv_size = 0;
    osi3::GroundTruth osi_gt;

    const char* sv = SE_GetOSIGroundTruth(&sv_size);

    EXPECT_EQ(sv_size, 0);
    EXPECT_EQ(sv, nullptr);
}

TEST(GroundTruthTests, check_GroundTruth_including_init_state)
{
    const osi3::GroundTruth* osi_gt_ptr;
    osi3::GroundTruth        osi_gt;
    struct stat              fileStatus;
    double                   seconds       = 0.0, obj_x, obj_y, obj_z;
    double                   x_vals[]      = {51.400, 51.600, 51.800};
    double                   time_stamps[] = {0.00, 0.01, 0.02};

    SE_EnableOSIFile("gt.osi");
    ASSERT_EQ(SE_Init("../../../resources/xosc/cut-in_simple.xosc", 0, 0, 0, 0), 0);
    SE_UpdateOSIGroundTruth();

    osi_gt_ptr = reinterpret_cast<const osi3::GroundTruth*>(SE_GetOSIGroundTruthRaw());

    for (int i = 0; i < 3; i++)
    {
        EXPECT_EQ(osi_gt_ptr->moving_object().size(), 2);
        seconds = static_cast<double>(osi_gt_ptr->timestamp().seconds()) + 1E-9 * static_cast<double>(osi_gt_ptr->timestamp().nanos());
        EXPECT_NEAR(seconds, time_stamps[i], 1E-5);
        obj_x = osi_gt_ptr->moving_object(0).base().position().x();
        obj_y = osi_gt_ptr->moving_object(0).base().position().y();
        obj_z = osi_gt_ptr->moving_object(0).base().position().z();
        EXPECT_NEAR(obj_x, x_vals[i], 1E-5);
        EXPECT_NEAR(obj_y, -1.535, 1E-5);
        EXPECT_NEAR(obj_z, 0.75, 1E-5);
        EXPECT_NEAR(osi_gt_ptr->moving_object(0).vehicle_attributes().bbcenter_to_rear().z(), -0.35, 1E-5);

        if (i < 2)  // skip step of the last round
        {
            SE_StepDT(0.01f);
            SE_UpdateOSIGroundTruth();
        }
    }

    SE_Close();
    SE_DisableOSIFile();

    ASSERT_EQ(stat("gt.osi", &fileStatus), 0);
    EXPECT_EQ(fileStatus.st_size, 9252);

    // Read OSI file
    FILE* file = FileOpen("gt.osi", "rb");
    ASSERT_NE(file, nullptr);

    const int max_msg_size = 10000;
    int       msg_size;
    char      msg_buf[max_msg_size];

    for (int i = 0; i < 3; i++)
    {
        ASSERT_EQ(fread(reinterpret_cast<char*>(&msg_size), 1, sizeof(msg_size), file), sizeof(msg_size));

        ASSERT_LE(msg_size, max_msg_size);
        EXPECT_EQ(fread(msg_buf, 1, static_cast<size_t>(msg_size), file), msg_size);
        osi_gt.ParseFromArray(msg_buf, msg_size);

        EXPECT_EQ(osi_gt.mutable_moving_object()->size(), 2);
        seconds = static_cast<double>(osi_gt.mutable_timestamp()->seconds()) + 1E-9 * static_cast<double>(osi_gt.mutable_timestamp()->nanos());
        EXPECT_NEAR(seconds, time_stamps[i], 1E-5);
        obj_x = osi_gt.mutable_moving_object(0)->mutable_base()->mutable_position()->x();
        obj_y = osi_gt.mutable_moving_object(0)->mutable_base()->mutable_position()->y();
        obj_z = osi_gt.mutable_moving_object(0)->mutable_base()->mutable_position()->z();
        EXPECT_NEAR(obj_x, x_vals[i], 1E-5);
        EXPECT_NEAR(obj_y, -1.535, 1E-5);
        EXPECT_NEAR(obj_z, 0.75, 1E-5);
    }

    fclose(file);
}

TEST(GroundTruthTests, check_frequency_implicit)
{
    const osi3::GroundTruth* osi_gt_ptr;
    osi3::GroundTruth        osi_gt;
    struct stat              fileStatus;
    double                   seconds       = 0.0, obj_x, obj_y, obj_z;
    double                   x_vals[]      = {51.400, 51.800, 52.200};
    double                   time_stamps[] = {0.00, 0.02, 0.04};

    SE_EnableOSIFile("gt_implicit.osi");
    ASSERT_EQ(SE_Init("../../../resources/xosc/cut-in_simple.xosc", 0, 0, 0, 0), 0);
    SE_UpdateOSIGroundTruth();

    osi_gt_ptr = reinterpret_cast<const osi3::GroundTruth*>(SE_GetOSIGroundTruthRaw());

    for (int i = 0; i < 6; i++)
    {
        // Read OSI message, should be identical every two pair of frames - i/2 will result in 0, 0, 1, 1, 2, 2 and so on
        EXPECT_EQ(osi_gt_ptr->moving_object().size(), 2);
        seconds = static_cast<double>(osi_gt_ptr->timestamp().seconds()) + 1E-9 * static_cast<double>(osi_gt_ptr->timestamp().nanos());
        EXPECT_NEAR(seconds, time_stamps[i / 2], 1E-5);
        obj_x = osi_gt_ptr->moving_object(0).base().position().x();
        obj_y = osi_gt_ptr->moving_object(0).base().position().y();
        obj_z = osi_gt_ptr->moving_object(0).base().position().z();
        EXPECT_NEAR(obj_x, x_vals[i / 2], 1E-5);
        EXPECT_NEAR(obj_y, -1.535, 1E-5);
        EXPECT_NEAR(obj_z, 0.75, 1E-5);
        EXPECT_NEAR(osi_gt_ptr->moving_object(0).vehicle_attributes().bbcenter_to_rear().z(), -0.35, 1E-5);

        if (i < 5)  // skip step of the last round
        {
            SE_StepDT(0.01f);
            if ((i % 2) == 1)  // Update OSI every second time
            {
                SE_UpdateOSIGroundTruth();
            }
        }
    }

    SE_DisableOSIFile();
    SE_Close();

    ASSERT_EQ(stat("gt_implicit.osi", &fileStatus), 0);
    EXPECT_EQ(fileStatus.st_size, 9252);

    // Read OSI file
    FILE* file = FileOpen("gt_implicit.osi", "rb");
    ASSERT_NE(file, nullptr);

    const int max_msg_size = 10000;
    int       msg_size;
    char      msg_buf[max_msg_size];

    for (int i = 0; i < 3; i++)
    {
        ASSERT_EQ(fread(reinterpret_cast<char*>(&msg_size), 1, sizeof(msg_size), file), sizeof(msg_size));

        ASSERT_LE(msg_size, max_msg_size);
        EXPECT_EQ(fread(msg_buf, 1, static_cast<size_t>(msg_size), file), msg_size);
        osi_gt.ParseFromArray(msg_buf, msg_size);

        EXPECT_EQ(osi_gt.mutable_moving_object()->size(), 2);
        seconds = static_cast<double>(osi_gt.mutable_timestamp()->seconds()) + 1E-9 * static_cast<double>(osi_gt.mutable_timestamp()->nanos());
        EXPECT_NEAR(seconds, time_stamps[i], 1E-5);
        obj_x = osi_gt.mutable_moving_object(0)->mutable_base()->mutable_position()->x();
        obj_y = osi_gt.mutable_moving_object(0)->mutable_base()->mutable_position()->y();
        obj_z = osi_gt.mutable_moving_object(0)->mutable_base()->mutable_position()->z();
        EXPECT_NEAR(obj_x, x_vals[i], 1E-5);
        EXPECT_NEAR(obj_y, -1.535, 1E-5);
        EXPECT_NEAR(obj_z, 0.75, 1E-5);
    }

    fclose(file);
}

TEST(GroundTruthTests, check_frequency_explicit)
{
    const osi3::GroundTruth* osi_gt_ptr;
    osi3::GroundTruth        osi_gt;
    struct stat              fileStatus;
    double                   seconds       = 0.0, obj_x, obj_y, obj_z;
    double                   x_vals[]      = {51.400, 51.800, 52.200};
    double                   time_stamps[] = {0.00, 0.02, 0.04};

    const char* args[] = {"--osc", "../../../resources/xosc/cut-in_simple.xosc", "--headless", "--osi_file", "gt_explicit.osi", "--osi_freq", "2"};

    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);

    osi_gt_ptr = reinterpret_cast<const osi3::GroundTruth*>(SE_GetOSIGroundTruthRaw());

    for (int i = 0; i < 6; i++)
    {
        // Read OSI message, should be identical every two pair of frames - i/2 will result in 0, 0, 1, 1, 2, 2 and so on
        EXPECT_EQ(osi_gt_ptr->moving_object().size(), 2);
        seconds = static_cast<double>(osi_gt_ptr->timestamp().seconds()) + 1E-9 * static_cast<double>(osi_gt_ptr->timestamp().nanos());
        EXPECT_NEAR(seconds, time_stamps[i / 2], 1E-5);
        obj_x = osi_gt_ptr->moving_object(0).base().position().x();
        obj_y = osi_gt_ptr->moving_object(0).base().position().y();
        obj_z = osi_gt_ptr->moving_object(0).base().position().z();
        EXPECT_NEAR(obj_x, x_vals[i / 2], 1E-5);
        EXPECT_NEAR(obj_y, -1.535, 1E-5);
        EXPECT_NEAR(obj_z, 0.75, 1E-5);
        EXPECT_NEAR(osi_gt_ptr->moving_object(0).vehicle_attributes().bbcenter_to_rear().z(), -0.35, 1E-5);

        if (i < 5)  // skip step of the last round
        {
            SE_StepDT(0.01f);
        }
    }

    SE_Close();

    ASSERT_EQ(stat("gt_explicit.osi", &fileStatus), 0);
    EXPECT_EQ(fileStatus.st_size, 9252);

    // Read OSI file
    FILE* file = FileOpen("gt_explicit.osi", "rb");
    ASSERT_NE(file, nullptr);

    const int max_msg_size = 10000;
    int       msg_size;
    char      msg_buf[max_msg_size];

    for (int i = 0; i < 3; i++)
    {
        ASSERT_EQ(fread(reinterpret_cast<char*>(&msg_size), 1, sizeof(msg_size), file), sizeof(msg_size));

        // Read OSI message
        ASSERT_LE(msg_size, max_msg_size);
        EXPECT_EQ(fread(msg_buf, 1, static_cast<size_t>(msg_size), file), msg_size);
        osi_gt.ParseFromArray(msg_buf, msg_size);

        EXPECT_EQ(osi_gt.mutable_moving_object()->size(), 2);
        seconds = static_cast<double>(osi_gt.mutable_timestamp()->seconds()) + 1E-9 * static_cast<double>(osi_gt.mutable_timestamp()->nanos());
        EXPECT_NEAR(seconds, time_stamps[i], 1E-5);
        obj_x = osi_gt.mutable_moving_object(0)->mutable_base()->mutable_position()->x();
        obj_y = osi_gt.mutable_moving_object(0)->mutable_base()->mutable_position()->y();
        obj_z = osi_gt.mutable_moving_object(0)->mutable_base()->mutable_position()->z();
        EXPECT_NEAR(obj_x, x_vals[i], 1E-5);
        EXPECT_NEAR(obj_y, -1.535, 1E-5);
        EXPECT_NEAR(obj_z, 0.75, 1E-5);
    }

    fclose(file);
}

TEST(GroundTruthTests, check_teleport_not_affecting_vel_and_acc)
{
    const osi3::GroundTruth* osi_gt_ptr;
    osi3::GroundTruth        osi_gt;
    double                   seconds = 0.0;

    const char* args[] = {"--osc",
                          "../../../EnvironmentSimulator/Unittest/xosc/sudden_teleport.xosc",
                          "--headless",
                          "--osi_file",
                          "gt_sudden_teleport.osi"};

    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);

    osi_gt_ptr = reinterpret_cast<const osi3::GroundTruth*>(SE_GetOSIGroundTruthRaw());

    // Read OSI message, should be identical every two pair of frames - i/2 will result in 0, 0, 1, 1, 2, 2 and so on
    EXPECT_EQ(osi_gt_ptr->moving_object().size(), 1);
    seconds = static_cast<double>(osi_gt_ptr->timestamp().seconds()) + 1E-9 * static_cast<double>(osi_gt_ptr->timestamp().nanos());
    EXPECT_NEAR(seconds, 0.0, 1e-5);
    EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().position().x(), 21.4, 1e-5);  // OSI center is center of entity bounding box
    EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().velocity().x(), 10.0, 1e-5);
    EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().acceleration().x(), 0.0, 1e-5);

    while (SE_GetSimulationTime() < 1.5f)
    {
        SE_StepDT(0.1f);
        EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().velocity().x(), 10.0, 1e-5);
        EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().acceleration().x(), 0.0, 1e-5);
    }

    SE_Close();
}

TEST(GetMiscObjFromGroundTruth, receive_miscobj)
{
    int               sv_size = 0;
    osi3::GroundTruth osi_gt;

    SE_Init("../../../EnvironmentSimulator/Unittest/xosc/miscobj_basic.xosc", 0, 0, 0, 0);

    SE_StepDT(0.001f);

    SE_UpdateOSIGroundTruth();

    const char* gt = SE_GetOSIGroundTruth(&sv_size);
    osi_gt.ParseFromArray(gt, sv_size);

    int n_miscobjects = osi_gt.mutable_stationary_object()->size();

    uint64_t                                   miscobj_id   = osi_gt.mutable_stationary_object(0)->mutable_id()->value();
    osi3::StationaryObject_Classification_Type miscobj_type = osi_gt.mutable_stationary_object(0)->mutable_classification()->type();

    double miscobj_length = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_dimension()->length();
    double miscobj_width  = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_dimension()->width();
    double miscobj_height = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_dimension()->height();

    double miscobj_x = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_position()->x();
    double miscobj_y = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_position()->y();
    double miscobj_z = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_position()->z();

    double miscobj_roll  = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_orientation()->roll();
    double miscobj_pitch = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_orientation()->pitch();
    double miscobj_yaw   = osi_gt.mutable_stationary_object(0)->mutable_base()->mutable_orientation()->yaw();

    EXPECT_EQ(n_miscobjects, 1);
    EXPECT_EQ(miscobj_id, 0);

    EXPECT_EQ(miscobj_type, osi3::StationaryObject_Classification_Type::StationaryObject_Classification_Type_TYPE_BARRIER);

    EXPECT_EQ(miscobj_length, 200);
    EXPECT_EQ(miscobj_width, 1);
    EXPECT_EQ(miscobj_height, 4);

    EXPECT_EQ(miscobj_x, 110);
    EXPECT_EQ(miscobj_y, -10);
    EXPECT_EQ(miscobj_z, 0.0);  // adjusted to the road z

    // Angles in OSI should be in range [-PI, PI]
    EXPECT_NEAR(miscobj_roll, 0.2, 1e-3);   // Aligned to the road (so if road roll is 1.0 total roll will be 6.0)
    EXPECT_NEAR(miscobj_pitch, 0.0, 1e-3);  // Aligned to the road (so if road pitch is 1.0 total pitch will be 6.0)
    EXPECT_NEAR(miscobj_yaw, -0.05, 1e-3);

    SE_Close();
}

TEST(TestGetAndSet, SetOSITimestampTest)
{
    const osi3::GroundTruth* osi_gt;

    std::string scenario_file = "../../../resources/xosc/cut-in.xosc";

    EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);

    int n_Objects = SE_GetNumberOfObjects();
    EXPECT_EQ(n_Objects, 2);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();

    osi_gt = reinterpret_cast<const osi3::GroundTruth*>(SE_GetOSIGroundTruthRaw());

    EXPECT_EQ(osi_gt->moving_object().size(), 2);

    double seconds = static_cast<double>(osi_gt->timestamp().seconds()) + 1E-9 * static_cast<double>(osi_gt->timestamp().nanos());
    EXPECT_DOUBLE_EQ(seconds, 0.001);

    SE_OSISetTimeStamp(1234543210);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    EXPECT_EQ(osi_gt->timestamp().seconds(), 1);
    EXPECT_EQ(osi_gt->timestamp().nanos(), static_cast<unsigned int>(234543210));

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    // Expect no change as timestamp has been set explicitly only once
    EXPECT_EQ(osi_gt->timestamp().seconds(), 1);
    EXPECT_EQ(osi_gt->timestamp().nanos(), static_cast<unsigned int>(234543210));

    SE_OSISetTimeStamp(5234543229);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    // Expect updated timestamp
    EXPECT_EQ(osi_gt->timestamp().seconds(), 5);
    EXPECT_EQ(osi_gt->timestamp().nanos(), static_cast<unsigned int>(234543229));

    SE_Close();
}

TEST(TestGetAndSet, ReportObjectAcc)
{
    const osi3::GroundTruth* osi_gt;

    std::string scenario_file = "../../../resources/xosc/cut-in_simple.xosc";

    EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);

    int n_Objects = SE_GetNumberOfObjects();
    EXPECT_EQ(n_Objects, 2);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();

    osi_gt = reinterpret_cast<const osi3::GroundTruth*>(SE_GetOSIGroundTruthRaw());

    EXPECT_EQ(osi_gt->moving_object().size(), 2);

    double seconds = static_cast<double>(osi_gt->timestamp().seconds()) + 1E-9 * static_cast<double>(osi_gt->timestamp().nanos());
    EXPECT_DOUBLE_EQ(seconds, 0.001);

    SE_ReportObjectAcc(0, 0, 1, 2, 3);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    EXPECT_EQ(osi_gt->moving_object(0).base().acceleration().x(), 1.0);

    SE_ReportObjectAcc(0, 0, 4, 1, 8);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    EXPECT_EQ(osi_gt->moving_object(0).base().acceleration().x(), 4.0);
    EXPECT_EQ(osi_gt->moving_object(0).base().acceleration().y(), 1.0);
    EXPECT_EQ(osi_gt->moving_object(0).base().acceleration().z(), 8.0);

    SE_ReportObjectAngularAcc(1, 0, 5, 4, 3);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    EXPECT_EQ(osi_gt->moving_object(1).base().orientation_acceleration().yaw(), 5.0);
    EXPECT_EQ(osi_gt->moving_object(1).base().orientation_acceleration().pitch(), 4.0);
    EXPECT_EQ(osi_gt->moving_object(1).base().orientation_acceleration().roll(), 3.0);

    SE_Close();
}

TEST(TestGetAndSet, ReportObjectVel)
{
    const osi3::GroundTruth* osi_gt;

    std::string scenario_file = "../../../resources/xosc/cut-in_simple.xosc";

    EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);

    int n_Objects = SE_GetNumberOfObjects();
    EXPECT_EQ(n_Objects, 2);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();

    osi_gt = reinterpret_cast<const osi3::GroundTruth*>(SE_GetOSIGroundTruthRaw());

    EXPECT_EQ(osi_gt->moving_object().size(), 2);

    double seconds = static_cast<double>(osi_gt->timestamp().seconds()) + 1E-9 * static_cast<double>(osi_gt->timestamp().nanos());
    EXPECT_DOUBLE_EQ(seconds, 0.001);

    SE_ReportObjectVel(0, 0, 11, 12, 13);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    EXPECT_EQ(osi_gt->moving_object(0).base().velocity().x(), 11.0);

    SE_ReportObjectVel(0, 0, 21, 22, 23);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    EXPECT_EQ(osi_gt->moving_object(0).base().velocity().x(), 21.0);
    EXPECT_EQ(osi_gt->moving_object(0).base().velocity().y(), 22.0);
    EXPECT_EQ(osi_gt->moving_object(0).base().velocity().z(), 23.0);

    SE_ReportObjectAngularVel(1, 0, 25, 24, 23);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    EXPECT_EQ(osi_gt->moving_object(1).base().orientation_rate().yaw(), 25.0);
    EXPECT_EQ(osi_gt->moving_object(1).base().orientation_rate().pitch(), 24.0);
    EXPECT_EQ(osi_gt->moving_object(1).base().orientation_rate().roll(), 23.0);

    SE_Close();
}

TEST(OSILaneParing, multi_roads)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/consecutive_roads.xosc";
    const char* Scenario_file = scenario_file.c_str();
    int         i_init        = SE_Init(Scenario_file, 0, 0, 0, 0);
    ASSERT_EQ(i_init, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();

    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
    osi_gt.ParseFromArray(gt, sv_size);

    // order: lane, predecessor, successor
    std::vector<std::vector<int>> lane_pairs = {{0, -1, 3}, {2, -1, 5}, {3, 0, 6}, {5, 2, 8}, {6, 3, 9}, {8, 5, 11}, {9, 6, -1}, {11, 8, -1}};
    int                           successor;
    int                           predecessor;
    int                           gt_successor   = -1;
    int                           gt_predecessor = -1;
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
            successor   = -1;
            for (size_t k = 0; k < lane_pairs.size(); k++)
            {
                if (osi_gt.mutable_lane(i)->id().value() == static_cast<unsigned int>(lane_pairs[k][0]))
                {
                    predecessor = lane_pairs[k][1];
                    successor   = lane_pairs[k][2];
                }
            }

            if (successor == -1 && predecessor == -1)
            {
                ASSERT_EQ(true, false);
            }
            if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
            {
                gt_successor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value());
            }
            else
            {
                gt_successor = -1;
            }
            if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
            {
                gt_predecessor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value());
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
    const char* Scenario_file = scenario_file.c_str();
    int         i_init        = SE_Init(Scenario_file, 0, 0, 0, 0);
    ASSERT_EQ(i_init, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();

    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
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
    int                           successor;
    int                           predecessor;
    int                           gt_successor;
    int                           gt_predecessor;
    for (int i = 0; i < osi_gt.lane_size(); i++)
    {
        if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
        {
            ASSERT_TRUE(false);
        }
        for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
        {
            predecessor = -1;
            successor   = -1;
            for (size_t k = 0; k < lane_pairs.size(); k++)
            {
                if (osi_gt.mutable_lane(i)->id().value() == static_cast<unsigned int>(lane_pairs[k][0]))
                {
                    predecessor = lane_pairs[k][1];
                    successor   = lane_pairs[k][2];
                }
            }

            if (successor == -1 && predecessor == -1)
            {
                ASSERT_EQ(true, false);
            }
            if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
            {
                gt_successor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value());
            }
            else
            {
                gt_successor = -1;
            }
            if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
            {
                gt_predecessor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value());
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
    const char* Scenario_file = scenario_file.c_str();
    int         i_init        = SE_Init(Scenario_file, 0, 0, 0, 0);
    ASSERT_EQ(i_init, 0);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
    osi_gt.ParseFromArray(gt, sv_size);
    // order: lane, predecessor, successor
    std::vector<std::vector<int>> lane_pairs = {{1, -1, 8}, {2, -1, 10}, {8, 1, 4}, {10, 2, 6}, {4, 8, -1}, {6, 10, -1}};

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
            successor   = -1;
            for (size_t k = 0; k < lane_pairs.size(); k++)
            {
                if (osi_gt.mutable_lane(i)->id().value() == static_cast<unsigned int>(lane_pairs[k][0]))
                {
                    predecessor = lane_pairs[k][1];
                    successor   = lane_pairs[k][2];
                }
            }

            if (successor == -1 && predecessor == -1)
            {
                ASSERT_EQ(true, false);
            }
            if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
            {
                gt_successor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value());
            }
            else
            {
                gt_successor = -1;
            }
            if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
            {
                gt_predecessor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value());
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

// rht split is a merge with lht
TEST(OSILaneParing, highway_merge_lht)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/highway_split_lht.xosc";
    const char* Scenario_file = scenario_file.c_str();
    int         i_init        = SE_Init(Scenario_file, 0, 0, 0, 0);
    ASSERT_EQ(i_init, 0);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
    osi_gt.ParseFromArray(gt, sv_size);
    // order: lane, predecessor, successor
    std::vector<std::vector<int>> lane_pairs = {{1, -1, 8}, {2, -1, 10}, {8, 1, 4}, {10, 2, 6}, {4, 8, -1}, {6, 10, -1}};

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
            successor   = -1;
            for (size_t k = 0; k < lane_pairs.size(); k++)
            {
                if (osi_gt.mutable_lane(i)->id().value() == static_cast<unsigned int>(lane_pairs[k][0]))
                {
                    predecessor = lane_pairs[k][1];
                    successor   = lane_pairs[k][2];
                }
            }

            if (successor == -1 && predecessor == -1)
            {
                ASSERT_EQ(true, false);
            }
            if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
            {
                gt_successor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value());
            }
            else
            {
                gt_successor = -1;
            }
            if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
            {
                gt_predecessor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value());
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
    const char* Scenario_file = scenario_file.c_str();
    int         i_init        = SE_Init(Scenario_file, 0, 0, 0, 0);
    ASSERT_EQ(i_init, 0);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
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
    int                           successor;
    int                           predecessor;
    int                           gt_successor;
    int                           gt_predecessor;
    for (int i = 0; i < osi_gt.lane_size(); i++)
    {
        if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
        {
            ASSERT_TRUE(false);
        }
        for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
        {
            predecessor = -1;
            successor   = -1;
            for (size_t k = 0; k < lane_pairs.size(); k++)
            {
                if (osi_gt.mutable_lane(i)->id().value() == static_cast<unsigned int>(lane_pairs[k][0]))
                {
                    predecessor = lane_pairs[k][1];
                    successor   = lane_pairs[k][2];
                }
            }

            if (successor == -1 && predecessor == -1)
            {
                ASSERT_EQ(true, false);
            }
            if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
            {
                gt_successor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value());
            }
            else
            {
                gt_successor = -1;
            }
            if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
            {
                gt_predecessor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value());
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
    const char* Scenario_file = scenario_file.c_str();
    int         i_init        = SE_Init(Scenario_file, 0, 0, 0, 0);
    ASSERT_EQ(i_init, 0);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
    osi_gt.ParseFromArray(gt, sv_size);
    // order: lane, predecessor, successor
    std::vector<std::vector<int>> lane_pairs = {{0, -1, 5},   {1, -1, 6},   {3, -1, 8},   {4, -1, 9},   {5, 0, 11},   {6, 1, 12},
                                                {8, 3, 14},   {9, 4, 15},   {10, -1, 16}, {11, 5, 24},  {12, 6, 25},  {14, 8, 27},
                                                {15, 9, 28},  {16, 10, 30}, {24, 11, 17}, {25, 12, 18}, {27, 14, 20}, {28, 15, 21},
                                                {30, 16, 23}, {17, 24, -1}, {18, 25, -1}, {20, 27, -1}, {21, 28, -1}, {23, 30, -1}};
    int                           successor;
    int                           predecessor;
    int                           gt_successor;
    int                           gt_predecessor;
    for (int i = 0; i < osi_gt.lane_size(); i++)
    {
        if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
        {
            ASSERT_TRUE(false);
        }
        for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
        {
            predecessor = -1;
            successor   = -1;
            for (size_t k = 0; k < lane_pairs.size(); k++)
            {
                if (osi_gt.mutable_lane(i)->id().value() == static_cast<unsigned int>(lane_pairs[k][0]))
                {
                    predecessor = lane_pairs[k][1];
                    successor   = lane_pairs[k][2];
                }
            }

            if (successor == -1 && predecessor == -1)
            {
                ASSERT_EQ(true, false);
            }
            if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
            {
                gt_successor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value());
            }
            else
            {
                gt_successor = -1;
            }
            if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
            {
                gt_predecessor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value());
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
    const char* Scenario_file = scenario_file.c_str();
    int         i_init        = SE_Init(Scenario_file, 0, 0, 0, 0);
    ASSERT_EQ(i_init, 0);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
    osi_gt.ParseFromArray(gt, sv_size);
    // order: lane, predecessor, successor
    std::vector<std::vector<int>> lane_pairs = {{0, 5, 11}, {2, 3, 9}, {3, 2, 6}, {5, 0, 8}, {6, 3, 9}, {8, 5, 11}, {9, 6, 2}, {11, 8, 0}};
    int                           successor;
    int                           predecessor;
    int                           gt_successor;
    int                           gt_predecessor;
    for (int i = 0; i < osi_gt.lane_size(); i++)
    {
        if (osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size() == 0)
        {
            ASSERT_TRUE(false);
        }
        for (int j = 0; j < osi_gt.mutable_lane(i)->mutable_classification()->lane_pairing_size(); j++)
        {
            predecessor = -1;
            successor   = -1;
            for (size_t k = 0; k < lane_pairs.size(); k++)
            {
                if (osi_gt.mutable_lane(i)->id().value() == static_cast<unsigned int>(lane_pairs[k][0]))
                {
                    predecessor = lane_pairs[k][1];
                    successor   = lane_pairs[k][2];
                }
            }

            if (successor == -1 && predecessor == -1)
            {
                ASSERT_EQ(true, false);
            }
            if (successor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_successor_lane_id())
            {
                gt_successor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value());
            }
            else
            {
                gt_successor = -1;
            }
            if (predecessor >= 0 && osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
            {
                gt_predecessor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value());
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
    const char* Scenario_file = scenario_file.c_str();
    int         i_init        = SE_Init(Scenario_file, 0, 0, 0, 0);
    ASSERT_EQ(i_init, 0);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
    osi_gt.ParseFromArray(gt, sv_size);
    // order: lane, predecessor, successor
    std::vector<std::vector<int>> lane_pairs = {{0, -1, 18},
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
                                                {18, 6, 0}};

    std::sort(lane_pairs.begin(), lane_pairs.end());

    int gt_successor;
    int gt_predecessor;

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
                gt_successor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value());
            }
            else
            {
                gt_successor = -1;
            }
            if (osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
            {
                gt_predecessor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value());
            }
            else
            {
                gt_predecessor = -1;
            }

            gt_lane_pairs.push_back({static_cast<int>(osi_gt.lane(i).id().value()), gt_predecessor, gt_successor});
        }
    }
    std::sort(gt_lane_pairs.begin(), gt_lane_pairs.end());

    ASSERT_EQ(gt_lane_pairs.size(), lane_pairs.size());
    for (size_t i = 0; i < lane_pairs.size(); i++)
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
    const char* Scenario_file = scenario_file.c_str();
    int         i_init        = SE_Init(Scenario_file, 0, 0, 0, 0);
    ASSERT_EQ(i_init, 0);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
    osi_gt.ParseFromArray(gt, sv_size);
    // order: lane, predecessor, successor
    std::vector<std::vector<int>> lane_pairs = {{0, -1, 18},
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
                                                {18, 8, 3}};

    std::sort(lane_pairs.begin(), lane_pairs.end());

    int gt_successor;
    int gt_predecessor;

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
                gt_successor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value());
            }
            else
            {
                gt_successor = -1;
            }
            if (osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
            {
                gt_predecessor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value());
            }
            else
            {
                gt_predecessor = -1;
            }

            gt_lane_pairs.push_back({static_cast<int>(osi_gt.lane(i).id().value()), gt_predecessor, gt_successor});
        }
    }
    std::sort(gt_lane_pairs.begin(), gt_lane_pairs.end());

    ASSERT_EQ(gt_lane_pairs.size(), lane_pairs.size());
    for (size_t i = 0; i < lane_pairs.size(); i++)
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
    const char* Scenario_file = scenario_file.c_str();
    int         i_init        = SE_Init(Scenario_file, 0, 0, 0, 0);
    ASSERT_EQ(i_init, 0);

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();
    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
    osi_gt.ParseFromArray(gt, sv_size);
    // order: lane, predecessor, successor
    std::vector<std::vector<int>> lane_pairs = {{0, -1, 30},  {2, -1, 30}, {3, 30, -1}, {5, 30, -1}, {6, 30, -1}, {8, 30, -1}, {9, 30, -1},
                                                {11, 30, -1}, {30, 2, 5},  {30, 2, 8},  {30, 2, 11}, {30, 3, 0},  {30, 3, 8},  {30, 3, 11},
                                                {30, 6, 0},   {30, 6, 5},  {30, 6, 11}, {30, 9, 0},  {30, 9, 5},  {30, 9, 8}};
    std::sort(lane_pairs.begin(), lane_pairs.end());

    int gt_successor;
    int gt_predecessor;

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
                gt_successor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).successor_lane_id().value());
            }
            else
            {
                gt_successor = -1;
            }
            if (osi_gt.lane(i).classification().lane_pairing(j).has_antecessor_lane_id())
            {
                gt_predecessor = static_cast<int>(osi_gt.lane(i).classification().lane_pairing(j).antecessor_lane_id().value());
            }
            else
            {
                gt_predecessor = -1;
            }

            gt_lane_pairs.push_back({static_cast<int>(osi_gt.lane(i).id().value()), gt_predecessor, gt_successor});
        }
    }
    std::sort(gt_lane_pairs.begin(), gt_lane_pairs.end());

    ASSERT_EQ(gt_lane_pairs.size(), lane_pairs.size());
    for (size_t i = 0; i < lane_pairs.size(); i++)
    {
        ASSERT_EQ(gt_lane_pairs[i][0], lane_pairs[i][0]);
        ASSERT_EQ(gt_lane_pairs[i][1], lane_pairs[i][1]);
        ASSERT_EQ(gt_lane_pairs[i][2], lane_pairs[i][2]);
    }

    SE_Close();
}

TEST(OSILaneParing, Signs)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/sign_test.xosc";
    const char* Scenario_file = scenario_file.c_str();

    SE_AddPath("../../../resources");

    int i_init = SE_Init(Scenario_file, 0, 0, 0, 0);
    ASSERT_EQ(i_init, 0);
    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();

    osi3::GroundTruth osi_gt;
    int               sv_size = 0;
    const char*       gt      = SE_GetOSIGroundTruth(&sv_size);
    osi_gt.ParseFromArray(gt, sv_size);
    // order: id, type, country, subtypevalue, text, pitch, roll, height, s, t, zOffset
    std::vector<
        std::tuple<unsigned int, osi3::TrafficSign_MainSign_Classification_Type, double, std::string, double, double, double, double, double, double>>
        signs = {std::make_tuple(0,
                                 osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_DANGER_SPOT,
                                 0,
                                 "",
                                 0.0,
                                 0.0,
                                 0.61,
                                 0.0,
                                 3.57,
                                 1.7),
                 std::make_tuple(1,
                                 osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_DANGER_SPOT,
                                 0,
                                 "",
                                 0.0,
                                 0.0,
                                 0.61,
                                 0.0,
                                 3.57,
                                 1.7),
                 std::make_tuple(2,
                                 osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_ZEBRA_CROSSING,
                                 11,
                                 "",
                                 0.0,
                                 0.0,
                                 0.61,
                                 100.0,
                                 3.57,
                                 1.7),
                 std::make_tuple(3,
                                 osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN,
                                 0,
                                 "",
                                 0.0,
                                 0.0,
                                 0.61,
                                 100.0,
                                 3.57,
                                 1.7),
                 std::make_tuple(4,
                                 osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_HILL_UPWARDS,
                                 0,
                                 "",
                                 0.0,
                                 0.0,
                                 0.61,
                                 100.0,
                                 3.57,
                                 1.7),
                 std::make_tuple(5,
                                 osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_DOUBLE_TURN_LEFT,
                                 10,
                                 "",
                                 0.0,
                                 0.0,
                                 0.61,
                                 100.0,
                                 3.57,
                                 1.7),
                 std::make_tuple(6,
                                 osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_DOUBLE_TURN_RIGHT,
                                 20,
                                 "",
                                 0.0,
                                 0.0,
                                 0.61,
                                 200.0,
                                 3.57,
                                 1.7),
                 std::make_tuple(7,
                                 osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN,
                                 0,
                                 "",
                                 0.0,
                                 0.0,
                                 0.61,
                                 200.0,
                                 3.57,
                                 1.7),
                 std::make_tuple(8,
                                 osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN,
                                 0,
                                 "",
                                 0.0,
                                 0.0,
                                 0.61,
                                 200.0,
                                 3.57,
                                 1.7),
                 std::make_tuple(9,
                                 osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN,
                                 0,
                                 "",
                                 0.0,
                                 0.0,
                                 0.61,
                                 200.0,
                                 3.57,
                                 1.7),
                 std::make_tuple(10,
                                 osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN,
                                 0,
                                 "",
                                 0.0,
                                 0.0,
                                 0.61,
                                 500.0,
                                 3.57,
                                 1.7),
                 std::make_tuple(11,
                                 osi3::TrafficSign_MainSign_Classification_Type::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN,
                                 0,
                                 "",
                                 0.0,
                                 0.0,
                                 0.61,
                                 500.0,
                                 3.57,
                                 1.7)};

    unsigned int                                   sign_id = 0;
    osi3::TrafficSign_MainSign_Classification_Type type    = osi3::TrafficSign_MainSign_Classification_Type_TYPE_UNKNOWN;
    double                                         value   = 0;
    std::string                                    text    = "";
    double                                         pitch   = 0;
    double                                         roll    = 0;
    double                                         height  = 0;

    for (auto traffic_sign : osi_gt.traffic_sign())
    {
        for (auto sign : signs)
        {
            if (traffic_sign.id().value() == std::get<0>(sign))
            {
                sign_id = std::get<0>(sign);
                type    = std::get<1>(sign);
                value   = std::get<2>(sign);
                text    = std::get<3>(sign);
                pitch   = std::get<4>(sign);
                roll    = std::get<5>(sign);
                height  = std::get<6>(sign);
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

static void ReadDat(std::string filename, std::vector<scenarioengine::ReplayEntry>& entries)
{
    std::ifstream             file;
    scenarioengine::DatHeader header;

    file.open(filename, std::ofstream::binary);
    ASSERT_EQ(file.fail(), false);

    file.read(reinterpret_cast<char*>(&header), sizeof(header));

    scenarioengine::ReplayEntry entry;

    while (!file.eof())
    {
        file.read(reinterpret_cast<char*>(&entry.state), sizeof(entry.state));

        if (!file.eof())
        {
            entries.push_back(entry);
        }
    }
    file.close();
}

TEST(ExternalControlTest, TestTimings)
{
    // This test case imitates a custom application controlling the Ego vehicle
    // It makes use of the esmini ghost feature to provide input to a driver model
    // Ego vehicle state is reported for each time step.

    SE_ScenarioObjectState ego_state;
    SE_RoadInfo            road_info;
    float                  duration = 10.0f;
    float                  dt       = 0.1f;
    float                  ghost_speed;

    const char* args[2][10] = {{"--osc",
                                "../../../EnvironmentSimulator/Unittest/xosc/timing_scenario0.xosc",
                                "--record",
                                "sim.dat",
                                "--fixed_timestep",
                                "0.1",
                                //"--window", "60", "60", "800", "400",
                                "--csv_logger",
                                "csv_log.csv",
                                "--osi_file",
                                "gt.osi"},
                               {"--osc",
                                "../../../EnvironmentSimulator/Unittest/xosc/timing_scenario_with_restarts.xosc",
                                "--record",
                                "sim.dat",
                                "--fixed_timestep",
                                "0.1",
                                //"--window", "60", "60", "800", "400",
                                "--csv_logger",
                                "csv_log.csv",
                                "--osi_file",
                                "gt.osi"}};

    SE_AddPath("../../../resources/xodr");

    for (int j = 0; j < 2; j++)
    {
        ASSERT_EQ(SE_InitWithArgs(sizeof(args[j]) / sizeof(char*), args[j]), 0);
        ASSERT_EQ(SE_GetNumberOfObjects(), 3);

        // show some road features, including road sensor
        SE_ViewerShowFeature(4 + 8, true);  // NODE_MASK_TRAIL_DOTS (1 << 2) & NODE_MASK_ODR_FEATURES (1 << 3),

        // Initialize the vehicle model, fetch initial state from the scenario
        SE_GetObjectState(0, &ego_state);

        while (SE_GetSimulationTime() < duration && SE_GetQuitFlag() != 1)
        {
            // After ghost restart, check polyline returnvalues
            if (j == 1 && NEAR_NUMBERS(SE_GetSimulationTimeDouble(), 2.2))
            {
                // try too small timestamp not preset in ghost trail which is now defined between 2.25 and 5.1
                EXPECT_EQ(SE_GetRoadInfoGhostTrailTime(0, 2.25f - SMALL_NUMBERF, &road_info, &ghost_speed),
                          SE_GhostTrailReturnCode::SE_GHOST_TRAIL_TIME_PRIOR);

                // try too large timestamp
                EXPECT_EQ(SE_GetRoadInfoGhostTrailTime(0, 5.10f + SMALL_NUMBERF, &road_info, &ghost_speed),
                          SE_GhostTrailReturnCode::SE_GHOST_TRAIL_TIME_PAST);

                // try non existing ghost
                EXPECT_EQ(SE_GetRoadInfoGhostTrailTime(1, 5.10f + SMALL_NUMBERF, &road_info, &ghost_speed),
                          SE_GhostTrailReturnCode::SE_GHOST_TRAIL_ERROR);
            }

            // Copy position and heading from ghost at next timestamp
            EXPECT_EQ(SE_GetRoadInfoGhostTrailTime(0, SE_GetSimulationTime() + dt, &road_info, &ghost_speed),
                      SE_GhostTrailReturnCode::SE_GHOST_TRAIL_OK);

            ego_state.x = road_info.global_pos_x + 100;
            ego_state.y = road_info.global_pos_y;
            ego_state.h = road_info.trail_heading;

            SE_ReportObjectPosXYH(0, 0, ego_state.x, ego_state.y, ego_state.h);
            SE_ReportObjectSpeed(0, ghost_speed);

            // Finally, update scenario using same time step as for vehicle model
            SE_StepDT(dt);
        }

        SE_Close();

        // Check .dat file
        std::vector<scenarioengine::ReplayEntry> entries;
        ReadDat("sim.dat", entries);

        // Check first timestep (-3.0)
        size_t i = 0;
        EXPECT_NEAR(entries[i].state.info.timeStamp, -3.0, 1E-3);
        EXPECT_STREQ(entries[i].state.info.name, "Ego");
        EXPECT_NEAR(entries[i].state.pos.x, 10.0, 1E-3);
        EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
        i++;
        EXPECT_NEAR(entries[i].state.info.timeStamp, -3.0, 1E-3);
        EXPECT_STREQ(entries[i].state.info.name, "Target");
        EXPECT_NEAR(entries[i].state.pos.x, 10.0, 1E-3);
        EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
        i++;
        EXPECT_NEAR(entries[i].state.info.timeStamp, -3.0, 1E-3);
        EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
        EXPECT_NEAR(entries[i].state.pos.x, 10.0, 1E-3);
        EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

        // Check timestep before 0.0
        while (i < entries.size() - 1 && entries[i].state.info.timeStamp < static_cast<float>(-SMALL_NUMBER))
            i++;
        i -= 3;
        EXPECT_NEAR(entries[i].state.info.timeStamp, -0.05f, 1E-3);
        EXPECT_STREQ(entries[i].state.info.name, "Ego");
        EXPECT_NEAR(entries[i].state.pos.x, 10.0, 1E-3);
        EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
        i++;
        EXPECT_NEAR(entries[i].state.info.timeStamp, -0.05f, 1E-3);
        EXPECT_STREQ(entries[i].state.info.name, "Target");
        EXPECT_NEAR(entries[i].state.pos.x, 10.0, 1E-3);
        EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
        i++;
        EXPECT_NEAR(entries[i].state.info.timeStamp, -0.05f, 1E-3);
        EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
        EXPECT_NEAR(entries[i].state.pos.x, 39.5, 1E-3);
        EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

        // Check timestep 0.0
        i++;
        EXPECT_NEAR(entries[i].state.info.timeStamp, 0.0, 1E-3);
        EXPECT_STREQ(entries[i].state.info.name, "Ego");
        EXPECT_NEAR(entries[i].state.pos.x, 10.0, 1E-3);
        EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
        i++;
        EXPECT_NEAR(entries[i].state.info.timeStamp, 0.0, 1E-3);
        EXPECT_STREQ(entries[i].state.info.name, "Target");
        EXPECT_NEAR(entries[i].state.pos.x, 10.0, 1E-3);
        EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
        i++;
        EXPECT_NEAR(entries[i].state.info.timeStamp, 0.0, 1E-3);
        EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
        EXPECT_NEAR(entries[i].state.pos.x, 40.0, 1E-3);
        EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

        // Check timestep after 0.0
        i++;
        EXPECT_NEAR(entries[i].state.info.timeStamp, dt, 1E-3);
        EXPECT_STREQ(entries[i].state.info.name, "Ego");
        EXPECT_NEAR(entries[i].state.pos.x, 111.0, 1E-3);
        EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
        i++;
        EXPECT_NEAR(entries[i].state.info.timeStamp, dt, 1E-3);
        EXPECT_STREQ(entries[i].state.info.name, "Target");
        EXPECT_NEAR(entries[i].state.pos.x, 12.0, 1E-3);
        EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
        i++;
        EXPECT_NEAR(entries[i].state.info.timeStamp, dt, 1E-3);
        EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
        EXPECT_NEAR(entries[i].state.pos.x, 41.0, 1E-3);
        EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

        if (j == 1)  // additional restart tests
        {
            // Check first restart
            i = 243;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 2.1, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego");
            EXPECT_NEAR(entries[i].state.pos.x, 131.0, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 2.1, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Target");
            EXPECT_NEAR(entries[i].state.pos.x, 52.0, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 2.1, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
            EXPECT_NEAR(entries[i].state.pos.x, 61.0, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, -0.75, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego");
            EXPECT_NEAR(entries[i].state.pos.x, 132.0, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, -0.75, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Target");
            EXPECT_NEAR(entries[i].state.pos.x, 54.0, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, -0.75, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
            EXPECT_NEAR(entries[i].state.pos.x, 131.502, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, -0.70, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego");
            EXPECT_NEAR(entries[i].state.pos.x, 132.0, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, -0.70, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Target");
            EXPECT_NEAR(entries[i].state.pos.x, 54.0, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, -0.70, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
            EXPECT_NEAR(entries[i].state.pos.x, 132.005, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

            i = 423;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 2.2, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego");
            EXPECT_NEAR(entries[i].state.pos.x, 132.0, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 2.2, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Target");
            EXPECT_NEAR(entries[i].state.pos.x, 54.0, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 2.2, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
            EXPECT_NEAR(entries[i].state.pos.x, 169.124, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 2.3, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego");
            EXPECT_NEAR(entries[i].state.pos.x, 232.008, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 2.3, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Target");
            EXPECT_NEAR(entries[i].state.pos.x, 56.0, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 2.3, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
            EXPECT_NEAR(entries[i].state.pos.x, 170.624, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

            i = 600;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 8.1, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego");
            EXPECT_NEAR(entries[i].state.pos.x, 312.624, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 8.1, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Target");
            EXPECT_NEAR(entries[i].state.pos.x, 172.000, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 8.1, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
            EXPECT_NEAR(entries[i].state.pos.x, 257.624, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 5.25, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego");
            EXPECT_NEAR(entries[i].state.pos.x, 314.124, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 5.25, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Target");
            EXPECT_NEAR(entries[i].state.pos.x, 174.000, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 5.25, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
            EXPECT_NEAR(entries[i].state.pos.x, 313.376, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

            i = 774;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 8.1, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego");
            EXPECT_NEAR(entries[i].state.pos.x, 314.124, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 8.1, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Target");
            EXPECT_NEAR(entries[i].state.pos.x, 174.000, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 8.1, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
            EXPECT_NEAR(entries[i].state.pos.x, 363.748, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

            i = 780;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 8.2, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego");
            EXPECT_NEAR(entries[i].state.pos.x, 314.124, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 8.2, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Target");
            EXPECT_NEAR(entries[i].state.pos.x, 174.000, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 8.2, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
            EXPECT_NEAR(entries[i].state.pos.x, 365.748, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 8.3, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego");
            EXPECT_NEAR(entries[i].state.pos.x, 414.133, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 8.3, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Target");
            EXPECT_NEAR(entries[i].state.pos.x, 176.000, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -4.5, 1E-3);
            i++;
            EXPECT_NEAR(entries[i].state.info.timeStamp, 8.3, 1E-3);
            EXPECT_STREQ(entries[i].state.info.name, "Ego_ghost");
            EXPECT_NEAR(entries[i].state.pos.x, 367.748, 1E-3);
            EXPECT_NEAR(entries[i].state.pos.y, -1.5, 1E-3);

            // Also check a few entries in the csv log file, focus on scenario controlled entity "Target"
            std::vector<std::vector<std::string>> csv;
            ASSERT_EQ(SE_ReadCSVFile("csv_log.csv", csv, 6), 0);
            EXPECT_NEAR(std::stod(csv[1][1]), -3.0, 1E-3);
            EXPECT_NEAR(std::stod(csv[61][44]), 10.0, 1E-3);
            EXPECT_NEAR(std::stod(csv[61][47]), 20.0, 1E-3);
            EXPECT_NEAR(std::stod(csv[61][50]), 0.0, 1E-3);
            EXPECT_NEAR(std::stod(csv[82][1]), 2.1, 1E-3);
            EXPECT_NEAR(std::stod(csv[82][44]), 52.0, 1E-3);
            EXPECT_NEAR(std::stod(csv[82][47]), 20.0, 1E-3);
            EXPECT_NEAR(std::stod(csv[82][50]), 0.0, 1E-3);
            EXPECT_NEAR(std::stod(csv[83][1]), -0.75, 1E-3);
            EXPECT_NEAR(std::stod(csv[83][44]), 54.0, 1E-3);
            EXPECT_NEAR(std::stod(csv[83][47]), 20.0, 1E-3);
            EXPECT_NEAR(std::stod(csv[83][50]), 0.0, 1E-3);
            EXPECT_NEAR(std::stod(csv[142][1]), 2.2, 1E-3);
            EXPECT_NEAR(std::stod(csv[142][44]), 54.0, 1E-3);
            EXPECT_NEAR(std::stod(csv[142][47]), 20.0, 1E-3);
            EXPECT_NEAR(std::stod(csv[142][50]), 0.0, 1E-3);

            // Read OSI file
            FILE* file = FileOpen("gt.osi", "rb");
            ASSERT_NE(file, nullptr);

            const int         max_msg_size = 10000;
            char              msg_buf[max_msg_size];
            int               msg_size;
            osi3::GroundTruth osi_gt;
            double            seconds = -1.0;

            double time_stamps[] = {0.0, 2.1, 2.2};

            // Focus on position and acceleration
            double pos_x[3][3] = {// for three timestamps and three entities
                                  {11.4, 11.3, 41.4},
                                  {132.4, 53.3, 62.4},
                                  {133.4, 55.3, 170.524}};

            double acc_x[3][3] = {// for three timestamps and three entities
                                  {0.0, 0.0, 0.0},
                                  {0.0, 0.0, 0.0},
                                  {0.0, 0.0, 0.0}};

            // Read OSI message size
            ASSERT_EQ(fread(reinterpret_cast<char*>(&msg_size), 1, sizeof(msg_size), file), sizeof(msg_size));
            ASSERT_LE(msg_size, max_msg_size);

            // Read first OSI message
            EXPECT_EQ(fread(msg_buf, 1, static_cast<size_t>(msg_size), file), static_cast<size_t>(msg_size));
            osi_gt.ParseFromArray(msg_buf, msg_size);

            seconds = static_cast<double>(osi_gt.mutable_timestamp()->seconds()) + 1E-9 * static_cast<double>(osi_gt.mutable_timestamp()->nanos());

            EXPECT_NEAR(seconds, time_stamps[0], 1E-3);

            EXPECT_EQ(osi_gt.mutable_moving_object()->size(), 3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(0)->mutable_base()->mutable_position()->x(), pos_x[0][0], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(1)->mutable_base()->mutable_position()->x(), pos_x[0][1], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(2)->mutable_base()->mutable_position()->x(), pos_x[0][2], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(0)->mutable_base()->mutable_acceleration()->x(), acc_x[0][0], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(1)->mutable_base()->mutable_acceleration()->x(), acc_x[0][1], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(2)->mutable_base()->mutable_acceleration()->x(), acc_x[0][2], 1E-3);

            // fast forward until 2.0 seconds
            while (seconds < 2.0 - SMALL_NUMBER)
            {
                // Read OSI message size
                ASSERT_EQ(fread(reinterpret_cast<char*>(&msg_size), 1, sizeof(msg_size), file), sizeof(msg_size));
                ASSERT_LE(msg_size, max_msg_size);

                // Read OSI message
                EXPECT_EQ(fread(msg_buf, 1, static_cast<size_t>(msg_size), file), static_cast<size_t>(msg_size));
                osi_gt.ParseFromArray(msg_buf, msg_size);

                seconds =
                    static_cast<double>(osi_gt.mutable_timestamp()->seconds()) + 1E-9 * static_cast<double>(osi_gt.mutable_timestamp()->nanos());
            }

            // Read second OSI message (should be at 2.1s)
            ASSERT_EQ(fread(reinterpret_cast<char*>(&msg_size), 1, sizeof(msg_size), file), sizeof(msg_size));
            ASSERT_LE(msg_size, max_msg_size);
            EXPECT_EQ(fread(msg_buf, 1, static_cast<size_t>(msg_size), file), static_cast<size_t>(msg_size));
            osi_gt.ParseFromArray(msg_buf, msg_size);

            seconds = static_cast<double>(osi_gt.mutable_timestamp()->seconds()) + 1E-9 * static_cast<double>(osi_gt.mutable_timestamp()->nanos());

            EXPECT_NEAR(seconds, time_stamps[1], 1E-3);

            EXPECT_EQ(osi_gt.mutable_moving_object()->size(), 3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(0)->mutable_base()->mutable_position()->x(), pos_x[1][0], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(1)->mutable_base()->mutable_position()->x(), pos_x[1][1], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(2)->mutable_base()->mutable_position()->x(), pos_x[1][2], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(0)->mutable_base()->mutable_acceleration()->x(), acc_x[1][0], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(1)->mutable_base()->mutable_acceleration()->x(), acc_x[1][1], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(2)->mutable_base()->mutable_acceleration()->x(), acc_x[1][2], 1E-3);

            // Read third OSI message (should be at 2.2s)
            ASSERT_EQ(fread(reinterpret_cast<char*>(&msg_size), 1, sizeof(msg_size), file), sizeof(msg_size));
            ASSERT_LE(msg_size, max_msg_size);
            EXPECT_EQ(fread(msg_buf, 1, static_cast<size_t>(msg_size), file), static_cast<size_t>(msg_size));
            osi_gt.ParseFromArray(msg_buf, msg_size);

            seconds = static_cast<double>(osi_gt.mutable_timestamp()->seconds()) + 1E-9 * static_cast<double>(osi_gt.mutable_timestamp()->nanos());

            EXPECT_NEAR(seconds, time_stamps[2], 1E-3);

            EXPECT_EQ(osi_gt.mutable_moving_object()->size(), 3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(0)->mutable_base()->mutable_position()->x(), pos_x[2][0], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(1)->mutable_base()->mutable_position()->x(), pos_x[2][1], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(2)->mutable_base()->mutable_position()->x(), pos_x[2][2], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(0)->mutable_base()->mutable_acceleration()->x(), acc_x[2][0], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(1)->mutable_base()->mutable_acceleration()->x(), acc_x[2][1], 1E-3);
            EXPECT_NEAR(osi_gt.mutable_moving_object(2)->mutable_base()->mutable_acceleration()->x(), acc_x[2][2], 1E-3);

            fclose(file);
        }
    }
}

#ifndef _OSI_VERSION_3_3_1
TEST(TestOsiReporter, AssignRoleTest)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/emergency_vehicle.xosc";
    const char* Scenario_file = scenario_file.c_str();
    int         i_init        = SE_Init(Scenario_file, 0, 0, 0, 0);
    ASSERT_EQ(i_init, 0);

    const osi3::GroundTruth* osi_gt;
    osi_gt = reinterpret_cast<const osi3::GroundTruth*>(SE_GetOSIGroundTruthRaw());

    SE_StepDT(0.001f);
    SE_UpdateOSIGroundTruth();

    EXPECT_EQ(osi_gt->moving_object(1).vehicle_classification().role(), osi3::MovingObject_VehicleClassification_Role_ROLE_POLICE);

    EXPECT_EQ(strcmp(SE_GetObjectModelFileName(1), "car_police.osgb"), 0);

    SE_Close();
}
#endif

#endif  // _USE_OSI

TEST(ParameterTest, GetTypedParameterValues)
{
    std::string scenario_file = "../../../resources/xosc/lane_change.xosc";
    SE_Init(scenario_file.c_str(), 0, 0, 0, 0);

    bool boolVar;
    int  retVal;
    retVal = SE_GetParameterBool("DummyParameter2", &boolVar);
    EXPECT_EQ(retVal, 0);
    EXPECT_EQ(boolVar, true);

    // Use common SE_Parameter class
    SE_Parameter param;
    param.name  = "DummyParameter2";
    bool value  = false;
    param.value = reinterpret_cast<void*>(&value);
    retVal      = SE_GetParameter(&param);
    EXPECT_EQ(retVal, 0);
    value = *reinterpret_cast<bool*>(param.value);
    EXPECT_EQ(value, true);

    value  = false;
    retVal = SE_SetParameter(param);
    EXPECT_EQ(retVal, 0);

    value  = true;
    retVal = SE_GetParameter(&param);
    EXPECT_EQ(retVal, 0);
    value = *reinterpret_cast<bool*>(param.value);
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
    const char* strVar;
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

TEST(VariableTest, GetTypedVariableValues)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/lane_change_trig_by_variable.xosc";
    EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);

    bool boolVar;
    int  retVal;
    retVal = SE_GetVariableBool("DummyVariable2", &boolVar);
    EXPECT_EQ(retVal, 0);
    EXPECT_EQ(boolVar, true);

    // Use common SE_Variable class
    SE_Variable var;
    var.name   = "DummyVariable2";
    bool value = false;
    var.value  = reinterpret_cast<void*>(&value);
    retVal     = SE_GetVariable(&var);
    EXPECT_EQ(retVal, 0);
    value = *reinterpret_cast<bool*>(var.value);
    EXPECT_EQ(value, true);

    value  = false;
    retVal = SE_SetVariable(var);
    EXPECT_EQ(retVal, 0);

    value  = true;
    retVal = SE_GetVariable(&var);
    EXPECT_EQ(retVal, 0);
    value = *reinterpret_cast<bool*>(var.value);
    EXPECT_EQ(value, false);

    // Unavailable
    retVal = SE_GetVariableBool("DoesNotExist", &boolVar);
    EXPECT_EQ(retVal, -1);

    // Set value
    SE_SetVariableBool("DummyVariable2", false);
    retVal = SE_GetVariableBool("DummyVariable2", &boolVar);
    EXPECT_EQ(retVal, 0);
    EXPECT_EQ(boolVar, false);

    // Wrong name
    retVal = SE_SetVariableBool("DummyVariable3", false);
    EXPECT_EQ(retVal, -1);

    // Wrong type
    retVal = SE_SetVariableInt("DummyVariable2", false);
    EXPECT_EQ(retVal, -1);

    // String
    const char* strVar;
    retVal = SE_GetVariableString("DummyVariable3", &strVar);
    EXPECT_EQ(retVal, 0);
    EXPECT_STREQ(strVar, "lane_change_var3");

    retVal = SE_SetVariableString("DummyVariable3", "Kalle");
    EXPECT_EQ(retVal, 0);
    retVal = SE_GetVariableString("DummyVariable3", &strVar);
    EXPECT_EQ(retVal, 0);
    EXPECT_STREQ(strVar, "Kalle");

    retVal = SE_GetParameterString("DoesNotExist", &strVar);
    EXPECT_EQ(retVal, -1);

    // verify no mixup between variable and parameter
    EXPECT_EQ(SE_GetParameterString("DummyParameter4", &strVar), 0);
    EXPECT_STREQ(strVar, "lane_change_param4");

    EXPECT_EQ(SE_GetVariableString("DummyParameter4", &strVar), 0);
    EXPECT_STREQ(strVar, "lane_change_var5");

    SE_Close();
}

static void paramDeclCallback(void*)
{
    static int counter  = 0;
    double     value[2] = {1.1, 1.5};

    if (counter < 2)
    {
        SE_SetParameterDouble("TargetSpeedFactor", value[counter]);
    }

    counter++;
}

TEST(ParameterTest, SetParameterValuesBeforeInit)
{
    double positions[3][2] = {
        {5.34382, 186.68216},  // TargetSpeedFactor = 1.1
        {8.83781, 240.59825},  // TargetSpeedFactor = 1.5
        {5.46731, 201.38162}   // TargetSpeedFactor = Default = 1.2
    };
    SE_ScenarioObjectState state;

    std::string scenario_file = "../../../resources/xosc/cut-in.xosc";

    for (int i = 0; i < 3 && SE_GetQuitFlag() != 1; i++)
    {
        SE_RegisterParameterDeclarationCallback(paramDeclCallback, 0);
        ASSERT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);
        ASSERT_EQ(SE_GetNumberOfObjects(), 2);

        while (SE_GetSimulationTime() < 5.0f && SE_GetQuitFlag() != 1)
        {
            SE_StepDT(0.1f);
        }

        // Check position of second vehicle
        SE_GetObjectState(1, &state);
        EXPECT_NEAR(state.x, positions[i][0], 1e-5);
        EXPECT_NEAR(state.y, positions[i][1], 1e-5);

        SE_Close();
    }
    SE_RegisterParameterDeclarationCallback(0, 0);
}

TEST(TestGetAndSet, OverrideActionTest)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/override_action.xosc";
    const char* Scenario_file = scenario_file.c_str();
    float       dt            = 0.1f;
    float       t             = 0.0f;

    SE_OverrideActionList list;

    ASSERT_EQ(SE_Init(Scenario_file, 0, 0, 0, 0), 0);

    EXPECT_EQ(SE_GetOverrideActionStatus(0, &list), 0);
    EXPECT_EQ(list.throttle.active, false);
    EXPECT_DOUBLE_EQ(list.throttle.value, 0.0);

    for (; t < 2.5f; t += dt)
    {
        SE_StepDT(dt);
    }
    // Should still be no changes
    EXPECT_EQ(SE_GetOverrideActionStatus(0, &list), 0);
    EXPECT_EQ(list.throttle.active, false);
    EXPECT_DOUBLE_EQ(list.throttle.value, 0.0);

    for (; t < 3.1f; t += dt)
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

    for (; t < 5.1f; t += dt)
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

    for (; t < 7.1f; t += dt)
    {
        SE_StepDT(dt);
    }
    EXPECT_EQ(SE_GetOverrideActionStatus(0, &list), 0);
    EXPECT_EQ(list.throttle.active, false);
    EXPECT_DOUBLE_EQ(list.throttle.value, 0.1);

    SE_Close();
}

TEST(TestGetAndSet, PropertyTest)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/VehiclePropertyTest.xosc";
    const char* Scenario_file = scenario_file.c_str();
    SE_Init(Scenario_file, 0, 0, 0, 0);

    // object 0, 3 properties
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
    // object 1, 4 properties
    //     <Property name="model_id" value="1"/>
    //     <Property name="dummy1_1" value="2"/>
    //     <Property name="dummy1_2" value="3"/>
    //     <Property name="dummy1_3" value="4"/>
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

    // object 2, 0 properties
    EXPECT_EQ(SE_GetNumberOfProperties(2), 0);
    EXPECT_STREQ(SE_GetObjectPropertyName(2, 0), "");
    EXPECT_STREQ(SE_GetObjectPropertyValue(2, "model_id"), "");

    // object -1 and 3, does not exist
    EXPECT_EQ(SE_GetNumberOfProperties(3), -1);
    EXPECT_EQ(SE_GetNumberOfProperties(-1), -1);

    SE_Close();
}

TEST(RoadSign, TestValidityRecord)
{
    std::string scenario_file = "../../../resources/xosc/distance_test.xosc";
    EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);

    int n_Objects = SE_GetNumberOfObjects();
    EXPECT_EQ(n_Objects, 2);

    EXPECT_EQ(SE_GetNumberOfRoadSigns(1), 15);
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

TEST(ObjectIds, check_ids)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/init_test_objects_strange_order.xosc";
    const char* Scenario_file = scenario_file.c_str();
    int         i_init        = SE_Init(Scenario_file, 0, 0, 0, 0);
    ASSERT_EQ(i_init, 0);
    SE_StepDT(0.001f);
    ASSERT_EQ(SE_GetId(0), 0);
    ASSERT_EQ(SE_GetId(1), 2);
    ASSERT_EQ(SE_GetId(2), 1);

    SE_Close();
}

void objectCallback(SE_ScenarioObjectState* state, void*)
{
    SE_ReportObjectRoadPos(state->id, state->timestamp, state->roadId, 5, -2.3f, state->s);
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

    SE_Close();
}

static void ghostParamDeclCB(void* user_arg)
{
    bool ghostMode = *reinterpret_cast<bool*>(user_arg);

    SE_SetParameterBool("GhostMode", ghostMode);
}

TEST(ExternalController, TestExternalDriver)
{
    const double defaultTargetSpeed = 50.0;
    const double curveWeight        = 30.0;
    const double throttleWeight     = 0.1;
    const float  dt                 = 0.05f;
    const float  duration           = 35.0f;
    bool         ghostMode[3]       = {false, true, true};

    void*                  vehicleHandle = 0;
    SE_SimpleVehicleState  vehicleState  = {0, 0, 0, 0, 0, 0, 0, 0};
    SE_ScenarioObjectState objectState;
    SE_RoadInfo            roadInfo;

    SE_AddPath("../../../resources/xodr");
    SE_AddPath("../../../resources/xosc/Catalogs/Vehicles");

    for (int i = 0; i < 3; i++)
    {
        SE_RegisterParameterDeclarationCallback(ghostParamDeclCB, &ghostMode[i]);

        ASSERT_EQ(SE_Init("../../../EnvironmentSimulator/code-examples/test-driver/test-driver.xosc", 0, 0, 0, 0), 0);

        // Lock object to the original lane
        // If setting to false, the object road position will snap to closest lane
        SE_SetLockOnLane(0, true);

        // Initialize the vehicle model, fetch initial state from the scenario
        SE_GetObjectState(0, &objectState);
        vehicleHandle = SE_SimpleVehicleCreate(objectState.x, objectState.y, objectState.h, 4.0, 0.0);
        SE_SimpleVehicleSteeringRate(vehicleHandle, 8.0f);

        // show some road features, including road sensor
        SE_ViewerShowFeature(4 + 8, true);  // NODE_MASK_TRAIL_DOTS (1 << 2) & NODE_MASK_ODR_FEATURES (1 << 3),

        // Run for 40 seconds or until 'Esc' button is pressed
        while (SE_GetSimulationTime() < duration && SE_GetQuitFlag() != 1)
        {
            // Get road information at a point some speed dependent distance ahead
            double targetSpeed;
            if (ghostMode[i] == true)
            {
                // ghost version
                float ghost_speed = 0.0;
                float timestamp   = 0.0;
                if (i < 2)
                {
                    SE_GetRoadInfoAlongGhostTrail(0, 5 + 0.75f * vehicleState.speed, &roadInfo, &ghost_speed, &timestamp);
                }
                else
                {
                    SE_GetRoadInfoGhostTrailTime(0, SE_GetSimulationTime() + 0.25f, &roadInfo, &ghost_speed);
                }
                targetSpeed = ghost_speed;
            }
            else
            {
                // Look ahead along the road, to establish target info for the driver model
                SE_GetRoadInfoAtDistance(0, 5 + 0.75f * vehicleState.speed, &roadInfo, 0, true);

                // Slow down when curve ahead - CURVE_WEIGHT is the tuning parameter
                targetSpeed = defaultTargetSpeed / (1 + curveWeight * fabs(static_cast<double>(roadInfo.angle)));
            }

            // Steer towards where the point
            double steerAngle = roadInfo.angle;

            // Accelerate or decelerate towards target speed - THROTTLE_WEIGHT tunes magnitude
            double throttle = throttleWeight * (targetSpeed - static_cast<double>(vehicleState.speed));

            // Step vehicle model with driver input, but wait until time > 0
            if (SE_GetSimulationTime() > static_cast<float>(SMALL_NUMBER) && !SE_GetPauseFlag())
            {
                SE_SimpleVehicleControlAnalog(vehicleHandle, dt, throttle, steerAngle);
            }

            // Fetch updated state and report to scenario engine
            SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);

            if (i == 0)
            {
                if (abs(SE_GetSimulationTime() - 11.0f) < static_cast<float>(SMALL_NUMBER))
                {
                    SE_GetObjectState(0, &objectState);
                    EXPECT_NEAR(objectState.x, 215.890, 1e-3);
                    EXPECT_NEAR(objectState.y, 113.7796, 1e-3);
                    EXPECT_NEAR(objectState.h, 1.362, 1e-3);
                    EXPECT_NEAR(objectState.p, 6.246, 1e-3);
                }
                else if (abs(SE_GetSimulationTime() - 30.0f) < static_cast<float>(SMALL_NUMBER))
                {
                    SE_GetObjectState(0, &objectState);
                    EXPECT_NEAR(objectState.x, 356.1803, 1e-3);
                    EXPECT_NEAR(objectState.y, 330.0853, 1e-3);
                    EXPECT_NEAR(objectState.h, 5.641, 1e-3);
                    EXPECT_NEAR(objectState.p, 0.046, 1e-3);
                }
            }
            else if (i == 1)
            {
                float speed2 = 0;
                if (abs(SE_GetSimulationTime() - 11.0f) < static_cast<float>(SMALL_NUMBER))
                {
                    SE_GetObjectState(0, &objectState);
                    EXPECT_NEAR(objectState.x, 202.472, 1e-3);
                    EXPECT_NEAR(objectState.y, 83.024, 1e-3);
                    EXPECT_NEAR(objectState.h, 1.134, 1e-3);
                    EXPECT_NEAR(objectState.p, 6.262, 1e-3);
                    if (ghostMode[i] == true)
                    {
                        SE_RoadInfo road_info2;
                        SE_GetRoadInfoGhostTrailTime(0, SE_GetSimulationTime(), &road_info2, &speed2);
                        EXPECT_NEAR(road_info2.global_pos_x, 206.716, 1e-3);
                        EXPECT_NEAR(road_info2.global_pos_y, 92.447, 1e-3);
                        EXPECT_NEAR(roadInfo.trail_heading, 1.2158, 1e-3);
                    }
                }
                else if (abs(SE_GetSimulationTime() - 30.0f) < static_cast<float>(SMALL_NUMBER))
                {
                    SE_GetObjectState(0, &objectState);
                    EXPECT_NEAR(objectState.x, 382.113, 1e-3);
                    EXPECT_NEAR(objectState.y, 301.632, 1e-3);
                    EXPECT_NEAR(objectState.h, 5.272, 1e-3);
                    EXPECT_NEAR(objectState.p, 0.025, 1e-3);
                    if (ghostMode[i] == true)
                    {
                        SE_RoadInfo road_info3;
                        SE_GetRoadInfoGhostTrailTime(0, SE_GetSimulationTime(), &road_info3, &speed2);
                        EXPECT_NEAR(road_info3.global_pos_x, 388.2154, 1e-3);
                        EXPECT_NEAR(road_info3.global_pos_y, 291.2665, 1e-3);
                        EXPECT_NEAR(roadInfo.trail_heading, 5.1278, 1e-3);
                    }
                }
            }
            else if (i == 2)
            {
                SE_RoadInfo road_info2;
                float       speed3 = 0;
                if (abs(SE_GetSimulationTime() - 11.0f) < static_cast<float>(SMALL_NUMBER))
                {
                    SE_GetObjectState(0, &objectState);
                    EXPECT_NEAR(objectState.x, 203.203, 1e-3);
                    EXPECT_NEAR(objectState.y, 84.484, 1e-3);
                    EXPECT_NEAR(objectState.h, 1.142, 1e-3);
                    EXPECT_NEAR(objectState.p, 6.262, 1e-3);
                    if (ghostMode[i] == true)
                    {
                        SE_GetRoadInfoGhostTrailTime(0, SE_GetSimulationTime(), &road_info2, &speed3);
                        EXPECT_NEAR(road_info2.global_pos_x, 206.716, 1e-3);
                        EXPECT_NEAR(road_info2.global_pos_y, 92.447, 1e-3);
                        EXPECT_NEAR(roadInfo.trail_heading, 1.2182, 1e-3);
                    }
                }
                else if (abs(SE_GetSimulationTime() - 30.0f) < static_cast<float>(SMALL_NUMBER))
                {
                    SE_GetObjectState(0, &objectState);
                    EXPECT_NEAR(objectState.x, 382.065, 1e-3);
                    EXPECT_NEAR(objectState.y, 302.552, 1e-3);
                    EXPECT_NEAR(objectState.h, 5.271, 1e-3);
                    EXPECT_NEAR(objectState.p, 0.026, 1e-3);
                    if (ghostMode[i] == true)
                    {
                        SE_GetRoadInfoGhostTrailTime(0, SE_GetSimulationTime(), &road_info2, &speed3);
                        EXPECT_NEAR(road_info2.global_pos_x, 388.2154, 1e-3);
                        EXPECT_NEAR(road_info2.global_pos_y, 291.2665, 1e-3);
                        EXPECT_NEAR(roadInfo.trail_heading, 5.1519, 1e-3);
                    }
                }
            }

            // Report updated vehicle position and heading. z, pitch and roll will be aligned to the road
            SE_ReportObjectPosXYH(0, 0, vehicleState.x, vehicleState.y, vehicleState.h);
            SE_ReportObjectWheelStatus(0, vehicleState.wheel_rotation, vehicleState.wheel_angle);
            SE_ReportObjectSpeed(0, vehicleState.speed);

            // Finally, update scenario using same time step as for vehicle model
            SE_StepDT(dt);
        }
        SE_SimpleVehicleDelete(vehicleHandle);
        SE_Close();
    }
    SE_RegisterParameterDeclarationCallback(0, 0);
}

TEST(ExternalController, TestPositionAlignment)
{
    void* vehicleHandle = 0;

    SE_SimpleVehicleState  vehicleState = {0, 0, 0, 0, 0, 0, 0, 0};
    SE_ScenarioObjectState objectState;
    SE_RoadInfo            roadInfo;
    const double           throttleWeight = 0.1;
    const int              n_tests        = 4;
    int                    test_index     = 0;
    float                  dt             = 0.05f;
    float                  z_rel          = 0.0;
    float                  p_rel          = 0.0;
    struct
    {
        double time;
        double x;
        double y;
        double z;
        double h;
        double p;
        double r;
    } test[n_tests] = {{6.0000, 101.8704, 98.8925, -1.0125, 1.5589, 0.0006, 0.4948},
                       {13.5000, 15.2391, 200.7041, 9.7422, 2.9904, 5.9919, 0.0002},
                       {19.0000, -79.7515, 163.3215, 8.7630, 4.0404, 0.2896, 6.2829},
                       {20.0000, -90.3175, 147.0343, 3.5332, 4.2312, 0.5000, 0.2000}};

    SE_AddPath("../../../resources/models");

    ASSERT_EQ(SE_Init("../../../EnvironmentSimulator/Unittest/xosc/follow_ghost_align_position.xosc", 0, 0, 0, 0), 0);

    // Lock object to the original lane
    // If setting to false, the object road position will snap to closest lane
    SE_SetLockOnLane(0, true);

    // Initialize the vehicle model, fetch initial state from the scenario
    SE_GetObjectState(0, &objectState);
    vehicleHandle = SE_SimpleVehicleCreate(objectState.x, objectState.y, objectState.h, 4.0, 0.0);
    SE_SimpleVehicleSteeringRate(vehicleHandle, 6.0f);

    // show some road features, including road sensor
    SE_ViewerShowFeature(4 + 8, true);  // NODE_MASK_TRAIL_DOTS (1 << 2) & NODE_MASK_ODR_FEATURES (1 << 3),

    int counter = 0;

    // Run for specified duration or until 'Esc' button is pressed
    while (SE_GetQuitFlag() != 1)
    {
        // Get road information at a point some speed dependent distance ahead
        double targetSpeed;
        float  ghost_speed = 0.0f;
        SE_GetRoadInfoGhostTrailTime(0, SE_GetSimulationTime() + 0.25f, &roadInfo, &ghost_speed);

        targetSpeed = ghost_speed;

        // Steer towards where the point
        double steerAngle = roadInfo.angle;

        // Accelerate or decelerate towards target speed - THROTTLE_WEIGHT tunes magnitude
        double throttle = throttleWeight * (targetSpeed - static_cast<double>(vehicleState.speed));

        // Step vehicle model with driver input, but wait until time > 0
        if (SE_GetSimulationTime() > 0 && !SE_GetPauseFlag())
        {
            SE_SimpleVehicleControlAnalog(vehicleHandle, dt, throttle, steerAngle);
        }

        // Fetch updated state and report to scenario engine
        SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);

        // Report updated vehicle position and heading. z, pitch and roll will be aligned to the road

        if (test_index < 3)
        {
            SE_SetObjectPositionMode(0,
                                     SE_PositionModeType::SE_SET,
                                     SE_PositionMode::SE_H_ABS | SE_PositionMode::SE_Z_REL | SE_PositionMode::SE_P_REL | SE_PositionMode::SE_R_REL);
            SE_ReportObjectPos(0, 0, vehicleState.x, vehicleState.y, z_rel, vehicleState.h, p_rel, 0.0f);
        }
        else
        {
            SE_SetObjectPositionMode(0,
                                     SE_PositionModeType::SE_SET,
                                     SE_PositionMode::SE_H_ABS | SE_PositionMode::SE_Z_REL | SE_PositionMode::SE_P_ABS | SE_PositionMode::SE_R_REL);
            SE_ReportObjectPos(0, 0, vehicleState.x, vehicleState.y, 0.0f, vehicleState.h, 0.5f, 0.2f);
        }

        // wheel status (revolution and steering angles)
        SE_ReportObjectWheelStatus(0, vehicleState.wheel_rotation, vehicleState.wheel_angle);

        // speed (along vehicle longitudinal (x) axis)
        SE_ReportObjectSpeed(0, vehicleState.speed);

        // Finally, update scenario using same time step as for vehicle model
        SE_StepDT(dt);

        if (test_index < n_tests && SE_GetSimulationTime() > static_cast<float>(test[test_index].time - SMALL_NUMBER))
        {
            SE_GetObjectState(0, &objectState);
            EXPECT_NEAR(objectState.x, test[test_index].x, 1e-4);
            EXPECT_NEAR(objectState.y, test[test_index].y, 1e-4);
            EXPECT_NEAR(objectState.z, test[test_index].z + static_cast<double>(z_rel), 1e-4);
            EXPECT_NEAR(objectState.h, test[test_index].h, 1e-4);
            EXPECT_NEAR(objectState.p, test[test_index].p + static_cast<double>(p_rel), 1e-4);
            EXPECT_NEAR(objectState.r, test[test_index].r, 1e-4);
            if (test_index == 1)
            {
                z_rel = 2.0f;
                p_rel = 0.1f;
            }
            else if (test_index == 2)
            {
                z_rel = 0.0f;
                p_rel = 0.0f;
            }

            test_index++;
        }
        counter++;
    }
    SE_SimpleVehicleDelete(vehicleHandle);
    SE_Close();
}

TEST(PositionMode, TestRoadAlignmentModes)
{
    struct
    {
        double time;
        double x;
        double y;
        double z;
        double h;
        double p;
        double r;
    } result[2][4] = {{{10.0, 100.739, 87.310, -0.830, 3.015, 0.000, 0.000},
                       {10.0, 75.249, 87.375, 33.144, 2.677, 0.067, 0.000},
                       {10.0, 70.115, 87.356, 7.888, 2.245, 0.070, 5.629},
                       {10.0, 60.000, 70.000, 2.000, 2.356, 0.152, 6.083}},
                      {{17.0, 16.145, 200.243, 10.457, 2.982, 0.000, 0.000},
                       {17.0, -35.413, 98.073, 16.592, 3.265, 0.154, 0.000},
                       {17.0, -31.555, 100.000, 28.494, 3.142, 6.050, 5.971},
                       {17.0, -24.000, 70.000, 28.600, 3.142, 5.977, 6.083}}};

    SE_AddPath("../../../EnvironmentSimulator/Unittest/xodr");
    SE_AddPath("../../../resources/xosc/Catalogs/Pedestrians");
    SE_AddPath("../../../resources/models");

    float dt = 0.1f;

    ASSERT_EQ(SE_Init("../../../EnvironmentSimulator/code-examples/position-mode/skater.xosc", 0, 0, 0, 0), 0);

    int state        = 0;
    int result_state = 0;
    while (SE_GetQuitFlag() == 0)
    {
        if (state == 0 && SE_GetSimulationTime() > 5.0f)
        {
            SE_ReportObjectPosMode(
                0,
                0.0f,
                std::nanf(""),
                std::nanf(""),
                0.0f,
                1.57f,
                0.0f,
                0.0f,
                SE_PositionMode::SE_H_REL | SE_PositionMode::SE_P_ABS | SE_PositionMode::SE_R_ABS);  // report only (relative) heading
            SE_SetObjectPositionMode(0,
                                     SE_PositionModeType::SE_UPDATE,
                                     SE_PositionMode::SE_H_REL | SE_PositionMode::SE_P_ABS | SE_PositionMode::SE_R_ABS);
            SE_ReportObjectSpeed(0, 20.0f);

            state++;
        }
        else if (state == 1 && SE_GetSimulationTime() > 15.0f)
        {
            SE_ReportObjectPosMode(0,
                                   0.0f,
                                   std::nanf(""),
                                   std::nanf(""),
                                   1.0f,
                                   0.0f,
                                   0.0f,
                                   0.0f,
                                   SE_PositionMode::SE_Z_REL | SE_PositionMode::SE_H_REL | SE_PositionMode::SE_P_ABS |
                                       SE_PositionMode::SE_R_ABS);  // report only (relative) heading

            SE_SetObjectPositionMode(0,
                                     SE_PositionModeType::SE_UPDATE,
                                     SE_PositionMode::SE_Z_REL | SE_PositionMode::SE_H_REL | SE_PositionMode::SE_P_ABS | SE_PositionMode::SE_R_ABS);

            SE_ReportObjectSpeed(0, 30.0f);

            state++;
        }

        if (result_state < 2 && static_cast<double>(SE_GetSimulationTime()) > result[result_state][0].time - SMALL_NUMBER)
        {
            SE_ScenarioObjectState obj_state;
            for (int i = 0; i < 4; i++)
            {
                SE_GetObjectState(i, &obj_state);
                EXPECT_NEAR(obj_state.x, result[result_state][i].x, 1e-3);
                EXPECT_NEAR(obj_state.y, result[result_state][i].y, 1e-3);
                EXPECT_NEAR(obj_state.z, result[result_state][i].z, 1e-3);
                EXPECT_NEAR(obj_state.h, result[result_state][i].h, 1e-3);
                EXPECT_NEAR(obj_state.p, result[result_state][i].p, 1e-3);
                EXPECT_NEAR(obj_state.r, result[result_state][i].r, 1e-3);
            }
            result_state++;
        }

        SE_StepDT(dt);
    }

    SE_Close();
    SE_ClearPaths();
}

TEST(TestGetAndSet, SeedTest)
{
    std::string scenario_file = "../../../resources/xosc/cut-in.xosc";

    SE_SetSeed(12345);
    EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);
    ASSERT_EQ(SE_GetNumberOfObjects(), 2);
    ASSERT_EQ(SE_GetSeed(), static_cast<unsigned int>(12345));

    SE_Close();
}

TEST(SimpleVehicleTest, TestControl)
{
    float dt = 0.01f;

    std::string            scenario_file = "../../../resources/xosc/parking_lot.xosc";
    SE_SimpleVehicleState  vehicleState  = {0, 0, 0, 0, 0, 0, 0, 0};
    SE_ScenarioObjectState objectState;
    void*                  vehicleHandle = 0;

    EXPECT_EQ(SE_Init(scenario_file.c_str(), 1, 0, 0, 0), 0);
    ASSERT_EQ(SE_GetNumberOfObjects(), 3);

    ASSERT_EQ(SE_GetObjectState(0, &objectState), 0);
    EXPECT_NEAR(objectState.x, 1.800, 1e-3);
    EXPECT_NEAR(objectState.y, -358.000, 1e-3);
    EXPECT_NEAR(objectState.h, 1.570, 1e-3);

    vehicleHandle = SE_SimpleVehicleCreate(objectState.x, objectState.y, objectState.h, 4.0, 0.0);

    for (int i = 0; i < 200; i++)
    {
        SE_SimpleVehicleControlTarget(vehicleHandle, dt, 30.0, 0.2);
        SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
        SE_ReportObjectPosXYH(0, 0.0f, vehicleState.x, vehicleState.y, vehicleState.h);

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
        SE_ReportObjectPosXYH(0, 0.0f, vehicleState.x, vehicleState.y, vehicleState.h);

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
        SE_ReportObjectPosXYH(0, 0.0f, vehicleState.x, vehicleState.y, vehicleState.h);
        SE_ReportObjectSpeed(0, vehicleState.speed);

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
        SE_ReportObjectPosXYH(0, 0.0f, vehicleState.x, vehicleState.y, vehicleState.h);
        SE_ReportObjectSpeed(0, vehicleState.speed);

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
        SE_SimpleVehicleControlAnalog(vehicleHandle, dt, 0, 0);  // no throttle -> engine brake applied
        SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
        SE_ReportObjectPosXYH(0, 0.0f, vehicleState.x, vehicleState.y, vehicleState.h);
        SE_ReportObjectSpeed(0, vehicleState.speed);

        SE_StepDT(dt);
    }

    SE_SimpleVehicleGetState(vehicleHandle, &vehicleState);
    EXPECT_NEAR(vehicleState.x, -211.792, 1e-3);
    EXPECT_NEAR(vehicleState.y, 138.885, 1e-3);
    EXPECT_NEAR(vehicleState.h, 1.924, 1e-3);
    EXPECT_NEAR(vehicleState.speed, 3.459, 1e-3);

    SE_SimpleVehicleDelete(vehicleHandle);
    SE_Close();
}

class APITestCutIn : public testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        EXPECT_EQ(SE_Init("../../../resources/xosc/cut-in.xosc", 0, 0, 0, 0), 0);
    }

    static void TearDownTestSuite()
    {
        SE_Close();
    }
};

TEST_F(APITestCutIn, TestGetName)
{
    ASSERT_EQ(SE_GetNumberOfObjects(), 2);

    EXPECT_STREQ(SE_GetObjectName(0), "Ego");
    EXPECT_STREQ(SE_GetObjectTypeName(0), "car_white");
    EXPECT_STREQ(SE_GetObjectModelFileName(0), "car_white.osgb");

    EXPECT_STREQ(SE_GetObjectName(1), "OverTaker");
    EXPECT_STREQ(SE_GetObjectTypeName(1), "car_red");
    EXPECT_STREQ(SE_GetObjectModelFileName(1), "car_red.osgb");
}

TEST_F(APITestCutIn, TestGetIdByName)
{
    EXPECT_EQ(SE_GetIdByName("OverTaker"), 1);
    EXPECT_EQ(SE_GetIdByName("Ego"), 0);
    EXPECT_EQ(SE_GetIdByName("NotExisting"), -1);
}

TEST(APITest, TestGetRoute)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/highway_exit_with_route.xosc";

    EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);

    int num_of_points = SE_GetNumberOfRoutePoints(0);
    EXPECT_EQ(num_of_points, 4);

    SE_RouteInfo route_info;
    SE_GetRoutePoint(0, 0, &route_info);
    EXPECT_EQ(route_info.t, -1.5);
    EXPECT_EQ(route_info.s, 15);
    EXPECT_EQ(route_info.x, 15);
    EXPECT_EQ(route_info.y, -1.5);
    EXPECT_EQ(route_info.osiLaneId, 3);
    SE_GetRoutePoint(0, 1, &route_info);
    EXPECT_EQ(route_info.t, -4.5);
    EXPECT_EQ(route_info.s, 150);
    EXPECT_EQ(route_info.x, 150);
    EXPECT_EQ(route_info.y, -4.5);
    EXPECT_EQ(route_info.osiLaneId, 15);

    SE_Close();
}

#ifdef _USE_OSG

static bool CheckFileExists(std::string filename, long long timestamp)
{
    struct stat fileStatus;

    if (stat(filename.c_str(), &fileStatus) == 0)
    {
        if (fileStatus.st_mtime > timestamp)
        {
            return true;
        }
    }

    return false;
}

TEST(APITest, TestFetchImage)
{
    struct stat fileStatus;
    long long   oldModTime          = 0;
    int         max_pixel_deviation = 4;
    std::string screenshotFilename0 = "screen_shot_00000.tga";
    std::string screenshotFilename1 = "screen_shot_00001.tga";
    std::string screenshotFilename2 = "screen_shot_00002.tga";
    std::string screenshotFilename3 = "screen_shot_00003.tga";
    std::string screenshotFilename4 = "screen_shot_00004.tga";
    std::string screenshotFilename5 = "screen_shot_00005.tga";

    // Fetch timestamp of any old screenshot0
    if (stat(screenshotFilename0.c_str(), &fileStatus) == 0)
    {
        oldModTime = fileStatus.st_mtime;
    }

    const char* args[] =
        {"--osc", "../../../resources/xosc/cut-in_simple.xosc", "--window", "60", "60", "800", "400", "--aa_mode", "4", "--headless"};

    SE_SaveImagesToRAM(true);
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);

    ASSERT_EQ(SE_GetNumberOfObjects(), 2);

    SE_Image image = {0, 0, 0, 0, 0};
    SE_FetchImage(&image);
    ASSERT_NE(image.data, nullptr);

    EXPECT_EQ(image.width, 800);
    EXPECT_EQ(image.height, 400);
    EXPECT_EQ(image.pixelSize, 3);

    // Check RGB color values of a random pixel x=222, y=250
    int pixelNr = ((400 - 1) - 250) * 800 + 222;                                      // image stored upside down
    EXPECT_NEAR(image.data[pixelNr * image.pixelSize + 2], 78, max_pixel_deviation);  // R
    EXPECT_NEAR(image.data[pixelNr * image.pixelSize + 1], 96, max_pixel_deviation);  // G
    EXPECT_NEAR(image.data[pixelNr * image.pixelSize + 0], 47, max_pixel_deviation);  // B

    // Save file for possible post processing or inspection
    SE_WriteTGA("offscreen0.tga", image.width, image.height, image.data, image.pixelSize, image.pixelFormat, true);

    // Verify that any screenshot file is old, since that feature has not been enabled yet
    EXPECT_EQ(CheckFileExists(screenshotFilename0, oldModTime), false);

    // SE_sleep(500);
    SE_StepDT(0.1f);  // Step once to create another image
    SE_FetchImage(&image);
    SE_WritePPM("offscreen1.ppm", image.width, image.height, image.data, image.pixelSize, image.pixelFormat, true);

    EXPECT_EQ(image.width, 800);
    EXPECT_EQ(image.height, 400);
    EXPECT_EQ(image.pixelSize, 3);

    // Check pixel
    EXPECT_NEAR(image.data[pixelNr * image.pixelSize + 2], 70, max_pixel_deviation);  // R
    EXPECT_NEAR(image.data[pixelNr * image.pixelSize + 1], 83, max_pixel_deviation);  // G
    EXPECT_NEAR(image.data[pixelNr * image.pixelSize + 0], 45, max_pixel_deviation);  // B

    SE_StepDT(0.1f);  // And another one

    SE_FetchImage(&image);
    SE_WritePPM("offscreen2.ppm", image.width, image.height, image.data, image.pixelSize, image.pixelFormat, true);
    EXPECT_EQ(image.width, 800);
    EXPECT_EQ(image.height, 400);
    EXPECT_EQ(image.pixelSize, 3);

    // Check pixel
    EXPECT_NEAR(image.data[pixelNr * image.pixelSize + 2], 75, max_pixel_deviation);  // R
    EXPECT_NEAR(image.data[pixelNr * image.pixelSize + 1], 90, max_pixel_deviation);  // G
    EXPECT_NEAR(image.data[pixelNr * image.pixelSize + 0], 45, max_pixel_deviation);  // B

    // Verify that we can't fetch images when feature disabled
    EXPECT_EQ(SE_SaveImagesToRAM(false), 0);
    SE_StepDT(0.1f);
    EXPECT_EQ(SE_FetchImage(&image), -1);

    // Now test screenshot functionality, starting with creating just two images
    EXPECT_EQ(SE_SaveImagesToFile(2), 0);

    // Check timestamp again of any old screenshot0
    EXPECT_EQ(CheckFileExists(screenshotFilename0, oldModTime), false);

    // Run a few steps to create the screenshot images
    for (int i = 0; i < 3; i++)
        SE_StepDT(0.1f);
    SE_sleep(100);  // Allow for last image to be created (running in a separate thread)

    EXPECT_EQ(CheckFileExists(screenshotFilename0, oldModTime), true);
    ASSERT_EQ(stat(screenshotFilename0.c_str(), &fileStatus), 0);
    oldModTime = fileStatus.st_mtime;                                       // use timestamp of first image as base for following comparisons
    EXPECT_EQ(CheckFileExists(screenshotFilename1, oldModTime - 1), true);  // -1 since timestamps could be equal
    // Make sure no screenshot2 has been created
    EXPECT_EQ(CheckFileExists(screenshotFilename2, oldModTime - 1), false);

    // Now test continuous snapshot functionality
    EXPECT_EQ(SE_SaveImagesToFile(-1), 0);

    for (int i = 0; i < 3; i++)
        SE_StepDT(0.1f);  // step to create three additional screenshots
    SE_sleep(100);        // Allow for last image to be created (running in a separate thread)
    EXPECT_EQ(CheckFileExists(screenshotFilename2, oldModTime - 1), true);
    EXPECT_EQ(CheckFileExists(screenshotFilename3, oldModTime - 1), true);
    EXPECT_EQ(CheckFileExists(screenshotFilename4, oldModTime - 1), true);
    // Make sure no screenshot5 has been created
    EXPECT_EQ(CheckFileExists(screenshotFilename5, oldModTime - 1), false);

    SE_Close();
    SE_SaveImagesToRAM(false);
}

#endif  // _USE_OSG

static void paramDeclCallbackSetRoute(void* args)
{
    double(*positions)[8] = reinterpret_cast<double(*)[8]>(args);
    static int counter    = 0;

    SE_SetParameterInt("StartRoadId", static_cast<int>(positions[counter][0]));
    SE_SetParameterInt("StartLaneId", static_cast<int>(positions[counter][1]));
    SE_SetParameterDouble("StartRoadS", positions[counter][2]);
    SE_SetParameterDouble("StartH", positions[counter][3]);
    SE_SetParameterInt("EndRoadId", static_cast<int>(positions[counter][4]));
    SE_SetParameterInt("EndLaneId", static_cast<int>(positions[counter][5]));
    SE_SetParameterDouble("EndRoadS", positions[counter][6]);
    SE_SetParameterDouble("EndH", positions[counter][7]);

    counter++;
}

TEST(DirectJunctionTest, TestVariousRoutes)
{
    // This test case will run the same scenario multiple times
    // each with a new route involving the direct junction
    // if the route can't be resolved esmini and test will fail
    // Additionally expected end positions are verified

    static double positions[][8] = {
        {1, -2, 50, 0.0, 5, -2, 50, 0.0},    // Start at road 1 lane -2, end at road 5 lane -2
        {1, -3, 50, 0.0, 3, -1, 50, 0.0},    // Start at road 1 lane -3, end at road 3 lane -1
        {5, -1, 40, M_PI, 1, -1, 10, M_PI},  // Start at road 1 lane -2, end at road 5 lane -2
        {3, -1, 50, M_PI, 1, -3, 10, M_PI},  // Start at road 1 lane -3, end at road 3 lane -1
    };

    double end_pos[][2] = {{250.000, -4.605}, {196.793, -23.858}, {0.000, -1.535}, {0.000, -7.675}};

    SE_ScenarioObjectState state;

    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/direct_junction.xosc";

    SE_AddPath("../../../resources/models");

    for (int i = 0; i < static_cast<int>(sizeof(positions) / sizeof(double[8])); i++)
    {
        SE_RegisterParameterDeclarationCallback(paramDeclCallbackSetRoute, &positions);
        ASSERT_EQ(SE_Init(scenario_file.c_str(), 1, 0, 0, 0), 0);
        ASSERT_EQ(SE_GetNumberOfObjects(), 1);

        while (SE_GetQuitFlag() != 1)
        {
            SE_StepDT(0.1f);
        }

        // Check position of second vehicle
        SE_GetObjectState(0, &state);
        EXPECT_NEAR(state.x, end_pos[i][0], 1e-3);
        EXPECT_NEAR(state.y, end_pos[i][1], 1e-3);

        SE_Close();
    }
    SE_RegisterParameterDeclarationCallback(0, 0);
}

TEST(ReplayTest, TestMultiReplayDifferentTimeSteps)
{
    const char* args[2][2][6] = {
        // First run with smaller timsteps in first scenario
        {{"--osc", "../../../resources/xosc/follow_ghost.xosc", "--record", "multirep_test1.dat", "--fixed_timestep", "0.01"},
         {"--osc", "../../../resources/xosc/left-hand-traffic_using_road_rule.xosc", "--record", "multirep_test2.dat", "--fixed_timestep", "0.1"}},
        // Then run with smaller timsteps in second scenario
        {{"--osc", "../../../resources/xosc/follow_ghost.xosc", "--record", "multirep_test1.dat", "--fixed_timestep", "0.1"},
         {"--osc", "../../../resources/xosc/left-hand-traffic_using_road_rule.xosc", "--record", "multirep_test2.dat", "--fixed_timestep", "0.01"}}};

    SE_AddPath("../../../resources/models");

    for (int k = 0; k < 2; k++)
    {
        // Run the two scenarios, create dat files
        for (int i = 0; i < 2; i++)
        {
            ASSERT_EQ(SE_InitWithArgs(sizeof(args[k][i]) / sizeof(char*), args[k][i]), 0);

            while (SE_GetQuitFlag() != 1)
            {
                if (k == 0 && i == 0)
                    SE_StepDT(0.01f);
                else if (k == 0 && i == 1)
                    SE_StepDT(0.1f);
                else if (k == 1 && i == 0)
                    SE_StepDT(0.1f);
                else if (k == 1 && i == 1)
                    SE_StepDT(0.01f);
            }

            SE_Close();
        }

        // Check multi replay
        scenarioengine::Replay* replay = new scenarioengine::Replay(".", "multirep_test", "");
        EXPECT_EQ(replay->GetNumberOfScenarios(), 2);

        EXPECT_NEAR(replay->data_[0].state.info.timeStamp, -2.5, 1E-3);
        EXPECT_STREQ(replay->data_[0].state.info.name, "Ego");
        EXPECT_STREQ(replay->data_[1].state.info.name, "Ego_ghost");
        EXPECT_STREQ(replay->data_[2].state.info.name, "Ego");
        EXPECT_NEAR(replay->data_[2].state.info.timeStamp, -2.45, 1E-3);
        EXPECT_NEAR(replay->data_[4].state.info.timeStamp, -2.40, 1E-3);
        EXPECT_NEAR(replay->data_[100].state.info.timeStamp, 0.0, 1E-3);
        EXPECT_NEAR(replay->data_[100].state.info.id, 0, 1E-3);
        EXPECT_NEAR(replay->data_[101].state.info.timeStamp, 0.0, 1E-3);
        EXPECT_NEAR(replay->data_[101].state.info.id, 1, 1E-3);
        EXPECT_NEAR(replay->data_[102].state.info.timeStamp, 0.0, 1E-3);
        EXPECT_NEAR(replay->data_[102].state.info.id, 100, 1E-3);
        EXPECT_NEAR(replay->data_[103].state.info.timeStamp, 0.0, 1E-3);
        EXPECT_NEAR(replay->data_[103].state.info.id, 101, 1E-3);
        EXPECT_NEAR(replay->data_[104].state.info.timeStamp, 0.01, 1E-3);
        EXPECT_NEAR(replay->data_[104].state.info.id, 0, 1E-3);
        EXPECT_NEAR(replay->data_[108].state.info.timeStamp, 0.02, 1E-3);
        EXPECT_NEAR(replay->data_[108].state.info.id, 0, 1E-3);
        EXPECT_NEAR(replay->data_[139].state.info.timeStamp, 0.09, 1E-3);
        EXPECT_NEAR(replay->data_[139].state.info.id, 101, 1E-3);
        EXPECT_NEAR(replay->data_[140].state.info.timeStamp, 0.1, 1E-3);
        EXPECT_NEAR(replay->data_[140].state.info.id, 0, 1E-3);

        EXPECT_NEAR(replay->data_[2012].state.info.timeStamp, 4.78, 1E-3);
        EXPECT_NEAR(replay->data_[2012].state.info.id, 0, 1E-3);
        EXPECT_NEAR(replay->data_[2015].state.info.timeStamp, 4.78, 1E-3);
        EXPECT_NEAR(replay->data_[2015].state.info.id, 101, 1E-3);

        if (k == 0)
        {
            EXPECT_NEAR(replay->data_[2012].state.pos.y, 130.994, 1E-3);
            EXPECT_NEAR(replay->data_[2015].state.pos.y, 207.392, 1E-3);
            EXPECT_NEAR(replay->data_[5965].state.info.timeStamp, 19.51, 1E-3);
            EXPECT_NEAR(replay->data_[5965].state.info.id, 1, 1E-3);
        }
        else
        {
            EXPECT_NEAR(replay->data_[2012].state.pos.y, 130.913, 1E-3);
            EXPECT_NEAR(replay->data_[2015].state.pos.y, 210.738, 1E-3);
            EXPECT_NEAR(replay->data_[4201].state.info.timeStamp, 19.6, 1E-3);
            EXPECT_NEAR(replay->data_[4201].state.info.id, 1, 1E-3);
        }

        delete replay;
    }
}

void ConditionCallbackInstance1(const char* element_name, double timestamp)
{
    EXPECT_STREQ(element_name, "act_start_condition");
    EXPECT_NEAR(timestamp, 0.1, 1E-4);
    EXPECT_NEAR(static_cast<float>(timestamp), SE_GetSimulationTime(), 1E-4);
}

TEST(EventCallbackTest, TestConditionCallback)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/highway_exit.xosc";

    SE_Init(scenario_file.c_str(), 0, 0, 0, 0);

    int n_Objects = SE_GetNumberOfObjects();
    EXPECT_EQ(n_Objects, 2);

    SE_RegisterConditionCallback(ConditionCallbackInstance1);

    // Just run for two steps such that the act gets started
    for (int i = 0; i < 2; i++)
    {
        SE_StepDT(0.1f);
    }

    SE_Close();
}

void StoryBoardElementStateCallbackInstance1(const char* element_name, int type, int state, const char* full_path)
{
    static int counter = 0;
    const int  n_runs  = 18;
    struct
    {
        const char* name;
        double      time;
        int         type;
        int         state;
        const char* full_path;
    } state_target[n_runs] = {
        {"hwe_act", 0.1, 3, 2, "hwe_story::hwe_act"},                                                                      // Act, Running
        {"hwe_maneuvergroup", 0.1, 4, 2, "hwe_story::hwe_act::hwe_maneuvergroup"},                                         // ManeuverGroup, Running
        {"hwe_maneuver", 0.1, 5, 2, "hwe_story::hwe_act::hwe_maneuvergroup::hwe_maneuver"},                                // Maneuver, Running
        {"slowdown event", 0.1, 6, 1, "hwe_story::hwe_act::hwe_maneuvergroup::hwe_maneuver::slowdown event"},              // Event, Standby
        {"lanechange event", 0.1, 6, 1, "hwe_story::hwe_act::hwe_maneuvergroup::hwe_maneuver::lanechange event"},          // Event, Standby
        {"slowdown event", 3.4, 6, 2, "hwe_story::hwe_act::hwe_maneuvergroup::hwe_maneuver::slowdown event"},              // Event, Running
        {"slowdown", 3.4, 7, 2, "hwe_story::hwe_act::hwe_maneuvergroup::hwe_maneuver::slowdown event::slowdown"},          // Action, Running
        {"slowdown", 4.4, 7, 3, "hwe_story::hwe_act::hwe_maneuvergroup::hwe_maneuver::slowdown event::slowdown"},          // Action, Complete
        {"slowdown event", 4.4, 6, 3, "hwe_story::hwe_act::hwe_maneuvergroup::hwe_maneuver::slowdown event"},              // Event, Complete
        {"lanechange event", 4.4, 6, 2, "hwe_story::hwe_act::hwe_maneuvergroup::hwe_maneuver::lanechange event"},          // Event, Running
        {"lane change", 4.4, 7, 2, "hwe_story::hwe_act::hwe_maneuvergroup::hwe_maneuver::lanechange event::lane change"},  // Action, Running
        {"lane change", 8.3, 7, 3, "hwe_story::hwe_act::hwe_maneuvergroup::hwe_maneuver::lanechange event::lane change"},  // Action, Complete
        {"lanechange event", 8.3, 6, 3, "hwe_story::hwe_act::hwe_maneuvergroup::hwe_maneuver::lanechange event"},          // Event, Complete
        {"hwe_maneuver", 8.3, 5, 3, "hwe_story::hwe_act::hwe_maneuvergroup::hwe_maneuver"},                                // Maneuver, Complete
        {"hwe_maneuvergroup", 8.3, 4, 3, "hwe_story::hwe_act::hwe_maneuvergroup"},                                         // ManeuverGroup, Complete
        {"hwe_act", 8.3, 3, 3, "hwe_story::hwe_act"},                                                                      // Act, Complete
        {"hwe_story", 8.3, 2, 3, "hwe_story"},                                                                             // Story, Complete
        {"storyBoard", 12.1, 1, 3, ""},                                                                                    // StoryBoard, Complete
    };

    if (counter < n_runs)
    {
        EXPECT_STREQ(element_name, state_target[counter].name);
        EXPECT_NEAR(SE_GetSimulationTime(), state_target[counter].time, 1E-4);
        EXPECT_EQ(type, state_target[counter].type);
        EXPECT_EQ(state, state_target[counter].state);
        EXPECT_STREQ(full_path, state_target[counter].full_path);
    }

    counter++;
}

TEST(EventCallbackTest, TestStoryboardElementStateCallback)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/highway_exit.xosc";

    SE_Init(scenario_file.c_str(), 0, 0, 0, 0);

    int n_Objects = SE_GetNumberOfObjects();
    EXPECT_EQ(n_Objects, 2);

    SE_RegisterStoryBoardElementStateChangeCallback(StoryBoardElementStateCallbackInstance1);

    // Just run until passed 12 seconds to cover the complete scenario
    for (int i = 0; i < 125; i++)
    {
        SE_StepDT(0.1f);
    }

    SE_Close();
}

TEST(RoadmanagerTest, TestGetInfoAtDistance)
{
    std::string scenario_file = "../../../resources/xosc/slow-lead-vehicle.xosc";

    SE_Init(scenario_file.c_str(), 0, 0, 0, 0);

    int n_Objects = SE_GetNumberOfObjects();
    EXPECT_EQ(n_Objects, 2);

    SE_RoadInfo roadinfo;
    ASSERT_EQ(SE_GetRoadInfoAtDistance(0, -49, &roadinfo, 0, true), 0);
    EXPECT_NEAR(roadinfo.global_pos_x, 1.0, 1e-5);
    EXPECT_NEAR(roadinfo.global_pos_y, -1.535, 1e-5);
    EXPECT_NEAR(roadinfo.local_pos_x, -49.0, 1e-5);
    EXPECT_NEAR(roadinfo.local_pos_y, 0.0, 1e-5);

    ASSERT_EQ(SE_GetRoadInfoAtDistance(0, -51, &roadinfo, 0, true), -2);
    EXPECT_NEAR(roadinfo.global_pos_x, 0.0, 1e-5);
    EXPECT_NEAR(roadinfo.global_pos_y, -1.535, 1e-5);
    EXPECT_NEAR(roadinfo.local_pos_x, -50.0, 1e-5);
    EXPECT_NEAR(roadinfo.local_pos_y, 0.0, 1e-5);

    ASSERT_EQ(SE_GetRoadInfoAtDistance(0, 449, &roadinfo, 0, true), 0);
    EXPECT_NEAR(roadinfo.global_pos_x, 499.0, 1e-5);
    EXPECT_NEAR(roadinfo.global_pos_y, -1.535, 1e-3);
    EXPECT_NEAR(roadinfo.local_pos_x, 449.0, 1e-5);
    EXPECT_NEAR(roadinfo.local_pos_y, 0.0, 1e-5);

    ASSERT_EQ(SE_GetRoadInfoAtDistance(0, 451, &roadinfo, 0, true), -2);
    EXPECT_NEAR(roadinfo.global_pos_x, 500.0, 1e-5);
    EXPECT_NEAR(roadinfo.global_pos_y, -1.535, 1e-3);
    EXPECT_NEAR(roadinfo.local_pos_x, 450.0, 1e-5);
    EXPECT_NEAR(roadinfo.local_pos_y, 0.0, 1e-5);

    SE_Close();
}

TEST(RoadmanagerTest, TestGetPositionDiff)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/loomingAdvancedTest.xosc";

    EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);

    int n_Objects = SE_GetNumberOfObjects();
    EXPECT_EQ(n_Objects, 2);

    SE_PositionDiff diff;
    ASSERT_EQ(SE_GetDistanceToObject(0, 1, false, &diff), 0);
    EXPECT_EQ(diff.dLaneId, 0);
    EXPECT_NEAR(diff.ds, 247.100, 1e-3);
    EXPECT_NEAR(diff.dt, 0.0, 1e-3);
    EXPECT_NEAR(diff.dx, 233.409, 1e-3);
    EXPECT_NEAR(diff.dy, 53.693, 1e-3);
    EXPECT_EQ(diff.oppositeLanes, false);

    while (SE_GetSimulationTime() < 30.9f)
    {
        SE_StepDT(0.1f);
    }

    ASSERT_EQ(SE_GetDistanceToObject(0, 1, true, &diff), 0);
    EXPECT_EQ(diff.dLaneId, 0);
    EXPECT_NEAR(diff.ds, 31.6925, 1e-3);
    EXPECT_NEAR(diff.dt, 0.0, 1e-3);
    EXPECT_NEAR(diff.dx, 20.8153, 1e-3);
    EXPECT_NEAR(diff.dy, -18.4346, 1e-3);
    EXPECT_EQ(diff.oppositeLanes, false);

    while (SE_GetSimulationTime() < 35.0f)
    {
        SE_StepDT(0.1f);
    }

    ASSERT_EQ(SE_GetDistanceToObject(0, 1, false, &diff), 0);
    EXPECT_EQ(diff.dLaneId, -1);
    EXPECT_NEAR(diff.ds, -93.0020, 1e-3);
    EXPECT_NEAR(diff.dt, -2.8970, 1e-3);
    EXPECT_NEAR(diff.dx, -30.5731, 1e-3);
    EXPECT_NEAR(diff.dy, -68.9285, 1e-3);
    EXPECT_EQ(diff.oppositeLanes, true);

    SE_Close();
}

TEST(ParamDistTest, TestRunAll)
{
#ifdef _USE_OSI
    std::vector<std::string> gt = {
        "gt_1_of_12.osi",
        "gt_2_of_12.osi",
        "gt_3_of_12.osi",
        "gt_4_of_12.osi",
        "gt_5_of_12.osi",
        "gt_6_of_12.osi",
        "gt_7_of_12.osi",
        "gt_8_of_12.osi",
        "gt_9_of_12.osi",
        "gt_10_of_12.osi",
        "gt_11_of_12.osi",
        "gt_12_of_12.osi",
    };
#endif  // _USE_OSI

    std::vector<std::string> dat = {
        "cut-in_1_of_12.dat",
        "cut-in_2_of_12.dat",
        "cut-in_3_of_12.dat",
        "cut-in_4_of_12.dat",
        "cut-in_5_of_12.dat",
        "cut-in_6_of_12.dat",
        "cut-in_7_of_12.dat",
        "cut-in_8_of_12.dat",
        "cut-in_9_of_12.dat",
        "cut-in_10_of_12.dat",
        "cut-in_11_of_12.dat",
        "cut-in_12_of_12.dat",
    };

    std::vector<std::string> log = {
        "log_1_of_12.txt",
        "log_2_of_12.txt",
        "log_3_of_12.txt",
        "log_4_of_12.txt",
        "log_5_of_12.txt",
        "log_6_of_12.txt",
        "log_7_of_12.txt",
        "log_8_of_12.txt",
        "log_9_of_12.txt",
        "log_10_of_12.txt",
        "log_11_of_12.txt",
        "log_12_of_12.txt",
    };

    // Fetch timestamp of any old run
    struct stat fileStatus;
    long long   oldModTime = 0;

    long long time_sample1 = 0;
    long long time_sample2 = 0;
    if (stat(log[0].c_str(), &fileStatus) == 0)
    {
        oldModTime = fileStatus.st_mtime;
    }

    if (stat(dat[0].c_str(), &fileStatus) == 0)
    {
        if (fileStatus.st_mtime > oldModTime)
        {
            oldModTime = fileStatus.st_mtime;
        }
    }

    if (stat(log[0].c_str(), &fileStatus) == 0)
    {
        if (fileStatus.st_mtime > oldModTime)
        {
            oldModTime = fileStatus.st_mtime;
        }
    }

    std::string scenario_file = "../../../resources/xosc/cut-in.xosc";
    EXPECT_EQ(SE_SetParameterDistribution("../../../resources/xosc/cut-in_parameter_set.xosc"), 0);

    EXPECT_EQ(SE_GetNumberOfPermutations(), 12);

    for (int i = 0; i < SE_GetNumberOfPermutations(); i++)
    {
        SE_Init(scenario_file.c_str(), 0, 0, 0, 1);

#ifdef _USE_OSI
        SE_EnableOSIFile("gt.osi");
#endif  // _USE_OSI

        for (int j = 0; j < 50 && SE_GetQuitFlag() == 0; j++)
        {
            SE_StepDT(0.1f);
        }

#ifdef _USE_OSI
        SE_DisableOSIFile();
#endif  // _USE_OSI

        SE_Close();
    }

    EXPECT_EQ(SE_GetNumberOfPermutations(), 12);

    // Check that files have been created as expected
    for (size_t i = 0; i < static_cast<size_t>(SE_GetNumberOfPermutations()); i++)
    {
#ifdef _USE_OSI
        EXPECT_EQ(stat(gt[i].c_str(), &fileStatus), 0);
        EXPECT_GE(fileStatus.st_mtime, oldModTime);
        EXPECT_GE(fileStatus.st_size, 0);
#endif  // _USE_OSI
        EXPECT_EQ(stat(dat[i].c_str(), &fileStatus), 0);
        EXPECT_GE(fileStatus.st_mtime, oldModTime);

        EXPECT_EQ(stat(log[i].c_str(), &fileStatus), 0);
        EXPECT_GE(fileStatus.st_mtime, oldModTime);

        if (i == 0)
        {
            time_sample1 = fileStatus.st_mtime;
        }
        if (i == 5)
        {
            time_sample2 = fileStatus.st_mtime;
        }
    }

    SE_sleep(20);

    // specify start at 4:th permutation, then expect automatic increment from there
    SE_SelectPermutation(3);
    do
    {
        SE_Init(scenario_file.c_str(), 0, 0, 0, 1);
        SE_EnableOSIFile("gt.osi");

        for (int j = 0; j < 50 && SE_GetQuitFlag() == 0; j++)
        {
            SE_StepDT(0.1f);
        }

        SE_DisableOSIFile();
        SE_Close();

    } while (SE_GetPermutationIndex() < SE_GetNumberOfPermutations() - 1);

    // The first 3 files should be untouched, while the last 3 should be updated
    // check two samples, one from each category
    EXPECT_EQ(stat(log[0].c_str(), &fileStatus), 0);
    EXPECT_EQ(fileStatus.st_mtime, time_sample1);
    EXPECT_EQ(stat(log[5].c_str(), &fileStatus), 0);
    EXPECT_GE(fileStatus.st_mtime, time_sample2);

    SE_ResetParameterDistribution();
}

TEST(GetFunctionsTest, TestGetAccelerations)
{
    std::string scenario_file = "../../../resources/xosc/cut-in_simple.xosc";

    EXPECT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);
    EXPECT_EQ(SE_GetNumberOfObjects(), 2);

    int   state    = 0;
    float acc_x    = 0.0f;
    float acc_y    = 0.0f;
    float acc_z    = 0.0f;
    float acc_lat  = 0.0f;
    float acc_long = 0.0f;

    while (SE_GetSimulationTime() < 8.0f && state < 4)
    {
        SE_StepDT(0.1f);
        if (state == 0 && SE_GetSimulationTime() > 6.5f)
        {
            EXPECT_EQ(SE_GetObjectAccelerationGlobalXYZ(1, &acc_x, &acc_y, &acc_z), 0);
            EXPECT_NEAR(acc_x, 0.0, 1e-3);
            EXPECT_NEAR(acc_y, 0.0, 1e-3);
            EXPECT_NEAR(acc_z, 0.0, 1e-3);

            EXPECT_EQ(SE_GetObjectAccelerationLocalLatLong(1, &acc_lat, &acc_long), 0);
            EXPECT_NEAR(acc_lat, 0.0, 1e-3);
            EXPECT_NEAR(acc_long, 0.0, 1e-3);

            state++;
        }
        else if (state == 1 && SE_GetSimulationTime() > 7.5f)
        {
            EXPECT_EQ(SE_GetObjectAccelerationGlobalXYZ(1, &acc_x, &acc_y, &acc_z), 0);
            EXPECT_NEAR(acc_x, -0.054, 1e-3);
            EXPECT_NEAR(acc_y, -0.998, 1e-3);
            EXPECT_NEAR(acc_z, 0.0, 1e-3);

            EXPECT_EQ(SE_GetObjectAccelerationLocalLatLong(1, &acc_lat, &acc_long), 0);
            EXPECT_NEAR(acc_lat, -1.000, 1e-3);
            EXPECT_NEAR(acc_long, -0.001, 1e-3);

            state++;
        }
        else if (state == 2)
        {
            // Set explicit acceleration
            SE_ReportObjectAcc(1, 0.0, 1.0, 2.0, 3.0);

            state++;
        }
        else if (state == 3)
        {
            EXPECT_EQ(SE_GetObjectAccelerationGlobalXYZ(1, &acc_x, &acc_y, &acc_z), 0);
            EXPECT_NEAR(acc_x, 1.0, 1e-3);
            EXPECT_NEAR(acc_y, 2.0, 1e-3);
            EXPECT_NEAR(acc_z, 3.0, 1e-3);

            EXPECT_EQ(SE_GetObjectAccelerationLocalLatLong(1, &acc_lat, &acc_long), 0);
            EXPECT_NEAR(acc_lat, 2.056, 1e-3);
            EXPECT_NEAR(acc_long, 0.880, 1e-3);

            state++;
        }
    }

    SE_Close();
}

TEST(StringIds, TestRoadStringIds)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/test_string_ids.xosc";

    ASSERT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);
    EXPECT_EQ(SE_GetNumberOfObjects(), 1);

    int                    test_state = 0;
    SE_ScenarioObjectState obj_state;
    while (SE_GetQuitFlag() == 0)
    {
        if (test_state == 0 && SE_GetSimulationTime() > 0.1f)
        {
            SE_GetObjectState(SE_GetId(0), &obj_state);
            EXPECT_EQ(obj_state.roadId, 0);
            EXPECT_STREQ(SE_GetRoadIdString(obj_state.roadId), "0");
            test_state++;
        }
        else if (test_state == 1 && SE_GetSimulationTime() > 1.25f)
        {
            SE_GetObjectState(SE_GetId(0), &obj_state);
            EXPECT_EQ(obj_state.roadId, 127);
            EXPECT_STREQ(SE_GetRoadIdString(obj_state.roadId), "ConnectingRoad9");
            test_state++;
        }
        else if (test_state == 2 && SE_GetSimulationTime() > 2.0f)
        {
            SE_GetObjectState(SE_GetId(0), &obj_state);
            EXPECT_EQ(obj_state.roadId, 3);
            EXPECT_STREQ(SE_GetRoadIdString(obj_state.roadId), "Kalle");
            test_state++;
        }
        else if (test_state == 3)
        {
            SE_ReportObjectRoadPos(SE_GetId(0), 0.0f, SE_GetRoadIdFromString("2Kalle3"), -1, 0.0, 1.0);
            test_state++;
        }
        else if (test_state == 4)
        {
            SE_GetObjectState(SE_GetId(0), &obj_state);
            EXPECT_EQ(obj_state.roadId, 128);
            EXPECT_STREQ(SE_GetRoadIdString(obj_state.roadId), "2Kalle3");
            EXPECT_NEAR(obj_state.x, 19.183f, 1e-3);
            EXPECT_NEAR(obj_state.y, -5.431f, 1e-3);
            test_state++;
        }

        SE_StepDT(0.1f);
    }

    // check function getting internal ID from string ID
    EXPECT_EQ(SE_GetRoadIdFromString("Kalle"), 3);
    EXPECT_EQ(SE_GetRoadIdFromString("16"), 16);
    EXPECT_EQ(SE_GetRoadIdFromString("4294967295"), 126);
    EXPECT_EQ(SE_GetRoadIdFromString("ConnectingRoad9"), 127);
    EXPECT_EQ(SE_GetRoadIdFromString("2Kalle3"), 128);
    EXPECT_EQ(SE_GetRoadIdFromString("4294967296"), 129);
    EXPECT_STREQ(SE_GetRoadIdString(3), "Kalle");
    EXPECT_STREQ(SE_GetRoadIdString(16), "16");
    EXPECT_STREQ(SE_GetRoadIdString(127), "ConnectingRoad9");
    EXPECT_STREQ(SE_GetRoadIdString(128), "2Kalle3");
    EXPECT_STREQ(SE_GetRoadIdString(129), "4294967296");

    EXPECT_EQ(SE_GetJunctionIdFromString("Junction4"), 0);
    EXPECT_STREQ(SE_GetJunctionIdString(0), "Junction4");

    SE_Close();
}

TEST(APITest, TestWheelData)
{
    std::string scenario_file = "../../../resources/xosc/lane_change_crest.xosc";

    ASSERT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);
    EXPECT_EQ(SE_GetNumberOfObjects(), 3);

    EXPECT_EQ(SE_GetObjectNumberOfWheels(SE_GetId(0)), 4);
    EXPECT_EQ(SE_GetObjectNumberOfWheels(SE_GetId(1)), 4);
    EXPECT_EQ(SE_GetObjectNumberOfWheels(SE_GetId(2)), 4);
    EXPECT_EQ(SE_GetObjectNumberOfWheels(-1), -1);
    EXPECT_EQ(SE_GetObjectNumberOfWheels(3), -1);

    SE_WheelData wheel_data;

    SE_GetObjectWheelData(SE_GetId(0), 0, &wheel_data);
    EXPECT_EQ(wheel_data.axle, 0);
    EXPECT_NEAR(wheel_data.h, 0.0, 1E-3);
    EXPECT_EQ(wheel_data.index, 0);
    EXPECT_NEAR(wheel_data.p, 0.0, 1E-3);
    EXPECT_NEAR(wheel_data.x, 2.98, 1E-3);
    EXPECT_NEAR(wheel_data.y, -0.840, 1E-3);
    EXPECT_NEAR(wheel_data.z, 0.4, 1E-3);

    SE_GetObjectWheelData(SE_GetId(0), 1, &wheel_data);
    EXPECT_EQ(wheel_data.axle, 0);
    EXPECT_NEAR(wheel_data.h, 0.0, 1E-3);
    EXPECT_EQ(wheel_data.index, 1);
    EXPECT_NEAR(wheel_data.p, 0.0, 1E-3);
    EXPECT_NEAR(wheel_data.x, 2.98, 1E-3);
    EXPECT_NEAR(wheel_data.y, 0.840, 1E-3);
    EXPECT_NEAR(wheel_data.z, 0.4, 1E-3);

    SE_GetObjectWheelData(SE_GetId(0), 2, &wheel_data);
    EXPECT_EQ(wheel_data.axle, 1);
    EXPECT_NEAR(wheel_data.h, 0.0, 1E-3);
    EXPECT_EQ(wheel_data.index, 0);
    EXPECT_NEAR(wheel_data.p, 0.0, 1E-3);
    EXPECT_NEAR(wheel_data.x, 0.0, 1E-3);
    EXPECT_NEAR(wheel_data.y, -0.840, 1E-3);
    EXPECT_NEAR(wheel_data.z, 0.4, 1E-3);

    SE_GetObjectWheelData(SE_GetId(0), 3, &wheel_data);
    EXPECT_EQ(wheel_data.axle, 1);
    EXPECT_NEAR(wheel_data.h, 0.0, 1E-3);
    EXPECT_EQ(wheel_data.index, 1);
    EXPECT_NEAR(wheel_data.p, 0.0, 1E-3);
    EXPECT_NEAR(wheel_data.x, 0.0, 1E-3);
    EXPECT_NEAR(wheel_data.y, 0.840, 1E-3);
    EXPECT_NEAR(wheel_data.z, 0.4, 1E-3);

    while (SE_GetSimulationTime() < 5.81f)
    {
        SE_StepDT(0.1f);
    }

    SE_GetObjectWheelData(SE_GetId(2), 0, &wheel_data);
    EXPECT_EQ(wheel_data.axle, 0);
    EXPECT_NEAR(wheel_data.h, 0.054, 1E-3);
    EXPECT_EQ(wheel_data.index, 0);
    EXPECT_NEAR(wheel_data.p, 1.989, 1E-3);
    EXPECT_NEAR(wheel_data.x, 2.98, 1E-3);
    EXPECT_NEAR(wheel_data.y, -0.840, 1E-3);
    EXPECT_NEAR(wheel_data.z, 0.4, 1E-3);

    SE_GetObjectWheelData(SE_GetId(2), 1, &wheel_data);
    EXPECT_EQ(wheel_data.axle, 0);
    EXPECT_NEAR(wheel_data.h, 0.054, 1E-3);
    EXPECT_EQ(wheel_data.index, 1);
    EXPECT_NEAR(wheel_data.p, 1.989, 1E-3);
    EXPECT_NEAR(wheel_data.x, 2.98, 1E-3);
    EXPECT_NEAR(wheel_data.y, 0.840, 1E-3);
    EXPECT_NEAR(wheel_data.z, 0.4, 1E-3);

    SE_GetObjectWheelData(SE_GetId(2), 2, &wheel_data);
    EXPECT_EQ(wheel_data.axle, 1);
    EXPECT_NEAR(wheel_data.h, 0.0, 1E-3);
    EXPECT_EQ(wheel_data.index, 0);
    EXPECT_NEAR(wheel_data.p, 1.989, 1E-3);
    EXPECT_NEAR(wheel_data.x, 0.0, 1E-3);
    EXPECT_NEAR(wheel_data.y, -0.840, 1E-3);
    EXPECT_NEAR(wheel_data.z, 0.4, 1E-3);

    SE_GetObjectWheelData(SE_GetId(2), 3, &wheel_data);
    EXPECT_EQ(wheel_data.axle, 1);
    EXPECT_NEAR(wheel_data.h, 0.0, 1E-3);
    EXPECT_EQ(wheel_data.index, 1);
    EXPECT_NEAR(wheel_data.p, 1.989, 1E-3);
    EXPECT_NEAR(wheel_data.x, 0.0, 1E-3);
    EXPECT_NEAR(wheel_data.y, 0.840, 1E-3);
    EXPECT_NEAR(wheel_data.z, 0.4, 1E-3);

    SE_Close();
}

TEST(RoutingTest, TestRouteStatus)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/route_detour.xosc";

    ASSERT_EQ(SE_Init(scenario_file.c_str(), 1, 0, 0, 0), 0);
    EXPECT_EQ(SE_GetNumberOfObjects(), 1);
    EXPECT_EQ(SE_GetObjectRouteStatus(SE_GetId(0)), 2);

    while (SE_GetSimulationTime() < 0.7f)
    {
        SE_StepDT(0.1f);
    }
    EXPECT_EQ(SE_GetObjectRouteStatus(SE_GetId(0)), 1);

    while (SE_GetSimulationTime() < 8.9f)
    {
        SE_StepDT(0.1f);
    }
    EXPECT_EQ(SE_GetObjectRouteStatus(SE_GetId(0)), 2);

    SE_Close();
}

TEST(RoutingTest, TestRoutePointsWithGhost)
{
    std::string scenario_file = "../../../EnvironmentSimulator/Unittest/xosc/ghost_route.xosc";

    ASSERT_EQ(SE_Init(scenario_file.c_str(), 0, 0, 0, 0), 0);
    EXPECT_EQ(SE_GetNumberOfObjects(), 2);
    EXPECT_EQ(SE_GetObjectRouteStatus(SE_GetId(0)), 2);

    int ghost_id = SE_GetObjectGhostId(SE_GetId(0));
    ASSERT_EQ(ghost_id, 1);

    EXPECT_EQ(SE_GetObjectRouteStatus(ghost_id), 2);

    int n_wp = SE_GetNumberOfRoutePoints(SE_GetId(0));
    EXPECT_EQ(n_wp, 9);

    SE_RouteInfo ri[2];
    for (int i = 0; i < n_wp; i++)
    {
        SE_GetRoutePoint(SE_GetId(0), i, &ri[0]);
        SE_GetRoutePoint(ghost_id, i, &ri[1]);
        EXPECT_EQ(ri[0].junctionId, ri[1].junctionId);
        EXPECT_EQ(ri[0].laneId, ri[1].laneId);
        EXPECT_EQ(ri[0].osiLaneId, ri[1].osiLaneId);
        EXPECT_EQ(ri[0].roadId, ri[1].roadId);
        EXPECT_EQ(ri[0].s, ri[1].s);
        EXPECT_EQ(ri[0].t, ri[1].t);
        EXPECT_EQ(ri[0].x, ri[1].x);
        EXPECT_EQ(ri[0].y, ri[1].y);
    }

    SE_GetRoutePoint(SE_GetId(0), 0, &ri[0]);
    EXPECT_EQ(ri[0].junctionId, -1);
    EXPECT_EQ(ri[0].roadId, 5);
    EXPECT_EQ(ri[0].laneId, -1);
    EXPECT_NEAR(ri[0].s, 490.0, 1e-3);
    EXPECT_NEAR(ri[0].x, 221.5, 1e-3);
    EXPECT_NEAR(ri[0].y, -32.8318, 1e-3);

    SE_GetRoutePoint(SE_GetId(0), 5, &ri[0]);
    EXPECT_EQ(ri[0].junctionId, 300);
    EXPECT_EQ(ri[0].roadId, 302);
    EXPECT_EQ(ri[0].laneId, -1);
    EXPECT_NEAR(ri[0].s, 13.2, 1e-3);
    EXPECT_NEAR(ri[0].x, -61.5, 1e-3);
    EXPECT_NEAR(ri[0].y, 96.8, 1e-3);

    SE_Close();
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

#if 0  // set to 1 and modify filter to run one single test
	testing::GTEST_FLAG(filter) = "*AssignRoleTest*";
	// Or make use of launch argument, e.g. --gtest_filter=TestFetchImage*
#else
    // TxtLogger::Inst().SetLogFilePath("log-test.txt");

    if (argc > 1)
    {
        if (!strcmp(argv[1], "--disable_stdout"))
        {
            //  disable logging to stdout from the esminiLib
            SE_SetOptionPersistent("disable_stdout");

            // disable logging to stdout from the test cases
            SE_Env::Inst().GetOptions().SetOptionValue("disable_stdout", "", false, true);
        }
        else
        {
            printf("Usage: %s [--disable_stout] [google test options...]\n", argv[0]);
            return -1;
        }
    }

#endif

    return RUN_ALL_TESTS();
}
