#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "esminiRMLib.hpp"
#include "CommonMini.hpp"
#include "TestHelper.hpp"

TEST(TestSetMethods, SetRoadId)
{
    const char* odr_file = "../../../resources/xodr/fabriksgatan.xodr";

    ASSERT_EQ(RM_Init(odr_file), 0);

    int pos_handle = RM_CreatePosition();

    EXPECT_EQ(pos_handle, 0);

    RM_SetLanePosition(pos_handle, 0, 1, 0.0, 10, true);
    RM_PositionData pos_data;
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_NEAR(pos_data.h, 1.79437, 1E-5);
    EXPECT_NEAR(pos_data.x, 31.11617, 1E-5);
    EXPECT_NEAR(pos_data.y, -19.56364, 1E-5);
    EXPECT_EQ(pos_data.laneId, 1);
    EXPECT_NEAR(pos_data.laneOffset, 0.0, 1E-5);
    EXPECT_NEAR(pos_data.s, 10.0, 1E-5);

    // Stay on same position, but snap to road 3
    RM_SetRoadId(pos_handle, 3);
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_EQ(pos_data.roadId, 3);
    EXPECT_NEAR(pos_data.h, 1.79437, 1E-5);
    EXPECT_NEAR(pos_data.x, 31.11617, 1E-5);
    EXPECT_NEAR(pos_data.y, -19.56364, 1E-5);
    EXPECT_EQ(pos_data.laneId, -1);
    EXPECT_NEAR(pos_data.laneOffset, -15.71443, 1E-5);
    EXPECT_NEAR(pos_data.s, RM_GetRoadLength(3), 1E-5);  // position is past length of road 3

    RM_Close();
}

TEST(TestSetMethods, SetWorldPosition)
{
    const char* odr_file = "../../../resources/xodr/fabriksgatan.xodr";

    ASSERT_EQ(RM_Init(odr_file), 0);

    int pos_handle = RM_CreatePosition();

    EXPECT_EQ(pos_handle, 0);

    RM_SetWorldXYHPosition(pos_handle, 6.6f, -7.23f, 5.0f);
    RM_PositionData pos_data;
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_NEAR(pos_data.x, 6.6, 1E-5);
    EXPECT_NEAR(pos_data.y, -7.23, 1E-5);
    EXPECT_NEAR(pos_data.z, 0.0, 1E-5);
    EXPECT_NEAR(pos_data.h, 5.0, 1E-5);
    EXPECT_NEAR(pos_data.p, 0.0, 1E-5);
    EXPECT_NEAR(pos_data.r, 0.0, 1E-5);
    EXPECT_EQ(pos_data.roadId, 3);
    EXPECT_EQ(pos_data.laneId, -1);
    EXPECT_NEAR(pos_data.laneOffset, 0.04858, 1E-5);
    EXPECT_NEAR(pos_data.s, 102.54887, 1E-5);

    RM_SetWorldXYZHPosition(pos_handle, 9.36f, -3.05f, 0.55f, 3.4f);
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_NEAR(pos_data.x, 9.36, 1E-5);
    EXPECT_NEAR(pos_data.y, -3.05, 1E-5);
    EXPECT_NEAR(pos_data.z, 0.55, 1E-5);
    EXPECT_NEAR(pos_data.h, 3.4, 1E-5);
    EXPECT_NEAR(pos_data.p, 0.0, 1E-5);
    EXPECT_NEAR(pos_data.r, 0.0, 1E-5);
    EXPECT_EQ(pos_data.roadId, 3);
    EXPECT_EQ(pos_data.laneId, 1);
    EXPECT_NEAR(pos_data.laneOffset, 0.28348, 1E-5);
    EXPECT_NEAR(pos_data.s, 105.88660, 1E-5);

    RM_SetWorldPosition(pos_handle, 40.0f, -1.80f, -0.3f, 0.25f, 2.0f, 3.0f);
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_NEAR(pos_data.x, 40.0, 1E-5);
    EXPECT_NEAR(pos_data.y, -1.8, 1E-5);
    EXPECT_NEAR(pos_data.z, -0.3, 1E-5);
    EXPECT_NEAR(pos_data.h, 0.25, 1E-5);
    EXPECT_NEAR(pos_data.p, 2.0, 1E-5);
    EXPECT_NEAR(pos_data.r, 3.0, 1E-5);
    EXPECT_EQ(pos_data.roadId, 1);
    EXPECT_EQ(pos_data.laneId, -1);
    EXPECT_NEAR(pos_data.laneOffset, -0.10529, 1E-5);
    EXPECT_NEAR(pos_data.s, 6.62796, 1E-5);

    RM_SetWorldPositionMode(pos_handle, 24.0f, 16.0f, 0.8f, 4.8f, std::nanf(""), 0.0f, RM_PositionMode::RM_Z_ABS | RM_PositionMode::RM_H_ABS);
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_NEAR(pos_data.x, 24.0, 1E-5);
    EXPECT_NEAR(pos_data.y, 16.0, 1E-5);
    EXPECT_NEAR(pos_data.z, 0.8, 1E-5);
    EXPECT_NEAR(pos_data.h, 4.8, 1E-5);
    EXPECT_NEAR(pos_data.p, 2.0, 1E-5);  // from last setting
    EXPECT_NEAR(pos_data.r, 0.0, 1E-5);
    EXPECT_EQ(pos_data.roadId, 2);
    EXPECT_EQ(pos_data.laneId, 1);
    EXPECT_NEAR(pos_data.laneOffset, 0.03118, 1E-5);
    EXPECT_NEAR(pos_data.s, 293.27118, 1E-5);

    RM_SetWorldPositionMode(pos_handle, 24.0f, 16.0f, 0.8f, 0.0f, 0.0f, 0.0f, RM_PositionMode::RM_Z_ABS | RM_PositionMode::RM_H_REL);
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_NEAR(pos_data.x, 24.0, 1E-5);
    EXPECT_NEAR(pos_data.y, 16.0, 1E-5);
    EXPECT_NEAR(pos_data.z, 0.8, 1E-5);
    EXPECT_NEAR(pos_data.h, 4.89411, 1E-5);
    EXPECT_NEAR(pos_data.p, 0.0, 1E-5);
    EXPECT_NEAR(pos_data.r, 0.0, 1E-5);
    EXPECT_EQ(pos_data.roadId, 2);
    EXPECT_EQ(pos_data.laneId, 1);
    EXPECT_NEAR(pos_data.laneOffset, 0.03118, 1E-5);
    EXPECT_NEAR(pos_data.s, 293.27118, 1E-5);

    RM_Close();
}

TEST(TestRelativeChecks, SubtractPositionsIntersection)
{
    const char* odr_file = "../../../resources/xodr/fabriksgatan.xodr";

    ASSERT_EQ(RM_Init(odr_file), 0);

    int             pA = RM_CreatePosition();
    int             pB = RM_CreatePosition();
    RM_PositionData pos_data;

    EXPECT_EQ(pA, 0);
    EXPECT_EQ(pB, 1);

    RM_SetWorldXYHPosition(pA, 33.0f, -27.0f, 1.57f);
    RM_GetPositionData(pA, &pos_data);
    EXPECT_NEAR(pos_data.roadId, 0, 1E-5);
    EXPECT_EQ(pos_data.laneId, 1);

    RM_SetWorldXYHPosition(pB, 22.0f, 27.0f, 1.57f);
    RM_GetPositionData(pB, &pos_data);
    EXPECT_NEAR(pos_data.roadId, 2, 1E-5);
    EXPECT_EQ(pos_data.laneId, 1);

    // B in front of A through intersection
    RM_PositionDiff pos_diff;
    RM_SubtractAFromB(pA, pB, &pos_diff);
    EXPECT_NEAR(pos_diff.ds, 55.152, 1E-3);
    EXPECT_NEAR(pos_diff.dt, -0.114, 1E-3);
    EXPECT_EQ(pos_diff.dLaneId, 0);

    // A behind B through intersection
    RM_SubtractAFromB(pB, pA, &pos_diff);
    EXPECT_NEAR(pos_diff.ds, -55.152, 1E-3);
    EXPECT_NEAR(pos_diff.dt, 0.114, 1E-3);
    EXPECT_EQ(pos_diff.dLaneId, 0);

    // Place A in opposite lane
    RM_SetWorldXYHPosition(pA, 29.0f, -26.0f, 4.71f);
    RM_GetPositionData(pA, &pos_data);
    EXPECT_NEAR(pos_data.roadId, 0, 1E-5);
    EXPECT_EQ(pos_data.laneId, -1);

    // B behind A through intersection, different lanes
    RM_SubtractAFromB(pA, pB, &pos_diff);
    EXPECT_NEAR(pos_diff.ds, -53.378, 1E-3);
    EXPECT_NEAR(pos_diff.dt, 3.555, 1E-3);
    EXPECT_EQ(pos_diff.dLaneId, 1);

    RM_Close();
}

TEST(TestRelativeChecks, SubtractPositionsHW)
{
    const char* odr_file = "../../../EnvironmentSimulator/Unittest/xodr/mw_100m.xodr";

    ASSERT_EQ(RM_Init(odr_file), 0);

    int             pA = RM_CreatePosition();
    int             pB = RM_CreatePosition();
    RM_PositionData pos_data;

    EXPECT_EQ(pA, 0);
    EXPECT_EQ(pB, 1);

    // A at right side of road in middle lane at s=50
    RM_SetLanePosition(pA, 1, -3, 0.0, 50.0, true);
    RM_GetPositionData(pA, &pos_data);
    EXPECT_NEAR(pos_data.x, 50.0, 1E-3);
    EXPECT_NEAR(pos_data.y, -8.0, 1E-3);
    EXPECT_NEAR(pos_data.h, 0.0, 1E-3);

    // B 20 m in front of A, one lane to the right
    RM_SetLanePosition(pB, 1, -4, 0.0, 70.0, true);
    RM_GetPositionData(pB, &pos_data);
    EXPECT_NEAR(pos_data.x, 70.0, 1E-3);
    EXPECT_NEAR(pos_data.y, -11.5, 1E-3);
    EXPECT_NEAR(pos_data.h, 0.0, 1E-3);

    RM_PositionDiff pos_diff;
    RM_SubtractAFromB(pA, pB, &pos_diff);
    EXPECT_NEAR(pos_diff.ds, 20.0, 1E-3);
    EXPECT_NEAR(pos_diff.dt, -3.5, 1E-3);

    // B 20 m behind A, one lane to the left
    RM_SetLanePosition(pB, 1, -2, 0.0, 30.0, true);
    RM_GetPositionData(pB, &pos_data);
    EXPECT_NEAR(pos_data.x, 30.0, 1E-3);
    EXPECT_NEAR(pos_data.y, -4.425, 1E-3);
    EXPECT_NEAR(pos_data.h, 0.0, 1E-3);

    RM_SubtractAFromB(pA, pB, &pos_diff);
    EXPECT_NEAR(pos_diff.ds, -20.0, 1E-3);
    EXPECT_NEAR(pos_diff.dt, 3.575, 1E-3);

    // Same tests in opposite direction

    // A at left side of road in middle lane at s=50
    RM_SetLanePosition(pA, 1, 3, 0.0, 50.0, true);
    RM_GetPositionData(pA, &pos_data);
    EXPECT_NEAR(pos_data.x, 50.0, 1E-3);
    EXPECT_NEAR(pos_data.y, 8.0, 1E-3);
    EXPECT_NEAR(pos_data.h, 3.142, 1E-3);

    // B 20 m in front of A, one lane to the right
    RM_SetLanePosition(pB, 1, 4, 0.0, 30.0, true);
    RM_GetPositionData(pB, &pos_data);
    EXPECT_NEAR(pos_data.x, 30.0, 1E-3);
    EXPECT_NEAR(pos_data.y, 11.7, 1E-3);
    EXPECT_NEAR(pos_data.h, 3.142, 1E-3);

    RM_SubtractAFromB(pA, pB, &pos_diff);
    EXPECT_NEAR(pos_diff.ds, 20.0, 1E-3);
    EXPECT_NEAR(pos_diff.dt, 3.7, 1E-3);

    // B 20 m behind A, one lane to the left
    RM_SetLanePosition(pB, 1, 2, 0.0, 70.0, true);
    RM_GetPositionData(pB, &pos_data);
    EXPECT_NEAR(pos_data.x, 70.0, 1E-3);
    EXPECT_NEAR(pos_data.y, 4.425, 1E-3);
    EXPECT_NEAR(pos_data.h, 3.142, 1E-3);

    RM_SubtractAFromB(pA, pB, &pos_diff);
    EXPECT_NEAR(pos_diff.ds, -20.0, 1E-3);
    EXPECT_NEAR(pos_diff.dt, -3.575, 1E-3);

    RM_Close();
}

TEST(TestProbe, TestSimpleProbe)
{
    const char* odr_file = "../../../resources/xodr/straight_500m.xodr";

    ASSERT_EQ(RM_Init(odr_file), 0);

    int pos_handle = RM_CreatePosition();

    EXPECT_EQ(pos_handle, 0);

    RM_SetLanePosition(pos_handle, 1, -1, 0.0, 10, true);
    RM_PositionData pos_data;
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_NEAR(pos_data.h, 0.0, 1E-5);
    EXPECT_NEAR(pos_data.x, 10.0, 1E-5);
    EXPECT_NEAR(pos_data.y, -1.535, 1E-5);
    EXPECT_EQ(pos_data.laneId, -1);
    EXPECT_NEAR(pos_data.laneOffset, 0.0, 1E-5);
    EXPECT_NEAR(pos_data.s, 10.0, 1E-5);

    RM_RoadProbeInfo info;
    RM_GetProbeInfo(0, 30, &info, 0, true);
    EXPECT_EQ(info.road_lane_info.roadId, 1);
    EXPECT_NEAR(info.road_lane_info.heading, 0.0, 1E-5);
    EXPECT_NEAR(info.road_lane_info.pos.x, 40.0, 1E-5);
    EXPECT_NEAR(info.road_lane_info.pos.y, -1.535, 1E-5);
    EXPECT_EQ(info.road_lane_info.laneId, -1);
    EXPECT_NEAR(info.road_lane_info.laneOffset, 0.0, 1E-5);
    EXPECT_NEAR(info.road_lane_info.s, 40.0, 1E-5);

    RM_SetLanePosition(pos_handle, 1, 1, 0.0, 100, true);
    RM_GetProbeInfo(0, 30, &info, 0, true);
    EXPECT_EQ(info.road_lane_info.roadId, 1);
    EXPECT_NEAR(info.road_lane_info.heading, 0.0, 1E-5);
    EXPECT_NEAR(info.road_lane_info.pos.x, 70.0, 1E-5);
    EXPECT_NEAR(info.road_lane_info.pos.y, 1.535, 1E-5);
    EXPECT_EQ(info.road_lane_info.laneId, 1);
    EXPECT_NEAR(info.road_lane_info.laneOffset, 0.0, 1E-5);
    EXPECT_NEAR(info.road_lane_info.s, 70.0, 1E-5);
    EXPECT_NEAR(info.relative_h, 0.0, 1E-5);
    EXPECT_NEAR(info.relative_pos.x, 30.0, 1E-5);
    EXPECT_NEAR(info.relative_pos.y, 0.0, 1E-5);
    EXPECT_NEAR(info.relative_pos.z, 0.0, 1E-5);

    RM_RoadLaneInfo r_info;
    RM_GetLaneInfo(pos_handle, 30.0, &r_info, 0, true);
    EXPECT_NEAR(r_info.curvature, 0.0, 1E-5);
    EXPECT_NEAR(r_info.s, 70.0, 1E-5);

    RM_Close();
}

TEST(TestLaneType, TestDetailedLaneType)
{
    const char* odr_file = "../../../EnvironmentSimulator/Unittest/xodr/mw_100m.xodr";

    ASSERT_EQ(RM_Init(odr_file), 0);

    int pos_handle = RM_CreatePosition();

    EXPECT_EQ(pos_handle, 0);

    RM_SetSnapLaneTypes(pos_handle, -1);
    RM_SetLanePosition(pos_handle, 1, -3, 0.0, 100.0, false);
    EXPECT_EQ(RM_GetInLaneType(pos_handle), 2);

    RM_SetWorldXYHPosition(pos_handle, 34.35f, -13.40f, 0.0f);
    int lane_type = RM_GetInLaneType(pos_handle);
    EXPECT_EQ(lane_type, 128);
    EXPECT_EQ(lane_type & 1966594, 0);
    EXPECT_NE(lane_type & 1966726, 0);

    RM_SetWorldXYHPosition(pos_handle, 43.17f, -14.81f, 0.0f);
    lane_type = RM_GetInLaneType(pos_handle);
    EXPECT_EQ(lane_type, 4);
    EXPECT_EQ(lane_type & 1966594, 0);
    EXPECT_NE(lane_type & 1966726, 0);

    RM_SetWorldXYHPosition(pos_handle, 49.65f, -16.95f, 0.0f);
    lane_type = RM_GetInLaneType(pos_handle);
    EXPECT_EQ(lane_type, 64);
    EXPECT_EQ(lane_type & 1966594, 0);
    EXPECT_EQ(lane_type & 1966726, 0);

    RM_SetWorldXYHPosition(pos_handle, 60.24f, -20.29f, 0.0f);
    lane_type = RM_GetInLaneType(pos_handle);
    EXPECT_EQ(lane_type, 64);
    EXPECT_EQ(lane_type & 1966594, 0);
    EXPECT_EQ(lane_type & 1966726, 0);

    RM_SetWorldXYHPosition(pos_handle, 74.24f, -24.69f, 0.0f);
    lane_type = RM_GetInLaneType(pos_handle);
    EXPECT_EQ(lane_type, 1);
    EXPECT_EQ(lane_type & 1966594, 0);
    EXPECT_EQ(lane_type & 1966726, 0);

    RM_Close();
}

TEST(TestLaneDirection, TestLaneDirectionByRule)
{
    const char* odr_file[2] = {"../../../resources/xodr/e6mini.xodr", "../../../resources/xodr/e6mini-lht.xodr"};

    for (int i = 0; i < 2; i++)
    {
        ASSERT_EQ(RM_Init(odr_file[i]), 0);

        int             pos_handle = RM_CreatePosition();
        RM_PositionData r_data;

        EXPECT_EQ(pos_handle, 0);

        // going along road s-axis
        RM_SetLanePosition(pos_handle, 0, -3, 0.0, 100.0, true);
        RM_GetPositionData(pos_handle, &r_data);

        if (i == 0)
        {
            EXPECT_NEAR(r_data.h, 1.566, 1e-3);
        }
        else
        {
            EXPECT_NEAR(r_data.h, 4.708, 1e-3);
        }

        // going opposite road s-axis
        RM_SetLanePosition(pos_handle, 0, 3, 0.0, 100.0, true);
        RM_GetPositionData(pos_handle, &r_data);

        if (i == 0)
        {
            EXPECT_NEAR(r_data.h, 4.708, 1e-3);
        }
        else
        {
            EXPECT_NEAR(r_data.h, 1.566, 1e-3);
        }

        RM_Close();
    }
}

TEST(RoadId, TestStringRoadId)
{
    const char*     odr_file = "../../../EnvironmentSimulator/Unittest/xodr/fabriksgatan_mixed_id_types.xodr";
    RM_PositionData pos_data;

    ASSERT_EQ(RM_Init(odr_file), 0);
    ASSERT_EQ(RM_GetNumberOfRoads(), 16);

    EXPECT_STREQ(RM_GetRoadIdString(3), "Kalle");
    EXPECT_STREQ(RM_GetRoadIdString(1), "1");
    EXPECT_STREQ(RM_GetRoadIdString(128), "2Kalle3");
    EXPECT_STREQ(RM_GetRoadIdString(0), "0");
    EXPECT_STREQ(RM_GetJunctionIdString(0), "Junction4");

    int pos_handle = RM_CreatePosition();

    RM_SetLanePosition(pos_handle, 2, -1, 0.0, 20, true);
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_EQ(pos_data.roadId, 2);

    RM_SetLanePosition(pos_handle, 0, 1, 0.0, 10, true);
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_EQ(pos_data.roadId, 0);

    RM_SetLanePosition(pos_handle, RM_GetRoadIdFromString("2"), 1, 0.0, 10, true);
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_EQ(pos_data.roadId, 2);
    EXPECT_EQ(pos_data.junctionId, -1);

    RM_SetLanePosition(pos_handle, RM_GetRoadIdFromString("Kalle"), 1, 0.0, 10, true);
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_EQ(pos_data.roadId, 3);
    EXPECT_EQ(pos_data.junctionId, -1);

    RM_SetLanePosition(pos_handle, RM_GetRoadIdFromString("2Kalle3"), 1, 0.0, 10, true);
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_EQ(pos_data.roadId, 128);
    EXPECT_EQ(pos_data.junctionId, 0);

    EXPECT_EQ(RM_GetJunctionIdFromString("Junction4"), 0);

    RM_Close();
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    if (!strcmp(argv[1], "--disable_stdout"))
    {
        RM_EnableConsoleLogging(false, true);
        RM_SetLogFilePath("log-test.txt");
    }

    if (ParseAndSetLoggerOptions(argc, argv) != 0)
    {
        return -1;
    }

    // testing::GTEST_FLAG(filter) = "*check_GroundTruth_including_init_state*";

    return RUN_ALL_TESTS();
}