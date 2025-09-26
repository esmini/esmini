#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "esminiRMLib.hpp"
#include "CommonMini.hpp"

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
    EXPECT_EQ(pos_data.roadId, 0);
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
    EXPECT_NEAR(pos_data.h, 3.3915, 1E-3);
    EXPECT_NEAR(pos_data.p, 1.1415, 1E-3);
    EXPECT_NEAR(pos_data.r, 6.1415, 1E-3);
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
    EXPECT_NEAR(pos_data.p, 1.1415, 1E-3);  // from last setting
    EXPECT_NEAR(pos_data.r, 0.0, 1E-5);
    EXPECT_EQ(pos_data.roadId, 2);
    EXPECT_EQ(pos_data.laneId, 1);
    EXPECT_NEAR(pos_data.laneOffset, 0.0311, 1E-3);
    EXPECT_NEAR(pos_data.s, 293.2713, 1E-3);

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
    EXPECT_NEAR(pos_data.laneOffset, 0.0311, 1E-3);
    EXPECT_NEAR(pos_data.s, 293.2713, 1E-3);

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
    EXPECT_NEAR(pos_diff.ds, 55.1497, 1E-3);
    EXPECT_NEAR(pos_diff.dt, -0.114, 1E-3);
    EXPECT_EQ(pos_diff.dLaneId, 0);

    // A behind B through intersection
    RM_SubtractAFromB(pB, pA, &pos_diff);
    EXPECT_NEAR(pos_diff.ds, -55.1497, 1E-3);
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

TEST(TestProbe, TestSimpleRoadTypes)
{
    const char* odr_file = "../../../EnvironmentSimulator/Unittest/xodr/straight_500m_signs.xodr";

    ASSERT_EQ(RM_Init(odr_file), 0);

    int pos_handle = RM_CreatePosition();

    EXPECT_EQ(pos_handle, 0);

    RM_SetLanePosition(pos_handle, 1, -1, 0.0, 0.0, false);
    RM_RoadProbeInfo info;
    const float      tolerance = 1e-3f;
    float            distance  = 200.0f;

    RM_GetProbeInfo(pos_handle, 100.0f, &info, 0, true);
    EXPECT_EQ(info.road_lane_info.road_rule, 1);
    EXPECT_NEAR(info.road_lane_info.speed_limit, 10.0f, 1e-3f);

    for (int i = 0; i < 14; i++)
    {
        RM_GetProbeInfo(pos_handle, distance, &info, 0, true);
        EXPECT_EQ(info.road_lane_info.road_rule, 1);
        if (i == 0)  // m/s entry
        {
            EXPECT_EQ(info.road_lane_info.road_type, i);
            EXPECT_NEAR(info.road_lane_info.speed_limit, 10.0f, tolerance);
        }
        else if (i == 1)  // mph entry
        {
            EXPECT_EQ(info.road_lane_info.road_type, i);
            EXPECT_NEAR(info.road_lane_info.speed_limit, 20.0f * 0.44704f, tolerance);
        }
        else if (i == 13)
        {
            EXPECT_EQ(info.road_lane_info.road_type, 12);
            EXPECT_NEAR(info.road_lane_info.speed_limit, 130.0f / 3.6f, tolerance);
        }
        else
        {
            EXPECT_EQ(info.road_lane_info.road_type, i);
            EXPECT_NEAR(info.road_lane_info.speed_limit, static_cast<float>((i + 1) * 10) / 3.6f, tolerance);
        }
        distance += 25.0f;
    }

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

TEST(TestSetMethods, TestSetH)
{
    const char*     odr_file = "../../../EnvironmentSimulator/Unittest/xodr/slope_up_slope_down.xodr";
    RM_PositionData pos_data;

    ASSERT_EQ(RM_Init(odr_file), 0);
    ASSERT_EQ(RM_GetNumberOfRoads(), 1);

    int pos_handle = RM_CreatePosition();

    RM_SetLanePosition(pos_handle, 1, -1, 0.0f, 25.0f, true);
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_NEAR(pos_data.roadId, 1, 1e-3);
    EXPECT_NEAR(pos_data.x, 18.9151, 1e-3);
    EXPECT_NEAR(pos_data.y, 16.4402, 1e-3);
    EXPECT_NEAR(pos_data.z, 12.5, 1e-3);
    EXPECT_NEAR(pos_data.h, 0.7853, 1e-3);
    EXPECT_NEAR(pos_data.p, 5.8195, 1e-3);
    EXPECT_NEAR(pos_data.r, 0.0, 1e-3);

    // set absolute heading quarter of circle left
    RM_SetH(pos_handle, static_cast<float>(M_PI_2));
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_NEAR(pos_data.roadId, 1, 1e-3);
    EXPECT_NEAR(pos_data.x, 18.9151, 1e-3);
    EXPECT_NEAR(pos_data.y, 16.4402, 1e-3);
    EXPECT_NEAR(pos_data.z, 12.5, 1e-3);
    EXPECT_NEAR(pos_data.h, 1.6264, 1e-3);
    EXPECT_NEAR(pos_data.p, 5.9614, 1e-3);
    EXPECT_NEAR(pos_data.r, 5.9433, 1e-3);

    // set relative (road) heading quarter of circle right
    RM_SetHMode(pos_handle, static_cast<float>(2.0 * M_PI * 7.0 / 8.0), RM_PositionMode::RM_H_REL);
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_NEAR(pos_data.roadId, 1, 1e-3);
    EXPECT_NEAR(pos_data.x, 18.9151, 1e-3);
    EXPECT_NEAR(pos_data.y, 16.4402, 1e-3);
    EXPECT_NEAR(pos_data.z, 12.5, 1e-3);
    EXPECT_NEAR(pos_data.h, 6.2275, 1e-3);
    EXPECT_NEAR(pos_data.p, 5.9614, 1e-3);
    EXPECT_NEAR(pos_data.r, 0.3398, 1e-3);

    // set aboslute heading quarter of circle left (again) using mode variant of function
    RM_SetHMode(pos_handle, static_cast<float>(M_PI_2), RM_PositionMode::RM_H_ABS);
    RM_GetPositionData(pos_handle, &pos_data);
    EXPECT_NEAR(pos_data.roadId, 1, 1e-3);
    EXPECT_NEAR(pos_data.x, 18.9151, 1e-3);
    EXPECT_NEAR(pos_data.y, 16.4402, 1e-3);
    EXPECT_NEAR(pos_data.z, 12.5, 1e-3);
    EXPECT_NEAR(pos_data.h, 1.6264, 1e-3);
    EXPECT_NEAR(pos_data.p, 5.9614, 1e-3);
    EXPECT_NEAR(pos_data.r, 5.9433, 1e-3);

    RM_Close();
}

TEST(TestGetMethods, TestGetLaneMethods)
{
    const char* odr_file = "../../../resources/xodr/fabriksgatan.xodr";
    int         lane_id  = 0;

    ASSERT_EQ(RM_Init(odr_file), 0);
    ASSERT_EQ(RM_GetNumberOfRoads(), 16);

    EXPECT_EQ(RM_GetRoadNumberOfLanes(3, 100.0, -1), 7);  // -1 = any/all lanes
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 0, 100.0, -1, &lane_id), 0);
    EXPECT_EQ(lane_id, 3);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 1, 100.0, -1, &lane_id), 0);
    EXPECT_EQ(lane_id, 2);

    EXPECT_EQ(RM_GetLaneIdByIndex(3, 2, 100.0, -1, &lane_id), 0);
    EXPECT_EQ(lane_id, 1);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 3, 100.0, -1, &lane_id), 0);
    EXPECT_EQ(lane_id, 0);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 4, 100.0, -1, &lane_id), 0);
    EXPECT_EQ(lane_id, -1);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 5, 100.0, -1, &lane_id), 0);
    EXPECT_EQ(lane_id, -2);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 6, 100.0, -1, &lane_id), 0);
    EXPECT_EQ(lane_id, -3);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 7, 100.0, -1, &lane_id), -1);  // does not exist
    EXPECT_EQ(lane_id, -3);                                         // not changed

    EXPECT_EQ(RM_GetRoadNumberOfLanes(3, 100.0, 32), 2);  // 32 = sidewalk
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 0, 100.0, 32, &lane_id), 0);
    EXPECT_EQ(lane_id, 3);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 1, 100.0, 32, &lane_id), 0);
    EXPECT_EQ(lane_id, -3);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 2, 100.0, 32, &lane_id), -1);  // does not exist

    EXPECT_EQ(RM_GetRoadNumberOfLanes(3, 100.0, 64), 2);  // 64 = border
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 0, 100.0, 64, &lane_id), 0);
    EXPECT_EQ(lane_id, 2);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 1, 100.0, 64, &lane_id), 0);
    EXPECT_EQ(lane_id, -2);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 2, 100.0, 64, &lane_id), -1);  // does not exist

    EXPECT_EQ(RM_GetRoadNumberOfLanes(3, 100.0, 2), 2);  // 2 = driving
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 0, 100.0, 2, &lane_id), 0);
    EXPECT_EQ(lane_id, 1);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 1, 100.0, 2, &lane_id), 0);
    EXPECT_EQ(lane_id, -1);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 2, 100.0, 2, &lane_id), -1);  // does not exist

    EXPECT_EQ(RM_GetRoadNumberOfLanes(3, 100.0, 1966594), 2);  // 1966594 = any driving
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 0, 100.0, 1966594, &lane_id), 0);
    EXPECT_EQ(lane_id, 1);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 1, 100.0, 1966594, &lane_id), 0);
    EXPECT_EQ(lane_id, -1);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 2, 100.0, 1966594, &lane_id), -1);  // does not exist

    EXPECT_EQ(RM_GetRoadNumberOfDrivableLanes(3, 100.0), 2);
    EXPECT_EQ(RM_GetDrivableLaneIdByIndex(3, 0, 100.0, &lane_id), 0);
    EXPECT_EQ(lane_id, 1);
    EXPECT_EQ(RM_GetDrivableLaneIdByIndex(3, 1, 100.0, &lane_id), 0);
    EXPECT_EQ(lane_id, -1);
    EXPECT_EQ(RM_GetDrivableLaneIdByIndex(3, 2, 100.0, &lane_id), -1);  // does not exist

    EXPECT_EQ(RM_GetRoadNumberOfLanes(3, 100.0, 96), 4);  // 32 + 64 = sidewalk + border
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 0, 100.0, 96, &lane_id), 0);
    EXPECT_EQ(lane_id, 3);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 1, 100.0, 96, &lane_id), 0);
    EXPECT_EQ(lane_id, 2);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 2, 100.0, 96, &lane_id), 0);
    EXPECT_EQ(lane_id, -2);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 3, 100.0, 96, &lane_id), 0);
    EXPECT_EQ(lane_id, -3);
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 4, 100.0, 96, &lane_id), -1);  // does not exist

    EXPECT_EQ(RM_GetRoadNumberOfLanes(3, 1000.0, 2), -1);                // s out of range
    EXPECT_EQ(RM_GetLaneIdByIndex(3, 0, 1000.0, 2, &lane_id), -1);       // s out of range
    EXPECT_EQ(RM_GetRoadNumberOfDrivableLanes(3, 1000.0), -1);           // s out of range
    EXPECT_EQ(RM_GetDrivableLaneIdByIndex(3, 0, 1000.0, &lane_id), -1);  // s out of range

    RM_Close();
}

TEST(TestLoadRoad, TestLoadFromXMLString)
{
    std::string xml =
        R"(<?xml version='1.0' encoding='utf-8'?>
            <OpenDRIVE>
                <header name="tunnel_example" revMajor="1" revMinor="5" date="2025-09-09 09:40:00." north="0.0" south="0.0" east="0.0" west="0.0"/>
                <road rule="RHT" id="1" junction="-1" length="100">
                    <link/>
                    <planView>
                        <geometry s="0" x="0" y="0" hdg="0" length="100">
                            <line/>
                        </geometry>
                    </planView>
                    <elevationProfile/>
                    <lateralProfile/>
                    <lanes>
                        <laneSection s="0">
                            <left>
                                <lane id="1" type="driving" level="false">
                                    <link/>
                                    <width a="3.0" b="0.0" c="-0.0" d="0.0" sOffset="0"/>
                                    <roadMark sOffset="0" type="solid" weight="standard" color="standard" height="0.02" width="0.2"/>
                                </lane>
                            </left>
                            <center>
                                <lane id="0" type="none" level="false">
                                    <roadMark sOffset="0" type="broken" weight="standard" color="standard" height="0.02" width="0.2"/>
                                </lane>
                            </center>
                            <right>
                                <lane id="-1" type="driving" level="false">
                                    <link/>
                                    <width a="3.0" b="0.0" c="-0.0" d="0.0" sOffset="0"/>
                                    <roadMark sOffset="0" type="solid" weight="standard" color="standard" height="0.02" width="0.2"/>
                                </lane>
                            </right>
                        </laneSection>
                    </lanes>
                </road>
            </OpenDRIVE>
        )";

    ASSERT_EQ(RM_InitWithString(xml.c_str()), 0);
    ASSERT_EQ(RM_GetNumberOfRoads(), 1);
    EXPECT_EQ(RM_GetRoadNumberOfLanes(1, 100.0, -1), 3);  // -1 = any/all lanes

    RM_Close();
}

TEST(TestGetMethods, TestGetPosMethodsWithLaneOffset)
{
    ASSERT_EQ(RM_Init("../../../EnvironmentSimulator/Unittest/xodr/four_lanes_with_offset.xodr"), 0);
    ASSERT_EQ(RM_GetNumberOfRoads(), 1);

    // Create a position object
    int p0 = RM_CreatePosition();

    RM_RoadLaneInfo laneInfo;
    RM_PositionData posData;

    id_t  road_id = 1;
    int   lane_id;
    int   lookahead_mode;
    float s      = 9.0f;
    float offset = 0.0f;

    // reference line (lane 0) at far left, look along center of current lane
    lane_id        = 0;
    lookahead_mode = 0;
    RM_SetLanePosition(p0, road_id, lane_id, offset, s, true);
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    // Closest to lane -1 (shoulder), but belongs to 0 (none)
    EXPECT_EQ(RM_GetInLaneType(p0), 8);                   // shoulder
    EXPECT_EQ(RM_GetLaneType(p0, lane_id), 1);            // none
    EXPECT_EQ(RM_GetLaneTypeByRoadId(1, lane_id, s), 1);  // none
    EXPECT_EQ(laneInfo.laneId, 0);
    EXPECT_NEAR(laneInfo.laneOffset, 0.0, 1e-3);  // offset from lane center

    lane_id        = -1;
    lookahead_mode = 0;  // lookahead center current lane
    RM_SetLanePosition(p0, road_id, lane_id, offset, s, true);
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    // Closest to lane -1 (shoulder), and belongs to the same
    EXPECT_EQ(RM_GetInLaneType(p0), 8);
    EXPECT_EQ(RM_GetLaneType(p0, lane_id), 8);
    EXPECT_EQ(RM_GetLaneTypeByRoadId(1, lane_id, s), 8);
    EXPECT_EQ(laneInfo.laneId, -1);
    EXPECT_NEAR(laneInfo.laneOffset, 0.0, 1e-3);  // offset from lane center

    lookahead_mode = 1;  // lookahead road center
    RM_SetLanePosition(p0, road_id, lane_id, offset, s, true);
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    EXPECT_EQ(RM_GetInLaneType(p0), 8);                   // shoulder
    EXPECT_EQ(RM_GetLaneType(p0, lane_id), 8);            // shoulder
    EXPECT_EQ(RM_GetLaneTypeByRoadId(1, lane_id, s), 8);  // shoulder
    EXPECT_EQ(laneInfo.laneId, -2);
    EXPECT_NEAR(laneInfo.laneOffset, -1.5, 1e-3);  // offset from lane center

    lookahead_mode = 2;  // lookahead current lane offset
    RM_SetLanePosition(p0, road_id, lane_id, offset, s, true);
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    EXPECT_EQ(RM_GetInLaneType(p0), 8);                   // shoulder
    EXPECT_EQ(RM_GetLaneType(p0, lane_id), 8);            // shoulder
    EXPECT_EQ(RM_GetLaneTypeByRoadId(1, lane_id, s), 8);  // shoulder
    EXPECT_EQ(laneInfo.laneId, -1);
    EXPECT_NEAR(laneInfo.laneOffset, 0.0, 1e-3);  // offset from lane center

    lane_id        = -3;
    lookahead_mode = 0;  // lookahead center current lane
    RM_SetLanePosition(p0, road_id, lane_id, offset, s, true);
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    EXPECT_EQ(RM_GetInLaneType(p0), 2);                   // driving
    EXPECT_EQ(RM_GetLaneType(p0, lane_id), 2);            // driving
    EXPECT_EQ(RM_GetLaneTypeByRoadId(1, lane_id, s), 2);  // driving
    EXPECT_EQ(laneInfo.laneId, -3);
    EXPECT_NEAR(laneInfo.laneOffset, 0.0, 1e-3);  // offset from lane center

    lookahead_mode = 1;  // lookahead road center
    RM_SetLanePosition(p0, road_id, lane_id, offset, s, true);
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    EXPECT_EQ(RM_GetInLaneType(p0), 2);                   // driving
    EXPECT_EQ(RM_GetLaneType(p0, lane_id), 2);            // driving
    EXPECT_EQ(RM_GetLaneTypeByRoadId(1, lane_id, s), 2);  // driving
    EXPECT_EQ(laneInfo.laneId, -3);
    EXPECT_NEAR(laneInfo.laneOffset, 1.5, 1e-3);  // offset from lane center

    lookahead_mode = 2;  // lookahead current lane offset
    RM_SetLanePosition(p0, road_id, lane_id, offset, s, true);
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    EXPECT_EQ(RM_GetInLaneType(p0), 2);                   // driving
    EXPECT_EQ(RM_GetLaneType(p0, lane_id), 2);            // driving
    EXPECT_EQ(RM_GetLaneTypeByRoadId(1, lane_id, s), 2);  // driving
    EXPECT_EQ(laneInfo.laneId, -3);
    EXPECT_NEAR(laneInfo.laneOffset, 0.0, 1e-3);  // offset from lane center

    lane_id        = -4;
    lookahead_mode = 0;  // lookahead center current lane
    RM_SetLanePosition(p0, road_id, lane_id, offset, s, true);
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    EXPECT_EQ(RM_GetInLaneType(p0), 8);                   // shoulder
    EXPECT_EQ(RM_GetLaneType(p0, lane_id), 8);            // shoulder
    EXPECT_EQ(RM_GetLaneTypeByRoadId(1, lane_id, s), 8);  // shoulder
    EXPECT_EQ(laneInfo.laneId, -4);
    EXPECT_NEAR(laneInfo.laneOffset, 0.0, 1e-3);  // offset from lane center

    lookahead_mode = 1;  // lookahead road center
    RM_SetLanePosition(p0, road_id, lane_id, offset, s, true);
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    EXPECT_EQ(RM_GetInLaneType(p0), 8);                   // shoulder
    EXPECT_EQ(RM_GetLaneType(p0, lane_id), 8);            // shoulder
    EXPECT_EQ(RM_GetLaneTypeByRoadId(1, lane_id, s), 8);  // shoulder
    EXPECT_EQ(laneInfo.laneId, -3);
    EXPECT_NEAR(laneInfo.laneOffset, 1.5, 1e-3);  // offset from lane center

    lookahead_mode = 2;  // lookahead current lane offset
    RM_SetLanePosition(p0, road_id, lane_id, offset, s, true);
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    EXPECT_EQ(RM_GetInLaneType(p0), 8);                   // shoulder
    EXPECT_EQ(RM_GetLaneType(p0, lane_id), 8);            // shoulder
    EXPECT_EQ(RM_GetLaneTypeByRoadId(1, lane_id, s), 8);  // shoulder
    EXPECT_EQ(laneInfo.laneId, -4);
    EXPECT_NEAR(laneInfo.laneOffset, 0.0, 1e-3);  // offset from lane center

    RM_Close();
}

TEST(TestGetMethods, TestGetPosMethodsWithLaneOffsetOutsideRoad)
{
    ASSERT_EQ(RM_Init("../../../EnvironmentSimulator/Unittest/xodr/four_lanes_with_offset.xodr"), 0);
    ASSERT_EQ(RM_GetNumberOfRoads(), 1);

    // Create a position object
    int p0 = RM_CreatePosition();

    RM_RoadLaneInfo laneInfo;
    RM_PositionData posData;

    id_t  road_id = 1;
    int   lookahead_mode;
    float s = 9.0f;
    float t;

    // to the left of road boundary
    t              = 6.0f;
    lookahead_mode = 0;  // lookahead center current lane
    RM_SetRoadPosition(p0, road_id, s, t, true);
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    // Snaps to lane -2 (shoulder)
    EXPECT_EQ(RM_GetInLaneType(p0), 1);  // actual position outside road, type none
    EXPECT_EQ(laneInfo.laneId, -2);
    EXPECT_NEAR(laneInfo.laneOffset, 0.0, 1e-3);
    EXPECT_EQ(posData.laneId, -2);
    EXPECT_NEAR(posData.laneOffset, 4.5, 1e-3);

    // verify same result using lane pos
    RM_SetLanePosition(p0, road_id, -2, 4.5, s, true);  // t = 6 is same as lane -2 offset 4.5
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    // Snaps to lane -2 (shoulder)
    EXPECT_EQ(RM_GetInLaneType(p0), 1);
    EXPECT_EQ(laneInfo.laneId, -2);
    EXPECT_NEAR(laneInfo.laneOffset, 0.0, 1e-3);
    EXPECT_EQ(posData.laneId, -2);
    EXPECT_NEAR(posData.laneOffset, 4.5, 1e-3);

    lookahead_mode = 1;  // lookahead road center
    RM_SetRoadPosition(p0, road_id, s, t, true);
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    // Closest to lane -1 (shoulder), but belongs to 0 (none)
    EXPECT_EQ(RM_GetInLaneType(p0), 1);  // closest to lane 0, type none
    EXPECT_EQ(laneInfo.laneId, -2);      // snapped to leftmost drivable lane
    EXPECT_NEAR(laneInfo.laneOffset, -1.5, 1e-3);
    EXPECT_EQ(posData.laneId, -2);
    EXPECT_NEAR(posData.laneOffset, 4.5, 1e-3);

    t              = -6.0f;
    lookahead_mode = 0;  // lookahead center current lane
    RM_SetRoadPosition(p0, road_id, s, t, true);
    RM_GetLaneInfo(p0, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(p0, &posData);

    // Snaps to lane 4 (shoulder)
    EXPECT_EQ(RM_GetInLaneType(p0), 1);  // actual position outside road, type none
    EXPECT_EQ(laneInfo.laneId, -3);      // snapped to rightmost drivable lane
    EXPECT_NEAR(laneInfo.laneOffset, 0.0, 1e-3);
    EXPECT_EQ(posData.laneId, -3);
    EXPECT_NEAR(posData.laneOffset, -4.5, 1e-3);

    RM_Close();
}

TEST(TestGetMethods, TestGetPosMethodsVariousLookaheadModes)
{
    ASSERT_EQ(RM_Init("../../../EnvironmentSimulator/Unittest/xodr/four_lanes_with_offset.xodr"), 0);
    ASSERT_EQ(RM_GetNumberOfRoads(), 1);

    // Create a position object
    int p0 = RM_CreatePosition();

    RM_RoadLaneInfo laneInfo;
    RM_PositionData posData;

    id_t  road_id = 1;
    int   lane_id = -2;
    int   lookahead_mode;
    float s           = 5.0f;
    float lane_offset = 0.4f;

    // set a position 0.4 to the right of lane center (3.0/2 + 0.5 = 1.9 from road center)
    RM_SetLanePosition(p0, road_id, lane_id, lane_offset, s, true);

    RM_GetPositionData(p0, &posData);
    EXPECT_EQ(posData.laneId, -2);
    EXPECT_NEAR(posData.laneOffset, 0.4, 1e-3);
    EXPECT_EQ(RM_GetInLaneType(p0), 2);  // lane is drivable

    lookahead_mode = 0;  // lookahead center current lane
    RM_GetLaneInfo(p0, 3.0, &laneInfo, lookahead_mode, false);
    EXPECT_EQ(laneInfo.laneId, -2);
    EXPECT_EQ(laneInfo.s, s + 3.0f);
    EXPECT_NEAR(laneInfo.laneOffset, 0.0, 1e-3);

    lookahead_mode = 1;  // lookahead center road
    RM_GetLaneInfo(p0, 3.0, &laneInfo, lookahead_mode, false);
    EXPECT_EQ(laneInfo.laneId, -2);
    EXPECT_NEAR(laneInfo.laneOffset, -1.5, 1e-3);  // road center is at offset -1.5 from lane -2 center

    lookahead_mode = 2;  // lookahead current lane and offset
    RM_GetLaneInfo(p0, 3.0, &laneInfo, lookahead_mode, false);
    EXPECT_EQ(laneInfo.laneId, -2);
    EXPECT_NEAR(laneInfo.laneOffset, 0.4, 1e-3);  // maintain lane offset

    RM_Close();
}

static std::string
GetRoadLaneInfo_PrintLaneInfo(int pos_id, id_t road_id, int lane_id, float offset, float s, int lookahead_mode, float t = std::nan(""))
{
    RM_RoadLaneInfo laneInfo;
    RM_PositionData posData;
    char            buffer[1000];

    if (std::isnan(t))
    {
        RM_SetLanePosition(pos_id, road_id, lane_id, offset, s, true);
    }
    else
    {
        RM_SetRoadPosition(pos_id, road_id, s, t, true);
    }

    RM_GetLaneInfo(pos_id, 0.0, &laneInfo, lookahead_mode, false);
    RM_GetPositionData(pos_id, &posData);

    int l3 = RM_GetInLaneType(pos_id);
    int l4 = RM_GetLaneType(pos_id, lane_id);
    int l5 = RM_GetLaneTypeByRoadId(road_id, lane_id, s);

    if (std::isnan(t))
    {
        // lateral position given in terms of lane id and offset
        sprintf(
            buffer,
            "s %3.1f lid % d offset % 2.1f -> pos: lid % 2d offset % 2.1f x % 3.1f y % 3.1f | lookhead_mode %d -> laneInfo: t % 3.1f x % 3.1f y % 3.1f lid % 2d offset % 4.1f RM_GetInLaneType %d RM_GetLaneType %d RM_GetLaneTypeByRoadId %d\n",
            static_cast<double>(posData.s),
            posData.laneId,
            static_cast<double>(offset),
            posData.laneId,
            static_cast<double>(posData.laneOffset),
            static_cast<double>(posData.x),
            static_cast<double>(posData.y),
            lookahead_mode,
            static_cast<double>(laneInfo.t),
            static_cast<double>(laneInfo.pos.x),
            static_cast<double>(laneInfo.pos.y),
            laneInfo.laneId,
            static_cast<double>(laneInfo.laneOffset),
            l3,
            l4,
            l5);
    }
    else
    {
        // lateral position given in terms of t
        sprintf(
            buffer,
            "s %3.1f t % 3.1f -> pos: lid % 2d offset % 2.1f x % 3.1f y % 3.1f | lookhead_mode %d -> laneInfo: lid % 2d offset % 2.1f x % 3.1f y % 3.1f RM_GetInLaneType %d RM_GetLaneType %d RM_GetLaneTypeByRoadId %d\n",
            static_cast<double>(posData.s),
            static_cast<double>(t),
            posData.laneId,
            static_cast<double>(posData.laneOffset),
            static_cast<double>(posData.x),
            static_cast<double>(posData.y),
            lookahead_mode,
            laneInfo.laneId,
            static_cast<double>(laneInfo.laneOffset),
            static_cast<double>(laneInfo.pos.x),
            static_cast<double>(laneInfo.pos.y),
            l3,
            l4,
            l5);
    }

    return std::string(buffer);
}

TEST(TestGetMethods, TestGetLaneInfoExtensive)
{
    static const char* expected_std_out =
        R"(s 9.0 lid  0 offset  0.0 -> pos: lid  0 offset  0.0 x  9.0 y  5.0 | lookhead_mode 0 -> laneInfo: t  5.0 x  9.0 y  5.0 lid  0 offset  0.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 lid -1 offset  0.0 -> pos: lid -1 offset  0.0 x  9.0 y  4.0 | lookhead_mode 0 -> laneInfo: t  4.0 x  9.0 y  4.0 lid -1 offset  0.0 RM_GetInLaneType 8 RM_GetLaneType 8 RM_GetLaneTypeByRoadId 8
s 9.0 lid -2 offset  0.0 -> pos: lid -2 offset  0.0 x  9.0 y  1.5 | lookhead_mode 0 -> laneInfo: t  1.5 x  9.0 y  1.5 lid -2 offset  0.0 RM_GetInLaneType 2 RM_GetLaneType 2 RM_GetLaneTypeByRoadId 2
s 9.0 lid -3 offset  0.0 -> pos: lid -3 offset  0.0 x  9.0 y -1.5 | lookhead_mode 0 -> laneInfo: t -1.5 x  9.0 y -1.5 lid -3 offset  0.0 RM_GetInLaneType 2 RM_GetLaneType 2 RM_GetLaneTypeByRoadId 2
s 9.0 lid -4 offset  0.0 -> pos: lid -4 offset  0.0 x  9.0 y -4.0 | lookhead_mode 0 -> laneInfo: t -4.0 x  9.0 y -4.0 lid -4 offset  0.0 RM_GetInLaneType 8 RM_GetLaneType 8 RM_GetLaneTypeByRoadId 8
s 9.0 lid  0 offset  0.0 -> pos: lid  0 offset  0.0 x  9.0 y  5.0 | lookhead_mode 1 -> laneInfo: t  0.0 x  9.0 y  0.0 lid -2 offset -1.5 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 lid -1 offset  0.0 -> pos: lid -1 offset  0.0 x  9.0 y  4.0 | lookhead_mode 1 -> laneInfo: t  0.0 x  9.0 y  0.0 lid -2 offset -1.5 RM_GetInLaneType 8 RM_GetLaneType 8 RM_GetLaneTypeByRoadId 8
s 9.0 lid -2 offset  0.0 -> pos: lid -2 offset  0.0 x  9.0 y  1.5 | lookhead_mode 1 -> laneInfo: t  0.0 x  9.0 y  0.0 lid -2 offset -1.5 RM_GetInLaneType 2 RM_GetLaneType 2 RM_GetLaneTypeByRoadId 2
s 9.0 lid -3 offset  0.0 -> pos: lid -3 offset  0.0 x  9.0 y -1.5 | lookhead_mode 1 -> laneInfo: t -0.0 x  9.0 y -0.0 lid -3 offset  1.5 RM_GetInLaneType 2 RM_GetLaneType 2 RM_GetLaneTypeByRoadId 2
s 9.0 lid -4 offset  0.0 -> pos: lid -4 offset  0.0 x  9.0 y -4.0 | lookhead_mode 1 -> laneInfo: t -0.0 x  9.0 y -0.0 lid -3 offset  1.5 RM_GetInLaneType 8 RM_GetLaneType 8 RM_GetLaneTypeByRoadId 8
s 9.0 lid  0 offset  0.0 -> pos: lid  0 offset  0.0 x  9.0 y  5.0 | lookhead_mode 2 -> laneInfo: t  5.0 x  9.0 y  5.0 lid  0 offset  0.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 lid -1 offset  0.0 -> pos: lid -1 offset  0.0 x  9.0 y  4.0 | lookhead_mode 2 -> laneInfo: t  4.0 x  9.0 y  4.0 lid -1 offset  0.0 RM_GetInLaneType 8 RM_GetLaneType 8 RM_GetLaneTypeByRoadId 8
s 9.0 lid -2 offset  0.0 -> pos: lid -2 offset  0.0 x  9.0 y  1.5 | lookhead_mode 2 -> laneInfo: t  1.5 x  9.0 y  1.5 lid -2 offset  0.0 RM_GetInLaneType 2 RM_GetLaneType 2 RM_GetLaneTypeByRoadId 2
s 9.0 lid -3 offset  0.0 -> pos: lid -3 offset  0.0 x  9.0 y -1.5 | lookhead_mode 2 -> laneInfo: t -1.5 x  9.0 y -1.5 lid -3 offset  0.0 RM_GetInLaneType 2 RM_GetLaneType 2 RM_GetLaneTypeByRoadId 2
s 9.0 lid -4 offset  0.0 -> pos: lid -4 offset  0.0 x  9.0 y -4.0 | lookhead_mode 2 -> laneInfo: t -4.0 x  9.0 y -4.0 lid -4 offset  0.0 RM_GetInLaneType 8 RM_GetLaneType 8 RM_GetLaneTypeByRoadId 8
s 9.0 lid  0 offset -0.3 -> pos: lid  0 offset -0.3 x  9.0 y  4.7 | lookhead_mode 0 -> laneInfo: t  5.0 x  9.0 y  5.0 lid  0 offset  0.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 lid -1 offset -0.3 -> pos: lid -1 offset -0.3 x  9.0 y  3.7 | lookhead_mode 0 -> laneInfo: t  4.0 x  9.0 y  4.0 lid -1 offset  0.0 RM_GetInLaneType 8 RM_GetLaneType 8 RM_GetLaneTypeByRoadId 8
s 9.0 lid -2 offset -0.3 -> pos: lid -2 offset -0.3 x  9.0 y  1.2 | lookhead_mode 0 -> laneInfo: t  1.5 x  9.0 y  1.5 lid -2 offset  0.0 RM_GetInLaneType 2 RM_GetLaneType 2 RM_GetLaneTypeByRoadId 2
s 9.0 lid -3 offset -0.3 -> pos: lid -3 offset -0.3 x  9.0 y -1.8 | lookhead_mode 0 -> laneInfo: t -1.5 x  9.0 y -1.5 lid -3 offset  0.0 RM_GetInLaneType 2 RM_GetLaneType 2 RM_GetLaneTypeByRoadId 2
s 9.0 lid -4 offset -0.3 -> pos: lid -4 offset -0.3 x  9.0 y -4.3 | lookhead_mode 0 -> laneInfo: t -4.0 x  9.0 y -4.0 lid -4 offset  0.0 RM_GetInLaneType 8 RM_GetLaneType 8 RM_GetLaneTypeByRoadId 8
s 9.0 lid  0 offset -0.3 -> pos: lid  0 offset -0.3 x  9.0 y  4.7 | lookhead_mode 1 -> laneInfo: t  0.0 x  9.0 y  0.0 lid -2 offset -1.5 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 lid -1 offset -0.3 -> pos: lid -1 offset -0.3 x  9.0 y  3.7 | lookhead_mode 1 -> laneInfo: t  0.0 x  9.0 y  0.0 lid -2 offset -1.5 RM_GetInLaneType 8 RM_GetLaneType 8 RM_GetLaneTypeByRoadId 8
s 9.0 lid -2 offset -0.3 -> pos: lid -2 offset -0.3 x  9.0 y  1.2 | lookhead_mode 1 -> laneInfo: t  0.0 x  9.0 y  0.0 lid -2 offset -1.5 RM_GetInLaneType 2 RM_GetLaneType 2 RM_GetLaneTypeByRoadId 2
s 9.0 lid -3 offset -0.3 -> pos: lid -3 offset -0.3 x  9.0 y -1.8 | lookhead_mode 1 -> laneInfo: t -0.0 x  9.0 y -0.0 lid -3 offset  1.5 RM_GetInLaneType 2 RM_GetLaneType 2 RM_GetLaneTypeByRoadId 2
s 9.0 lid -4 offset -0.3 -> pos: lid -4 offset -0.3 x  9.0 y -4.3 | lookhead_mode 1 -> laneInfo: t -0.0 x  9.0 y -0.0 lid -3 offset  1.5 RM_GetInLaneType 8 RM_GetLaneType 8 RM_GetLaneTypeByRoadId 8
s 9.0 lid  0 offset -0.3 -> pos: lid  0 offset -0.3 x  9.0 y  4.7 | lookhead_mode 2 -> laneInfo: t  4.7 x  9.0 y  4.7 lid  0 offset -0.3 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 lid -1 offset -0.3 -> pos: lid -1 offset -0.3 x  9.0 y  3.7 | lookhead_mode 2 -> laneInfo: t  3.7 x  9.0 y  3.7 lid -1 offset -0.3 RM_GetInLaneType 8 RM_GetLaneType 8 RM_GetLaneTypeByRoadId 8
s 9.0 lid -2 offset -0.3 -> pos: lid -2 offset -0.3 x  9.0 y  1.2 | lookhead_mode 2 -> laneInfo: t  1.2 x  9.0 y  1.2 lid -2 offset -0.3 RM_GetInLaneType 2 RM_GetLaneType 2 RM_GetLaneTypeByRoadId 2
s 9.0 lid -3 offset -0.3 -> pos: lid -3 offset -0.3 x  9.0 y -1.8 | lookhead_mode 2 -> laneInfo: t -1.8 x  9.0 y -1.8 lid -3 offset -0.3 RM_GetInLaneType 2 RM_GetLaneType 2 RM_GetLaneTypeByRoadId 2
s 9.0 lid -4 offset -0.3 -> pos: lid -4 offset -0.3 x  9.0 y -4.3 | lookhead_mode 2 -> laneInfo: t -4.3 x  9.0 y -4.3 lid -4 offset -0.3 RM_GetInLaneType 8 RM_GetLaneType 8 RM_GetLaneTypeByRoadId 8
)"
        R"(s 9.0 t  6.0 -> pos: lid -2 offset  4.5 x  9.0 y  6.0 | lookhead_mode 0 -> laneInfo: lid -2 offset  0.0 x  9.0 y  1.5 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  5.0 -> pos: lid -2 offset  3.5 x  9.0 y  5.0 | lookhead_mode 0 -> laneInfo: lid -2 offset  0.0 x  9.0 y  1.5 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  4.0 -> pos: lid -2 offset  2.5 x  9.0 y  4.0 | lookhead_mode 0 -> laneInfo: lid -2 offset  0.0 x  9.0 y  1.5 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  3.0 -> pos: lid -2 offset  1.5 x  9.0 y  3.0 | lookhead_mode 0 -> laneInfo: lid -2 offset  0.0 x  9.0 y  1.5 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  2.0 -> pos: lid -2 offset  0.5 x  9.0 y  2.0 | lookhead_mode 0 -> laneInfo: lid -2 offset  0.0 x  9.0 y  1.5 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  1.0 -> pos: lid -2 offset -0.5 x  9.0 y  1.0 | lookhead_mode 0 -> laneInfo: lid -2 offset  0.0 x  9.0 y  1.5 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  0.0 -> pos: lid -2 offset -1.5 x  9.0 y  0.0 | lookhead_mode 0 -> laneInfo: lid -2 offset  0.0 x  9.0 y  1.5 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -1.0 -> pos: lid -3 offset  0.5 x  9.0 y -1.0 | lookhead_mode 0 -> laneInfo: lid -3 offset  0.0 x  9.0 y -1.5 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -2.0 -> pos: lid -3 offset -0.5 x  9.0 y -2.0 | lookhead_mode 0 -> laneInfo: lid -3 offset  0.0 x  9.0 y -1.5 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -3.0 -> pos: lid -3 offset -1.5 x  9.0 y -3.0 | lookhead_mode 0 -> laneInfo: lid -3 offset  0.0 x  9.0 y -1.5 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -4.0 -> pos: lid -3 offset -2.5 x  9.0 y -4.0 | lookhead_mode 0 -> laneInfo: lid -3 offset  0.0 x  9.0 y -1.5 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -5.0 -> pos: lid -3 offset -3.5 x  9.0 y -5.0 | lookhead_mode 0 -> laneInfo: lid -3 offset  0.0 x  9.0 y -1.5 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -6.0 -> pos: lid -3 offset -4.5 x  9.0 y -6.0 | lookhead_mode 0 -> laneInfo: lid -3 offset  0.0 x  9.0 y -1.5 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  6.0 -> pos: lid -2 offset  4.5 x  9.0 y  6.0 | lookhead_mode 1 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  5.0 -> pos: lid -2 offset  3.5 x  9.0 y  5.0 | lookhead_mode 1 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  4.0 -> pos: lid -2 offset  2.5 x  9.0 y  4.0 | lookhead_mode 1 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  3.0 -> pos: lid -2 offset  1.5 x  9.0 y  3.0 | lookhead_mode 1 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  2.0 -> pos: lid -2 offset  0.5 x  9.0 y  2.0 | lookhead_mode 1 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  1.0 -> pos: lid -2 offset -0.5 x  9.0 y  1.0 | lookhead_mode 1 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  0.0 -> pos: lid -2 offset -1.5 x  9.0 y  0.0 | lookhead_mode 1 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -1.0 -> pos: lid -3 offset  0.5 x  9.0 y -1.0 | lookhead_mode 1 -> laneInfo: lid -3 offset  1.5 x  9.0 y -0.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -2.0 -> pos: lid -3 offset -0.5 x  9.0 y -2.0 | lookhead_mode 1 -> laneInfo: lid -3 offset  1.5 x  9.0 y -0.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -3.0 -> pos: lid -3 offset -1.5 x  9.0 y -3.0 | lookhead_mode 1 -> laneInfo: lid -3 offset  1.5 x  9.0 y -0.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -4.0 -> pos: lid -3 offset -2.5 x  9.0 y -4.0 | lookhead_mode 1 -> laneInfo: lid -3 offset  1.5 x  9.0 y -0.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -5.0 -> pos: lid -3 offset -3.5 x  9.0 y -5.0 | lookhead_mode 1 -> laneInfo: lid -3 offset  1.5 x  9.0 y -0.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -6.0 -> pos: lid -3 offset -4.5 x  9.0 y -6.0 | lookhead_mode 1 -> laneInfo: lid -3 offset  1.5 x  9.0 y -0.0 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  6.0 -> pos: lid -2 offset  4.5 x  9.0 y  6.0 | lookhead_mode 2 -> laneInfo: lid -2 offset  4.5 x  9.0 y  6.0 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  5.0 -> pos: lid -2 offset  3.5 x  9.0 y  5.0 | lookhead_mode 2 -> laneInfo: lid -2 offset  3.5 x  9.0 y  5.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  4.0 -> pos: lid -2 offset  2.5 x  9.0 y  4.0 | lookhead_mode 2 -> laneInfo: lid -2 offset  2.5 x  9.0 y  4.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  3.0 -> pos: lid -2 offset  1.5 x  9.0 y  3.0 | lookhead_mode 2 -> laneInfo: lid -2 offset  1.5 x  9.0 y  3.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  2.0 -> pos: lid -2 offset  0.5 x  9.0 y  2.0 | lookhead_mode 2 -> laneInfo: lid -2 offset  0.5 x  9.0 y  2.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  1.0 -> pos: lid -2 offset -0.5 x  9.0 y  1.0 | lookhead_mode 2 -> laneInfo: lid -2 offset -0.5 x  9.0 y  1.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  0.0 -> pos: lid -2 offset -1.5 x  9.0 y  0.0 | lookhead_mode 2 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -1.0 -> pos: lid -3 offset  0.5 x  9.0 y -1.0 | lookhead_mode 2 -> laneInfo: lid -3 offset  0.5 x  9.0 y -1.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -2.0 -> pos: lid -3 offset -0.5 x  9.0 y -2.0 | lookhead_mode 2 -> laneInfo: lid -3 offset -0.5 x  9.0 y -2.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -3.0 -> pos: lid -3 offset -1.5 x  9.0 y -3.0 | lookhead_mode 2 -> laneInfo: lid -3 offset -1.5 x  9.0 y -3.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -4.0 -> pos: lid -3 offset -2.5 x  9.0 y -4.0 | lookhead_mode 2 -> laneInfo: lid -3 offset -2.5 x  9.0 y -4.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -5.0 -> pos: lid -3 offset -3.5 x  9.0 y -5.0 | lookhead_mode 2 -> laneInfo: lid -3 offset -3.5 x  9.0 y -5.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -6.0 -> pos: lid -3 offset -4.5 x  9.0 y -6.0 | lookhead_mode 2 -> laneInfo: lid -3 offset -4.5 x  9.0 y -6.0 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
)"
        R"(s 9.0 t  5.7 -> pos: lid -2 offset  4.2 x  9.0 y  5.7 | lookhead_mode 0 -> laneInfo: lid -2 offset  0.0 x  9.0 y  1.5 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  4.7 -> pos: lid -2 offset  3.2 x  9.0 y  4.7 | lookhead_mode 0 -> laneInfo: lid -2 offset  0.0 x  9.0 y  1.5 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  3.7 -> pos: lid -2 offset  2.2 x  9.0 y  3.7 | lookhead_mode 0 -> laneInfo: lid -2 offset  0.0 x  9.0 y  1.5 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  2.7 -> pos: lid -2 offset  1.2 x  9.0 y  2.7 | lookhead_mode 0 -> laneInfo: lid -2 offset  0.0 x  9.0 y  1.5 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  1.7 -> pos: lid -2 offset  0.2 x  9.0 y  1.7 | lookhead_mode 0 -> laneInfo: lid -2 offset  0.0 x  9.0 y  1.5 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  0.7 -> pos: lid -2 offset -0.8 x  9.0 y  0.7 | lookhead_mode 0 -> laneInfo: lid -2 offset  0.0 x  9.0 y  1.5 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -0.3 -> pos: lid -3 offset  1.2 x  9.0 y -0.3 | lookhead_mode 0 -> laneInfo: lid -3 offset  0.0 x  9.0 y -1.5 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -1.3 -> pos: lid -3 offset  0.2 x  9.0 y -1.3 | lookhead_mode 0 -> laneInfo: lid -3 offset  0.0 x  9.0 y -1.5 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -2.3 -> pos: lid -3 offset -0.8 x  9.0 y -2.3 | lookhead_mode 0 -> laneInfo: lid -3 offset  0.0 x  9.0 y -1.5 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -3.3 -> pos: lid -3 offset -1.8 x  9.0 y -3.3 | lookhead_mode 0 -> laneInfo: lid -3 offset  0.0 x  9.0 y -1.5 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -4.3 -> pos: lid -3 offset -2.8 x  9.0 y -4.3 | lookhead_mode 0 -> laneInfo: lid -3 offset  0.0 x  9.0 y -1.5 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -5.3 -> pos: lid -3 offset -3.8 x  9.0 y -5.3 | lookhead_mode 0 -> laneInfo: lid -3 offset  0.0 x  9.0 y -1.5 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -6.3 -> pos: lid -3 offset -4.8 x  9.0 y -6.3 | lookhead_mode 0 -> laneInfo: lid -3 offset  0.0 x  9.0 y -1.5 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  5.7 -> pos: lid -2 offset  4.2 x  9.0 y  5.7 | lookhead_mode 1 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  4.7 -> pos: lid -2 offset  3.2 x  9.0 y  4.7 | lookhead_mode 1 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  3.7 -> pos: lid -2 offset  2.2 x  9.0 y  3.7 | lookhead_mode 1 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  2.7 -> pos: lid -2 offset  1.2 x  9.0 y  2.7 | lookhead_mode 1 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  1.7 -> pos: lid -2 offset  0.2 x  9.0 y  1.7 | lookhead_mode 1 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  0.7 -> pos: lid -2 offset -0.8 x  9.0 y  0.7 | lookhead_mode 1 -> laneInfo: lid -2 offset -1.5 x  9.0 y  0.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -0.3 -> pos: lid -3 offset  1.2 x  9.0 y -0.3 | lookhead_mode 1 -> laneInfo: lid -3 offset  1.5 x  9.0 y -0.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -1.3 -> pos: lid -3 offset  0.2 x  9.0 y -1.3 | lookhead_mode 1 -> laneInfo: lid -3 offset  1.5 x  9.0 y -0.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -2.3 -> pos: lid -3 offset -0.8 x  9.0 y -2.3 | lookhead_mode 1 -> laneInfo: lid -3 offset  1.5 x  9.0 y -0.0 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -3.3 -> pos: lid -3 offset -1.8 x  9.0 y -3.3 | lookhead_mode 1 -> laneInfo: lid -3 offset  1.5 x  9.0 y -0.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -4.3 -> pos: lid -3 offset -2.8 x  9.0 y -4.3 | lookhead_mode 1 -> laneInfo: lid -3 offset  1.5 x  9.0 y -0.0 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -5.3 -> pos: lid -3 offset -3.8 x  9.0 y -5.3 | lookhead_mode 1 -> laneInfo: lid -3 offset  1.5 x  9.0 y -0.0 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -6.3 -> pos: lid -3 offset -4.8 x  9.0 y -6.3 | lookhead_mode 1 -> laneInfo: lid -3 offset  1.5 x  9.0 y -0.0 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  5.7 -> pos: lid -2 offset  4.2 x  9.0 y  5.7 | lookhead_mode 2 -> laneInfo: lid -2 offset  4.2 x  9.0 y  5.7 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  4.7 -> pos: lid -2 offset  3.2 x  9.0 y  4.7 | lookhead_mode 2 -> laneInfo: lid -2 offset  3.2 x  9.0 y  4.7 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  3.7 -> pos: lid -2 offset  2.2 x  9.0 y  3.7 | lookhead_mode 2 -> laneInfo: lid -2 offset  2.2 x  9.0 y  3.7 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  2.7 -> pos: lid -2 offset  1.2 x  9.0 y  2.7 | lookhead_mode 2 -> laneInfo: lid -2 offset  1.2 x  9.0 y  2.7 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  1.7 -> pos: lid -2 offset  0.2 x  9.0 y  1.7 | lookhead_mode 2 -> laneInfo: lid -2 offset  0.2 x  9.0 y  1.7 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t  0.7 -> pos: lid -2 offset -0.8 x  9.0 y  0.7 | lookhead_mode 2 -> laneInfo: lid -2 offset -0.8 x  9.0 y  0.7 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -0.3 -> pos: lid -3 offset  1.2 x  9.0 y -0.3 | lookhead_mode 2 -> laneInfo: lid -3 offset  1.2 x  9.0 y -0.3 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -1.3 -> pos: lid -3 offset  0.2 x  9.0 y -1.3 | lookhead_mode 2 -> laneInfo: lid -3 offset  0.2 x  9.0 y -1.3 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -2.3 -> pos: lid -3 offset -0.8 x  9.0 y -2.3 | lookhead_mode 2 -> laneInfo: lid -3 offset -0.8 x  9.0 y -2.3 RM_GetInLaneType 2 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -3.3 -> pos: lid -3 offset -1.8 x  9.0 y -3.3 | lookhead_mode 2 -> laneInfo: lid -3 offset -1.8 x  9.0 y -3.3 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -4.3 -> pos: lid -3 offset -2.8 x  9.0 y -4.3 | lookhead_mode 2 -> laneInfo: lid -3 offset -2.8 x  9.0 y -4.3 RM_GetInLaneType 8 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -5.3 -> pos: lid -3 offset -3.8 x  9.0 y -5.3 | lookhead_mode 2 -> laneInfo: lid -3 offset -3.8 x  9.0 y -5.3 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
s 9.0 t -6.3 -> pos: lid -3 offset -4.8 x  9.0 y -6.3 | lookhead_mode 2 -> laneInfo: lid -3 offset -4.8 x  9.0 y -6.3 RM_GetInLaneType 1 RM_GetLaneType 1 RM_GetLaneTypeByRoadId 1
)";

    ASSERT_EQ(RM_Init("../../../EnvironmentSimulator/Unittest/xodr/four_lanes_with_offset.xodr"), 0);

    std::string log;

    int p0 = RM_CreatePosition();

    std::vector<float> offset = {0.0f, -0.3f};
    for (auto& o : offset)
    {
        for (int lookahead_mode = 0; lookahead_mode < 3; lookahead_mode++)
        {
            for (int i = 0; i > -5; i--)
            {
                log.append(GetRoadLaneInfo_PrintLaneInfo(p0, 1, i, o, 9, lookahead_mode));
            }
        }
    }

    for (auto& o : offset)
    {
        for (int lookahead_mode = 0; lookahead_mode < 3; lookahead_mode++)
        {
            for (float t = 6.0f; t > -6.1f; t -= 1.0f)
            {
                log.append(GetRoadLaneInfo_PrintLaneInfo(p0, 1, 0, std::nanf(""), 9, lookahead_mode, t + o));
            }
        }
    }

    RM_Close();

    EXPECT_EQ(expected_std_out, log);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    if (argc > 1)
    {
        if (!strcmp(argv[1], "--disable_stdout"))
        {
            //  disable logging to stdout from the esminiRMLib
            RM_SetOptionPersistent("disable_stdout");

            // disable logging to stdout from the test cases
            SE_Env::Inst().GetOptions().SetOptionValue("disable_stdout", "", false, true);
        }
        else
        {
            printf("Usage: %s [--disable_stout] [google test options...]\n", argv[0]);
            return -1;
        }
    }

    // testing::GTEST_FLAG(filter) = "*check_GroundTruth_including_init_state*";

    return RUN_ALL_TESTS();
}
