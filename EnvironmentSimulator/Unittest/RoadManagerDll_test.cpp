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

// Uncomment to print log output to console
// #define LOG_TO_CONSOLE

#ifdef LOG_TO_CONSOLE
static void log_callback(const char* str)
{
    printf("%s\n", str);
}
#endif

int main(int argc, char** argv)
{
#ifdef LOG_TO_CONSOLE
    if (!(Logger::Inst().IsCallbackSet()))
    {
        Logger::Inst().SetCallback(log_callback);
    }
#endif

    // testing::GTEST_FLAG(filter) = "*check_GroundTruth_including_init_state*";
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}