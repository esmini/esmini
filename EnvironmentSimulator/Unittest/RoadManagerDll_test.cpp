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