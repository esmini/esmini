#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <chrono>

#include "pugixml.hpp"
#include "simple_expr.h"
#include "LaneIndependentRouter.hpp"

#define TRIG_ERR_MARGIN 0.001

using namespace roadmanager;
OpenDrive *odrSmall = nullptr;
OpenDrive *odrMedium = nullptr;
OpenDrive *odrLarge = nullptr;
class FollowRouteTest : public ::testing::Test
{
public:
    static void SetUpTestSuite()
    {
        //Position::GetOpenDrive()->LoadOpenDriveFile("../../../../esmini/resources/xodr/highway_example_with_merge_and_split.xodr");
        odrSmall = new OpenDrive("../../../../esmini/resources/xodr/highway_example_with_merge_and_split.xodr");

        //Position::GetOpenDrive()->LoadOpenDriveFile("../../../../esmini/resources/xodr/multi_intersections.xodr");
        odrMedium = new OpenDrive("../../../../esmini/resources/xodr/multi_intersections.xodr");

        //Position::GetOpenDrive()->LoadOpenDriveFile("../../../../large.xodr");
        odrLarge = new OpenDrive("../../../../large.xodr");
    }
    static OpenDrive *odrSmall;
    static OpenDrive *odrMedium;
    static OpenDrive *odrLarge;
};

OpenDrive* FollowRouteTest::odrSmall = nullptr;
OpenDrive* FollowRouteTest::odrMedium = nullptr;
OpenDrive* FollowRouteTest::odrLarge = nullptr;

static void log_callback(const char *str);

TEST_F(FollowRouteTest, FindPathSmall1)
{
    Position::LoadOpenDrive(odrSmall);
    ASSERT_NE(odrSmall, nullptr);

    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(0, -1, 10, 0);
    start.SetHeadingRelativeRoadDirection(0);

    Position target(5, -2, 20, 0);

    LaneIndependentRouter router(odrSmall);
    std::vector<Node *> path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 5);
}

TEST_F(FollowRouteTest, FindPathSmall2)
{
    Position::LoadOpenDrive(odrSmall);
    ASSERT_NE(odrSmall, nullptr);

    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(0, -1, 10, 0);
    start.SetHeadingRelativeRoadDirection(0);

    Position target(2, -1, 20, 0);

    LaneIndependentRouter router(odrSmall);
    std::vector<Node *> path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 2);
}
TEST_F(FollowRouteTest, FindPathMedium1)
{
    Position::LoadOpenDrive(odrMedium);
    ASSERT_NE(odrMedium, nullptr);

    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(202, 2, 100, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);

    Position target(209, 1, 20, 0);

    LaneIndependentRouter router(odrMedium);
    std::vector<Node *> path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 209);
}

TEST_F(FollowRouteTest, FindPathMedium2)
{
    Position::LoadOpenDrive(odrMedium);
    ASSERT_NE(odrMedium, nullptr);

    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(217, -1, 50, 0);
    start.SetHeadingRelativeRoadDirection(0);

    Position target(275, -1, 50, 0);

    LaneIndependentRouter router(odrMedium);
    std::vector<Node *> path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 275);
}

TEST_F(FollowRouteTest, FindPathLarge1)
{
    Position::LoadOpenDrive(odrLarge);
    ASSERT_NE(odrLarge, nullptr);

    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(5147, -1, 50, 0);
    start.SetHeadingRelativeRoadDirection(0);

    Position target(2206, 1, 100, 0);

    LaneIndependentRouter router(odrLarge);
    std::vector<Node *> path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 2206);
}

TEST_F(FollowRouteTest, FindPathLarge2)
{
    Position::LoadOpenDrive(odrLarge);
    ASSERT_NE(odrLarge, nullptr);

    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(1006, 3, 1900, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);

    Position target(2203, 1, 20, 0);

    LaneIndependentRouter router(odrLarge);
    std::vector<Node *> path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 2203);
}

TEST_F(FollowRouteTest, FindPathLarge3)
{
    Position::LoadOpenDrive(odrLarge);
    ASSERT_NE(odrLarge, nullptr);

    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(8277, 1, 200, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);

    Position target(2291, -2, 50, 0);

    LaneIndependentRouter router(odrLarge);
    std::vector<Node *> path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 2291);
}

TEST_F(FollowRouteTest, FindPathLarge4)
{
    Position::LoadOpenDrive(odrLarge);
    ASSERT_NE(odrLarge, nullptr);

    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(2431, 1, 2200, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);

    Position target(2503, -1, 3700, 0);

    LaneIndependentRouter router(odrLarge);
    std::vector<Node *> path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 2503);
}

TEST_F(FollowRouteTest, FindPathShortest)
{
    Position::LoadOpenDrive(odrMedium);
    ASSERT_NE(odrMedium, nullptr);

    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(202, 2, 100, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);

    Position target(209, 1, 20, 0);

    LaneIndependentRouter router(odrMedium);
    std::vector<Node *> path = router.CalculatePath(start, target, RouteStrategy::SHORTEST);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 209);
}

TEST_F(FollowRouteTest, FindPathFastest)
{
    Position::LoadOpenDrive(odrMedium);
    ASSERT_NE(odrMedium, nullptr);

    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(202, 2, 100, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);

    Position target(209, 1, 20, 0);

    LaneIndependentRouter router(odrMedium);
    std::vector<Node *> path = router.CalculatePath(start, target, RouteStrategy::FASTEST);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 209);
}

TEST_F(FollowRouteTest, FindPathMinIntersections)
{
    Position::LoadOpenDrive(odrMedium);
    ASSERT_NE(odrMedium, nullptr);

    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(202, 2, 100, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);

    Position target(209, 1, 20, 0);

    LaneIndependentRouter router(odrMedium);
    std::vector<Node *> path = router.CalculatePath(start, target, RouteStrategy::MIN_INTERSECTIONS);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 209);
}

TEST_F(FollowRouteTest, FindPathTimeMedium)
{
    Position::LoadOpenDrive(odrMedium);
    ASSERT_NE(odrMedium, nullptr);

    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(202, 2, 100, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);

    Position target(209, 1, 20, 0);

    LaneIndependentRouter router(odrMedium);
    auto startTime = std::chrono::high_resolution_clock::now();
    std::vector<Node *> path = router.CalculatePath(start, target, RouteStrategy::SHORTEST);
    auto endTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 209);

    int maxMicroseconds = 5000;
    ASSERT_LT((int)elapsedTime, maxMicroseconds);
}

TEST_F(FollowRouteTest, FindPathTimeLarge)
{
    Position::LoadOpenDrive(odrLarge);
    ASSERT_NE(odrLarge, nullptr);

    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(2291, -1, 1100, 0);
    start.SetHeadingRelativeRoadDirection(0);

    Position target(2502, 1, 50, 0);

    LaneIndependentRouter router(odrLarge);
    auto startTime = std::chrono::high_resolution_clock::now();
    std::vector<Node *> path = router.CalculatePath(start, target, RouteStrategy::SHORTEST);
    auto endTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 2502);

    int maxMicroseconds = 15000;
    ASSERT_LT((int)elapsedTime, maxMicroseconds);
}

TEST_F(FollowRouteTest, CreateWaypointSmall)
{
    Position::LoadOpenDrive(odrSmall);
    ASSERT_NE(odrSmall, nullptr);
    std::vector<Position> expectedWaypoints = {Position(0,-4,125,0),Position(4,-1,50,0),Position(2,-1,20,0)};
    // Set start pos and the driving direction (heading) 
    // PI = against road dir,   0 = road dir
    Position start(0, -3, 10, 0);
    start.SetHeadingRelativeRoadDirection(0);
    Position target(2, -1, 20, 0);
    LaneIndependentRouter router(odrSmall);
    std::vector<Node *> path = router.CalculatePath(start, target);
    ASSERT_FALSE(path.empty());
    std::vector<Position> calcWaypoints = router.GetWaypoints(path,target);
    ASSERT_FALSE(calcWaypoints.empty());
    ASSERT_EQ(calcWaypoints.size(),expectedWaypoints.size());
    for(int i = 0; i < expectedWaypoints.size();i++){
        ASSERT_EQ(calcWaypoints[i].GetTrackId(),expectedWaypoints[i].GetTrackId());
        ASSERT_EQ(calcWaypoints[i].GetLaneId(),expectedWaypoints[i].GetLaneId());
        ASSERT_EQ(calcWaypoints[i].GetS(),expectedWaypoints[i].GetS());
        ASSERT_EQ(calcWaypoints[i].GetOffset(),expectedWaypoints[i].GetOffset());
    }
}


// Uncomment to print log output to console
#define LOG_TO_CONSOLE

#ifdef LOG_TO_CONSOLE
static void log_callback(const char *str)
{
    printf("%s\n", str);
}
#endif

int main(int argc, char **argv)
{
#ifdef LOG_TO_CONSOLE
    if (!(Logger::Inst().IsCallbackSet()))
    {
        Logger::Inst().SetCallback(log_callback);
    }
#endif

    // testing::GTEST_FLAG(filter) = "*TestOptionHandling*";

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}