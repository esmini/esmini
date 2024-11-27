#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <chrono>

#include "pugixml.hpp"
#include "simple_expr.h"
#include "LaneIndependentRouter.hpp"

// Enable testing of the mega road network
// #define ENABLE_LARGE_ROAD_NETWORK

#define TRIG_ERR_MARGIN 0.001

using namespace roadmanager;

class FollowRouteTestSmall : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/highway_example_with_merge_and_split.xodr");
    }
};

class FollowRouteTestMedium : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        SE_Env::Inst().AddPath("../../../resources/traffic_signals");
        Position::LoadOpenDrive("../../../resources/xodr/multi_intersections.xodr");
    }
};

class FollowRouteTestMediumChangedSpeeds : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        SE_Env::Inst().AddPath("../../../resources/traffic_signals");
        Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/multi_intersections_changed_speeds.xodr");
    }
};

class FollowRouteTestLarge : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        SE_Env::Inst().AddPath("../../../resources/traffic_signals");
        Position::LoadOpenDrive("../../../resources/xodr/large.xodr");
    }
};

class FollowRouteTestRouteTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/route_strategy_test_road.xodr");
    }
};

class FollowRouteTestRouteTestLHT : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        Position::LoadOpenDrive("../../../EnvironmentSimulator/Unittest/xodr/route_strategy_test_road_LHT.xodr");
    }
};

TEST_F(FollowRouteTestSmall, FindPathSmallInvalidPosition)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(),
              "../../../EnvironmentSimulator/Unittest/xodr/highway_example_with_merge_and_split.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(0, -1, 10, 0);
    start.SetHeadingRelativeRoadDirection(0);
    Position target(5, -4, 20, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());

    std::vector<Node> path = router.CalculatePath(start, target);
    // Check invalid target
    ASSERT_TRUE(path.empty());

    start  = Position(ID_UNDEFINED, -1, 10, 0);
    target = Position(5, -2, 20, 0);

    path = router.CalculatePath(start, target);
    // Check invalid start
    ASSERT_TRUE(path.empty());
}

TEST_F(FollowRouteTestSmall, FindPathSmall1)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(),
              "../../../EnvironmentSimulator/Unittest/xodr/highway_example_with_merge_and_split.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(0, -1, 10, 0);
    start.SetHeadingRelativeRoadDirection(0);

    Position target(5, -2, 20, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back().road->GetId(), 5);
}

TEST_F(FollowRouteTestSmall, FindPathSmall2)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(),
              "../../../EnvironmentSimulator/Unittest/xodr/highway_example_with_merge_and_split.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(0, -1, 10, 0);
    start.SetHeadingRelativeRoadDirection(0);

    Position target(2, -1, 20, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back().road->GetId(), 2);
}

TEST_F(FollowRouteTestSmall, CreateWaypointSmall1)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(),
              "../../../EnvironmentSimulator/Unittest/xodr/highway_example_with_merge_and_split.xodr");

    std::vector<Position> expectedWaypoints = {Position(0, -4, 125, 0), Position(4, -1, 50, 0), Position(2, -1, 20, 0)};
    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(0, -3, 10, 0);
    start.SetHeadingRelativeRoadDirection(0);
    Position target(2, -1, 20, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);
    ASSERT_FALSE(path.empty());

    std::vector<Position> calcWaypoints = router.GetWaypoints(path, start, target);
    ASSERT_FALSE(calcWaypoints.empty());
    ASSERT_EQ(calcWaypoints.size(), expectedWaypoints.size());
    for (size_t i = 0; i < expectedWaypoints.size(); i++)
    {
        ASSERT_EQ(calcWaypoints[i].GetTrackId(), expectedWaypoints[i].GetTrackId());
        ASSERT_EQ(calcWaypoints[i].GetLaneId(), expectedWaypoints[i].GetLaneId());
        ASSERT_NEAR(calcWaypoints[i].GetS(), expectedWaypoints[i].GetS(), 0.1);
        ASSERT_NEAR(calcWaypoints[i].GetOffset(), expectedWaypoints[i].GetOffset(), 0.1);
    }
}

TEST_F(FollowRouteTestSmall, CreateWaypointSmall2)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(),
              "../../../EnvironmentSimulator/Unittest/xodr/highway_example_with_merge_and_split.xodr");

    std::vector<Position> expectedWaypoints = {Position(0, -3, 10, 0),
                                               Position(3, -3, 25, 0),
                                               Position(1, -3, 57.5, 0),
                                               Position(6, -3, 15, 0),
                                               Position(5, -3, 20, 0)};

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(0, -3, 10, 0);
    start.SetHeadingRelativeRoadDirection(0);
    Position target(5, -3, 20, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);
    ASSERT_FALSE(path.empty());

    std::vector<Position> calcWaypoints = router.GetWaypoints(path, start, target);
    ASSERT_FALSE(calcWaypoints.empty());
    ASSERT_EQ(calcWaypoints.size(), expectedWaypoints.size());
    for (size_t i = 0; i < expectedWaypoints.size(); i++)
    {
        ASSERT_EQ(calcWaypoints[i].GetTrackId(), expectedWaypoints[i].GetTrackId());
        ASSERT_EQ(calcWaypoints[i].GetLaneId(), expectedWaypoints[i].GetLaneId());
        ASSERT_NEAR(calcWaypoints[i].GetS(), expectedWaypoints[i].GetS(), 0.1);
        ASSERT_NEAR(calcWaypoints[i].GetOffset(), expectedWaypoints[i].GetOffset(), 0.1);
    }
}

TEST_F(FollowRouteTestMedium, FindPathMedium1)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../resources/xodr/multi_intersections.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(202, 2, 100, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);

    Position target(209, 1, 20, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back().road->GetId(), 209);
}

TEST_F(FollowRouteTestMedium, FindPathMedium2)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../resources/xodr/multi_intersections.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(217, -1, 50, 0);
    start.SetHeadingRelativeRoadDirection(0);

    Position target(275, -1, 50, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back().road->GetId(), 275);
}

TEST_F(FollowRouteTestMedium, FindPathTimeMedium)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../resources/xodr/multi_intersections.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(202, 2, 100, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);

    Position target(209, 1, 20, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back().road->GetId(), 209);
}

TEST_F(FollowRouteTestMedium, CreateWaypointMedium)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../resources/xodr/multi_intersections.xodr");

    std::vector<Position> expectedWaypoints = {Position(266, 1, 50, 0),
                                               Position(258, -1, 8.85, 0),
                                               Position(261, -1, 54.5, 0),
                                               Position(196, 1, 54.5, 0),
                                               Position(204, -1, 11.5, 0),
                                               Position(197, -1, 54, 0),
                                               Position(275, 1, 55, 0)};

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(266, 1, 50, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);
    Position target(275, 1, 55, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);
    ASSERT_FALSE(path.empty());

    std::vector<Position> calcWaypoints = router.GetWaypoints(path, start, target);
    ASSERT_FALSE(calcWaypoints.empty());
    ASSERT_EQ(calcWaypoints.size(), expectedWaypoints.size());
    for (size_t i = 0; i < expectedWaypoints.size(); i++)
    {
        ASSERT_EQ(calcWaypoints[i].GetTrackId(), expectedWaypoints[i].GetTrackId());
        ASSERT_EQ(calcWaypoints[i].GetLaneId(), expectedWaypoints[i].GetLaneId());
        ASSERT_NEAR(calcWaypoints[i].GetS(), expectedWaypoints[i].GetS(), 0.5);
        ASSERT_NEAR(calcWaypoints[i].GetOffset(), expectedWaypoints[i].GetOffset(), 0.5);
    }
}

#ifdef ENABLE_LARGE_ROAD_NETWORK

TEST_F(FollowRouteTestLarge, FindPathLarge1)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../resources/xodr/large.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(5147, -1, 50, 0);
    start.SetHeadingRelativeRoadDirection(0);

    Position target(2206, 1, 100, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back().road->GetId(), 2206);
}

TEST_F(FollowRouteTestLarge, FindPathLarge2)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../resources/xodr/large.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(1006, 3, 1900, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);

    Position target(2203, 1, 20, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back().road->GetId(), 2203);
}

TEST_F(FollowRouteTestLarge, FindPathLarge3)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../resources/xodr/large.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(8277, 1, 200, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);

    Position target(2291, -2, 50, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back().road->GetId(), 2291);
}

TEST_F(FollowRouteTestLarge, FindPathLarge4)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../resources/xodr/large.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(2431, 1, 2200, 0);
    start.SetHeadingRelativeRoadDirection(M_PI);

    Position target(2503, -1, 3700, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back().road->GetId(), 2503);
}

TEST_F(FollowRouteTestLarge, FindPathTimeLarge)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../resources/xodr/large.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(2291, -1, 1100, 0);
    start.SetHeadingRelativeRoadDirection(0);

    Position target(2502, 1, 50, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back().road->GetId(), 2502);
}

TEST_F(FollowRouteTestLarge, CreateWaypointLarge)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../resources/xodr/large.xodr");

    std::vector<Position> expectedWaypoints = {Position(5219, -2, 226.8, 0),
                                               Position(8264, -2, 11, 0),
                                               Position(2357, -1, 150.6, 0),
                                               Position(2505, -1, 8.7, 0),
                                               Position(2704, -1, 21.8, 0),
                                               Position(2701, -1, 8.7, 0),
                                               Position(2512, -1, 765.5, 0),
                                               Position(2203, -1, 17.5, 0)};

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(5219, -1, 116, 0);
    start.SetHeadingRelativeRoadDirection(0);
    Position target(2203, -1, 17.5, 0);

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);
    ASSERT_FALSE(path.empty());

    std::vector<Position> calcWaypoints = router.GetWaypoints(path, start, target);

    ASSERT_FALSE(calcWaypoints.empty());
    ASSERT_EQ(calcWaypoints.size(), expectedWaypoints.size());
    for (int i = 0; i < expectedWaypoints.size(); i++)
    {
        ASSERT_EQ(calcWaypoints[i].GetTrackId(), expectedWaypoints[i].GetTrackId());
        ASSERT_EQ(calcWaypoints[i].GetLaneId(), expectedWaypoints[i].GetLaneId());
        ASSERT_NEAR(calcWaypoints[i].GetS(), expectedWaypoints[i].GetS(), 0.5);
        ASSERT_NEAR(calcWaypoints[i].GetOffset(), expectedWaypoints[i].GetOffset(), 0.5);
    }
}
#endif  // Large

TEST_F(FollowRouteTestRouteTest, FindPathShortest)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../EnvironmentSimulator/Unittest/xodr/route_strategy_test_road.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(1, -1, 100, 0);
    start.SetHeadingRelativeRoadDirection(0);
    Position target(7, -2, 10, 0);
    target.SetRouteStrategy(Position::RouteStrategy::SHORTEST);

    std::vector<int> expectedRoadIds = {1, 100, 2, 200, 3, 301, 6, 400, 7};

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);
    ASSERT_FALSE(path.empty());

    for (size_t i = 0; i < expectedRoadIds.size(); i++)
    {
        ASSERT_EQ(path[i].road->GetId(), expectedRoadIds[i]);
    }
}

TEST_F(FollowRouteTestRouteTest, FindPathFastest)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../EnvironmentSimulator/Unittest/xodr/route_strategy_test_road.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(1, -1, 100, 0);
    start.SetHeadingRelativeRoadDirection(0);
    Position target(7, -2, 10, 0);
    target.SetRouteStrategy(Position::RouteStrategy::FASTEST);

    std::vector<int> expectedRoadIds = {1, 100, 2, 201, 4, 302, 6, 400, 7};

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);
    ASSERT_FALSE(path.empty());

    for (size_t i = 0; i < expectedRoadIds.size(); i++)
    {
        ASSERT_EQ(path[i].road->GetId(), expectedRoadIds[i]);
    }
}

TEST_F(FollowRouteTestRouteTest, FindPathMinIntersections)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../EnvironmentSimulator/Unittest/xodr/route_strategy_test_road.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(1, -1, 100, 0);
    start.SetHeadingRelativeRoadDirection(0);
    Position target(7, -2, 10, 0);
    target.SetRouteStrategy(Position::RouteStrategy::MIN_INTERSECTIONS);

    std::vector<int> expectedRoadIds = {1, 101, 5, 401, 7};

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);
    ASSERT_FALSE(path.empty());

    for (size_t i = 0; i < expectedRoadIds.size(); i++)
    {
        ASSERT_EQ(path[i].road->GetId(), expectedRoadIds[i]);
    }
}

TEST_F(FollowRouteTestRouteTestLHT, FindPathShortestLHT)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(), "../../../EnvironmentSimulator/Unittest/xodr/route_strategy_test_road_LHT.xodr");

    // Set start pos and the driving direction (heading)
    // PI = against road dir,   0 = road dir
    Position start(1, 1, 100, 0);
    start.SetHeadingRelativeRoadDirection(0);
    Position target(7, 2, 10, 0);
    target.SetRouteStrategy(Position::RouteStrategy::SHORTEST);

    std::vector<int> expectedRoadIds = {1, 100, 2, 200, 3, 301, 6, 400, 7};

    LaneIndependentRouter router(Position::GetOpenDrive());
    std::vector<Node>     path = router.CalculatePath(start, target);
    ASSERT_FALSE(path.empty());

    for (size_t i = 0; i < expectedRoadIds.size(); i++)
    {
        ASSERT_EQ(path[i].road->GetId(), expectedRoadIds[i]);
    }
}

TEST_F(FollowRouteTestMediumChangedSpeeds, LogWaypointMedium)
{
    ASSERT_NE(Position::GetOpenDrive(), nullptr);
    ASSERT_EQ(Position::GetOpenDrive()->GetOpenDriveFilename(),
              "../../../EnvironmentSimulator/Unittest/xodr/multi_intersections_changed_speeds.xodr");

    std::vector<Position::RouteStrategy> routeStrategies = {Position::RouteStrategy::SHORTEST,
                                                            Position::RouteStrategy::FASTEST,
                                                            Position::RouteStrategy::MIN_INTERSECTIONS};

    // {start, target}
    std::vector<std::pair<Position, Position>> startTargetPairs = {{Position(266, 1, 29, 0), Position(275, 1, 55, 0)},
                                                                   {Position(202, 2, 33, 0), Position(196, 1, 49, 0)},
                                                                   {Position(197, -1, 52, 0), Position(267, 1, 32, 0)},
                                                                   {Position(227, 1, 55, 0), Position(242, -1, 50, 0)},
                                                                   {Position(242, 1, 50, 0), Position(197, -1, 51, 0)}};

    for (auto& pair : startTargetPairs)
    {
        if (pair.first.GetLaneId() < 0)
        {
            pair.first.SetHeadingRelativeRoadDirection(0);
        }
        else
        {
            pair.first.SetHeadingRelativeRoadDirection(M_PI);
        }
    }

    std::ofstream ofs;
    ofs.open("../../../follow_route_log.csv", std::ofstream::trunc);

    LaneIndependentRouter router(Position::GetOpenDrive());

    for (Position::RouteStrategy rs : routeStrategies)
    {
        std::string rsText = "Unknown";
        if (rs == Position::RouteStrategy::SHORTEST)
        {
            rsText = "Shortest";
        }
        else if (rs == Position::RouteStrategy::FASTEST)
        {
            rsText = "Fastest";
        }
        else if (rs == Position::RouteStrategy::MIN_INTERSECTIONS)
        {
            rsText = "MinIntersections";
        }

        for (auto& pair : startTargetPairs)
        {
            Position start  = pair.first;
            Position target = pair.second;
            target.SetRouteStrategy(rs);

            const int bufsize = 128;
            char      buffer[bufsize];
            snprintf(buffer,
                     bufsize,
                     "Start, %d, %d, %.2f, Target, %d, %d, %.2f, RouteStrategy, %s",
                     start.GetTrackId(),
                     start.GetLaneId(),
                     start.GetS(),
                     target.GetTrackId(),
                     target.GetLaneId(),
                     target.GetS(),
                     rsText.c_str());
            ofs << buffer << "\n";
            std::vector<Node>     calculatedPath      = router.CalculatePath(start, target);
            std::vector<Position> calculatedWaypoints = router.GetWaypoints(calculatedPath, start, target);
            for (Position& wp : calculatedWaypoints)
            {
                memset(buffer, 0, bufsize);  // Clear buffer
                snprintf(buffer, bufsize, "Waypoint, %d, %d, %.2f", wp.GetTrackId(), wp.GetLaneId(), wp.GetS());
                ofs << buffer << "\n";
            }
        }
    }
    ofs.close();
}

TEST(FollowRouteTest, CalcWeightShortest)
{
    RoadCalculations roadCalc;
    Road             road1(1, "1", "test");
    road1.SetLength(200);
    double weigth = roadCalc.CalcWeight(nullptr, Position::RouteStrategy::SHORTEST, road1.GetLength(), &road1);
    ASSERT_NEAR(200, weigth, 0.01);
}

TEST(FollowRouteTest, CalcWeightFastest)
{
    RoadCalculations roadCalc;
    Road             road1(1, "1", "test");
    road1.SetLength(200);
    Road::RoadTypeEntry* motorway = new Road::RoadTypeEntry;
    motorway->road_type_          = Road::RoadType::ROADTYPE_MOTORWAY;
    motorway->speed_              = 25;
    road1.AddRoadType(motorway);
    double weigth = roadCalc.CalcWeight(nullptr, Position::RouteStrategy::FASTEST, road1.GetLength(), &road1);
    // 200 / 25 = 8
    ASSERT_NEAR(8, weigth, 0.01);
}

TEST(FollowRouteTest, CalcWeightMinIntersections)
{
    RoadCalculations roadCalc;
    Road             road1(1, "1", "test");
    road1.SetLength(200);
    RoadLink plink(LinkType::SUCCESSOR, RoadLink::ELEMENT_TYPE_JUNCTION, "1", ContactPointType::CONTACT_POINT_UNDEFINED);
    Node     pNode;
    pNode.link    = &plink;
    double weigth = roadCalc.CalcWeight(&pNode, Position::RouteStrategy::MIN_INTERSECTIONS, road1.GetLength(), &road1);
    ASSERT_NEAR(1, weigth, 0.01);

    Road road2(2, "2", "test");
    road1.SetLength(200);
    RoadLink plink2(LinkType::SUCCESSOR, RoadLink::ELEMENT_TYPE_ROAD, "1", ContactPointType::CONTACT_POINT_UNDEFINED);
    Node     pNode2;
    pNode2.link    = &plink2;
    double weigth2 = roadCalc.CalcWeight(&pNode2, Position::RouteStrategy::MIN_INTERSECTIONS, road1.GetLength(), &road1);
    ASSERT_NEAR(0, weigth2, 0.01);
}

TEST(FollowRouteTest, CalcAverageSpeedForRoadsWithoutSpeed)
{
    RoadCalculations roadCalc;

    Road                 road1(1, "1", "420");
    Road                 road2(2, "2", "420");
    Road                 road3(3, "3", "420");
    Road                 road4(4, "4", "420");
    Road::RoadTypeEntry* lowSpeed = new Road::RoadTypeEntry;
    Road::RoadTypeEntry* town     = new Road::RoadTypeEntry;
    Road::RoadTypeEntry* rural    = new Road::RoadTypeEntry;
    Road::RoadTypeEntry* motorway = new Road::RoadTypeEntry;
    lowSpeed->road_type_          = Road::RoadType::ROADTYPE_LOWSPEED;
    town->road_type_              = Road::RoadType::ROADTYPE_TOWN;
    rural->road_type_             = Road::RoadType::ROADTYPE_RURAL;
    motorway->road_type_          = Road::RoadType::ROADTYPE_MOTORWAY;

    road1.AddRoadType(lowSpeed);
    road2.AddRoadType(town);
    road3.AddRoadType(rural);
    road4.AddRoadType(motorway);

    double averageSpeed  = roadCalc.CalcAverageSpeed(&road1);
    double expectedSpeed = 8.333;
    ASSERT_NEAR(averageSpeed, expectedSpeed, 0.01);

    averageSpeed  = roadCalc.CalcAverageSpeed(&road2);
    expectedSpeed = 13.888;
    ASSERT_NEAR(averageSpeed, expectedSpeed, 0.01);

    averageSpeed  = roadCalc.CalcAverageSpeed(&road3);
    expectedSpeed = 19.444;
    ASSERT_NEAR(averageSpeed, expectedSpeed, 0.01);

    averageSpeed  = roadCalc.CalcAverageSpeed(&road4);
    expectedSpeed = 25;
    ASSERT_NEAR(averageSpeed, expectedSpeed, 0.01);
}

TEST(FollowRouteTest, CalcAverageSpeedForRoadsWithDefinedSpeed)
{
    RoadCalculations roadCalc;

    Road                 road1(1, "1", "420");
    Road                 road2(2, "2", "420");
    Road                 road3(3, "3", "420");
    Road                 road4(4, "4", "420");
    Road::RoadTypeEntry* lowSpeed = new Road::RoadTypeEntry;
    Road::RoadTypeEntry* town     = new Road::RoadTypeEntry;
    Road::RoadTypeEntry* rural    = new Road::RoadTypeEntry;
    Road::RoadTypeEntry* motorway = new Road::RoadTypeEntry;
    lowSpeed->road_type_          = Road::RoadType::ROADTYPE_LOWSPEED;
    lowSpeed->speed_              = 10;
    town->road_type_              = Road::RoadType::ROADTYPE_TOWN;
    town->speed_                  = 20;
    rural->road_type_             = Road::RoadType::ROADTYPE_RURAL;
    rural->speed_                 = 30;
    motorway->road_type_          = Road::RoadType::ROADTYPE_MOTORWAY;
    motorway->speed_              = 40;

    road1.AddRoadType(lowSpeed);
    road2.AddRoadType(town);
    road3.AddRoadType(rural);
    road4.AddRoadType(motorway);

    double averageSpeed  = roadCalc.CalcAverageSpeed(&road1);
    double expectedSpeed = 10;
    ASSERT_NEAR(averageSpeed, expectedSpeed, 0.01);

    averageSpeed  = roadCalc.CalcAverageSpeed(&road2);
    expectedSpeed = 20;
    ASSERT_NEAR(averageSpeed, expectedSpeed, 0.01);

    averageSpeed  = roadCalc.CalcAverageSpeed(&road3);
    expectedSpeed = 30;
    ASSERT_NEAR(averageSpeed, expectedSpeed, 0.01);

    averageSpeed  = roadCalc.CalcAverageSpeed(&road4);
    expectedSpeed = 40;
    ASSERT_NEAR(averageSpeed, expectedSpeed, 0.01);
}

TEST(FollowRouteTest, CalcAverageSpeedForTwoRoadTypes)
{
    RoadCalculations roadCalc;

    Road                 road(1, "1", "420");
    Road::RoadTypeEntry* lowSpeed = new Road::RoadTypeEntry;
    Road::RoadTypeEntry* town     = new Road::RoadTypeEntry;
    lowSpeed->road_type_          = Road::RoadType::ROADTYPE_LOWSPEED;
    town->road_type_              = Road::RoadType::ROADTYPE_TOWN;

    road.AddRoadType(lowSpeed);
    road.AddRoadType(town);

    double averageSpeed  = roadCalc.CalcAverageSpeed(&road);
    double expectedSpeed = 11.11;
    ASSERT_NEAR(averageSpeed, expectedSpeed, 0.01);

    lowSpeed->speed_ = 10;
    town->speed_     = 20;

    averageSpeed  = roadCalc.CalcAverageSpeed(&road);
    expectedSpeed = 15;
    ASSERT_NEAR(averageSpeed, expectedSpeed, 0.01);
}

int main(int argc, char** argv)
{
    // testing::GTEST_FLAG(filter) = "*TestOptionHandling*";
    testing::InitGoogleTest(&argc, argv);

    if (argc > 1)
    {
        if (!strcmp(argv[1], "--disable_stdout"))
        {
            // disable logging to stdout from the test cases
            SE_Env::Inst().GetOptions().SetOptionValue("disable_stdout", "", false, true);
        }
        else
        {
            printf("Usage: %s [--disable_stout] [google test options...]\n", argv[0]);
            return -1;
        }
    }

    return RUN_ALL_TESTS();
}
