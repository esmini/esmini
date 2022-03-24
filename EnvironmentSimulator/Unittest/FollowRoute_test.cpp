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

static void log_callback(const char *str);

TEST(PathfinderTest, FindPathTest1)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../../esmini/resources/xodr/multi_intersections.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    Position start(202, 2, 100, 0);
    Position target(209, 1, 20, 0);

    LaneIndependentRouter router(odr);
    std::vector<Node *> path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path[0]->road->GetId(), 209);
}

TEST(PathfinderTest, FindPathTest2)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../../esmini/resources/xodr/multi_intersections.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    Position start(217, -1, 50, 0);
    Position target(275, -1, 50, 0);

    LaneIndependentRouter router(odr);
    std::vector<Node *> path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path[0]->road->GetId(), 275);
}

TEST(PathfinderTest, FindPathShortest)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../../esmini/resources/xodr/multi_intersections.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    Position start(202, 2, 100, 0);
    Position target(209, 1, 20, 0);

    LaneIndependentRouter router(odr);
    std::vector<Node *> path = router.CalculatePath(start, target, RouteStrategy::SHORTEST);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path[0]->road->GetId(), 209);
}

TEST(PathfinderTest, FindPathFastest)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../../esmini/resources/xodr/multi_intersections.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    Position start(202, 2, 100, 0);
    Position target(209, 1, 20, 0);

    LaneIndependentRouter router(odr);
    std::vector<Node *> path = router.CalculatePath(start, target, RouteStrategy::FASTEST);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path[0]->road->GetId(), 209);
}

TEST(PathfinderTest, FindPathMinIntersections)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../../esmini/resources/xodr/multi_intersections.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    Position start(202, 2, 100, 0);
    Position target(209, 1, 20, 0);

    LaneIndependentRouter router(odr);
    // std::vector<Node *> path = router.CalculatePath(start, target, RouteStrategy::MIN_INTERSECTIONS);
    std::vector<Node *> path = {};

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path[0]->road->GetId(), 209);
}

TEST(PathfinderTest, FindPathTimeSmall)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../../esmini/resources/xodr/multi_intersections.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    Position start(202, 2, 100, 0);
    Position target(209, 1, 20, 0);

    LaneIndependentRouter router(odr);
    auto startTime = std::chrono::high_resolution_clock::now();
    std::vector<Node *> path = router.CalculatePath(start, target, RouteStrategy::MIN_INTERSECTIONS);
    auto endTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path[0]->road->GetId(), 209);

    int maxMicroseconds = 5000;
    ASSERT_LT((int)elapsedTime, maxMicroseconds);
}

TEST(PathfinderTest, FindPathTimeLarge)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../../large.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    Position start(2291, -1, 1100, 0);
    Position target(2502, 1, 50, 0);

    LaneIndependentRouter router(odr);
    auto startTime = std::chrono::high_resolution_clock::now();
    std::vector<Node *> path = router.CalculatePath(start, target, RouteStrategy::MIN_INTERSECTIONS);
    auto endTime = std::chrono::high_resolution_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path[0]->road->GetId(), 2502);

    int maxMicroseconds = 15000;
    ASSERT_LT((int)elapsedTime, maxMicroseconds);
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