#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <chrono>

#include "pugixml.hpp"
#include "simple_expr.h"
#include "LaneIndependentRouter.hpp"
#include "ScenarioEngine.hpp"
#include "ScenarioReader.hpp"

#define TRIG_ERR_MARGIN 0.001

using namespace roadmanager;
using namespace scenarioengine;
class FollowRouteControllerTest : public ::testing::Test
{
public:
    static void SetUpTestSuite()
    {
    }
};

static void log_callback(const char *str);

TEST_F(FollowRouteControllerTest, PerformSingleLaneChange)
{
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/perform_single_lane_change.xosc");
    ASSERT_NE(se, nullptr);

    Position start(0, -1, 10, 0);
    Position target(3, -2, 20, 0);

    double dt = 0.1;

    // Fast forward
    while (se->getSimulationTime() < (10.0 - SMALL_NUMBER))
    {
        Position p = se->entities_.object_[0]->pos_;
        // LOG("s=%f, r=%d, l=%d", p.GetS(), p.GetTrackId(), p.GetLaneId());
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    Position finalPos = se->entities_.object_[0]->pos_;
    ASSERT_EQ(target.GetTrackId(), finalPos.GetTrackId());
    ASSERT_EQ(target.GetLaneId(), finalPos.GetLaneId());
    ASSERT_NEAR(target.GetS(), finalPos.GetS(), 10);
}

TEST_F(FollowRouteControllerTest, FollowRouteWithLaneChanges)
{
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/follow_route_with_lane_change.xosc");
    ASSERT_NE(se, nullptr);

    Position start(0, -1, 10, 0);
    Position target(2, -1, 20, 0);

    double dt = 0.1;

    // Fast forward
    while (se->getSimulationTime() < (15.0 - SMALL_NUMBER))
    {
        Position p = se->entities_.object_[0]->pos_;
        // LOG("s=%f, r=%d, l=%d", p.GetS(), p.GetTrackId(), p.GetLaneId());
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    Position finalPos = se->entities_.object_[0]->pos_;
    ASSERT_EQ(target.GetTrackId(), finalPos.GetTrackId());
    ASSERT_EQ(target.GetLaneId(), finalPos.GetLaneId());
    ASSERT_NEAR(target.GetS(), finalPos.GetS(), 10);
}


TEST_F(FollowRouteControllerTest, FollowRouteWithCollisionRisk)
{
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/lane_change_with_collision_risk.xosc");
    ASSERT_NE(se, nullptr);

    Position start(0, -1, 10, 0);
    Position target(5, -3, 20, 0);

    double dt = 0.1;

    // Fast forward
    while (se->getSimulationTime() < (15.0 - SMALL_NUMBER))
    {
        Position p = se->entities_.object_[0]->pos_;
        // LOG("s=%f, r=%d, l=%d", p.GetS(), p.GetTrackId(), p.GetLaneId());
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    Position finalPos = se->entities_.object_[0]->pos_;
    ASSERT_EQ(target.GetTrackId(), finalPos.GetTrackId());
    ASSERT_EQ(target.GetLaneId(), finalPos.GetLaneId());
    ASSERT_NEAR(target.GetS(), finalPos.GetS(), 10);
}

TEST_F(FollowRouteControllerTest, FollowRouteMedium)
{
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/follow_route_controller_test_medium.xosc");
    ASSERT_NE(se, nullptr);

    Position start(202, 2, 80, 0);
    Position target(196, 1, 50, 0);

    double dt = 0.1;

    // Fast forward
    while (se->getSimulationTime() < (100 - SMALL_NUMBER))
    {
        Position p = se->entities_.object_[0]->pos_;
        // LOG("s=%f, r=%d, l=%d", p.GetS(), p.GetTrackId(), p.GetLaneId());
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    Position finalPos = se->entities_.object_[0]->pos_;
    ASSERT_EQ(target.GetTrackId(), finalPos.GetTrackId());
    ASSERT_EQ(target.GetLaneId(), finalPos.GetLaneId());
    ASSERT_NEAR(target.GetS(), finalPos.GetS(), 10);
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