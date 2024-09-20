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
#include "ControllerFollowRoute.hpp"

#define TRIG_ERR_MARGIN 0.001

using namespace roadmanager;
using namespace scenarioengine;
class FollowRouteControllerTest : public ::testing::Test
{
public:
    static void SetUpTestSuite()
    {
        SE_Env::Inst().AddPath("../../../resources/traffic_signals");
    }
};

TEST_F(FollowRouteControllerTest, PerformSingleLaneChange)
{
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/perform_single_lane_change.xosc");
    ASSERT_NE(se, nullptr);
    if (se->GetInitStatus() != 0)
    {
        delete se;
        GTEST_FAIL();
    }

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
    EXPECT_EQ(target.GetTrackId(), finalPos.GetTrackId());
    EXPECT_EQ(target.GetLaneId(), finalPos.GetLaneId());
    EXPECT_NEAR(target.GetS(), finalPos.GetS(), 10);

    delete se;
}

TEST_F(FollowRouteControllerTest, FollowRouteWithLaneChanges)
{
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/follow_route_with_lane_change.xosc");
    ASSERT_NE(se, nullptr);
    if (se->GetInitStatus() != 0)
    {
        delete se;
        GTEST_FAIL();
    }

    Position target(2, -1, 20, 0);

    double dt = 0.1;

    // Fast forward
    while (se->getSimulationTime() < (15.0 - SMALL_NUMBER))
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    Position finalPos = se->entities_.object_[0]->pos_;
    ASSERT_EQ(target.GetTrackId(), finalPos.GetTrackId());
    ASSERT_EQ(target.GetLaneId(), finalPos.GetLaneId());
    ASSERT_NEAR(target.GetS(), finalPos.GetS(), 10);

    delete se;
}

TEST_F(FollowRouteControllerTest, FollowRouteWithCollisionRisk)
{
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/follow_route_collision_risk.xosc");
    ASSERT_NE(se, nullptr);
    if (se->GetInitStatus() != 0)
    {
        delete se;
        GTEST_FAIL();
    }

    Position target(5, -3, 20, 0);

    double dt = 0.1;

    // Fast forward
    while (se->getSimulationTime() < (15.0 - SMALL_NUMBER))
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    Position finalPos = se->entities_.object_[0]->pos_;
    ASSERT_EQ(target.GetTrackId(), finalPos.GetTrackId());
    ASSERT_EQ(target.GetLaneId(), finalPos.GetLaneId());
    ASSERT_NEAR(target.GetS(), finalPos.GetS(), 10);

    delete se;
}

TEST_F(FollowRouteControllerTest, FollowRouteBlockedByCollisionRisk)
{
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/follow_route_blocked_by_collision_risk.xosc");
    ASSERT_NE(se, nullptr);
    if (se->GetInitStatus() != 0)
    {
        delete se;
        GTEST_FAIL();
    }

    // Can't change lane due to collision
    // Vehicle stops when reached target road on wrong lane
    Position target(5, -1, 0, 0);

    double dt = 0.1;

    // Fast forward
    while (se->getSimulationTime() < (15.0 - SMALL_NUMBER))
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    Position finalPos = se->entities_.object_[0]->pos_;
    ASSERT_EQ(target.GetTrackId(), finalPos.GetTrackId());
    ASSERT_EQ(target.GetLaneId(), finalPos.GetLaneId());

    delete se;
}

TEST_F(FollowRouteControllerTest, FollowRouteMedium)
{
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/follow_route_controller_test_medium.xosc");
    ASSERT_NE(se, nullptr);
    if (se->GetInitStatus() != 0)
    {
        delete se;
        GTEST_FAIL();
    }

    Position target(196, 1, 50, 0);

    double dt = 0.1;

    // Fast forward
    while (se->getSimulationTime() < (35 - SMALL_NUMBER))
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    Position finalPos = se->entities_.object_[0]->pos_;
    ASSERT_EQ(target.GetTrackId(), finalPos.GetTrackId());
    ASSERT_EQ(target.GetLaneId(), finalPos.GetLaneId());
    ASSERT_NEAR(target.GetS(), finalPos.GetS(), 10);

    delete se;
}

TEST_F(FollowRouteControllerTest, FollowRouteNoPath)
{
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/follow_route_no_path.xosc");
    ASSERT_NE(se, nullptr);
    if (se->GetInitStatus() != 0)
    {
        delete se;
        GTEST_FAIL();
    }

    double dt = 0.1;
    // Perform one step so that the object position is set
    se->step(dt);
    se->prepareGroundTruth(dt);

    Position start = se->entities_.object_[0]->pos_;
    Position target(5, 1, 30, 0);

    // Fast forward
    while (se->getSimulationTime() < (30 - SMALL_NUMBER))
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    Position finalPos = se->entities_.object_[0]->pos_;
    // Expected behavior is that car stops when no path is found
    ASSERT_NE(target.GetTrackId(), finalPos.GetTrackId());
    ASSERT_NE(target.GetLaneId(), finalPos.GetLaneId());
    ASSERT_EQ(start.GetTrackId(), finalPos.GetTrackId());
    ASSERT_EQ(start.GetLaneId(), finalPos.GetLaneId());

    delete se;
}

TEST_F(FollowRouteControllerTest, FollowRouteMultipleScenarioWaypoints)
{
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/follow_route_multiple_scenario_waypoints.xosc");
    ASSERT_NE(se, nullptr);
    if (se->GetInitStatus() != 0)
    {
        delete se;
        GTEST_FAIL();
    }

    std::vector<Position> scenarioWaypoints = {Position(284, -1, 10, 0), Position(196, 1, 10, 0), Position(202, 2, 40, 0)};
    std::vector<Position> passedPositions;

    double dt = 0.1;

    // Fast forward
    while (se->getSimulationTime() < (100 - SMALL_NUMBER))
    {
        Position p = se->entities_.object_[0]->pos_;
        passedPositions.push_back(p);
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    for (Position &scenarioWp : scenarioWaypoints)
    {
        bool hasPassedWaypoint = std::find_if(passedPositions.begin(),
                                              passedPositions.end(),
                                              [&](const Position &p) {
                                                  return p.GetTrackId() == scenarioWp.GetTrackId() && p.GetLaneId() == scenarioWp.GetLaneId() &&
                                                         abs(p.GetS() - scenarioWp.GetS()) < 5;
                                              }) != passedPositions.end();
        ASSERT_TRUE(hasPassedWaypoint);
    }

    Position finalPos = se->entities_.object_[0]->pos_;
    ASSERT_EQ(scenarioWaypoints.back().GetTrackId(), finalPos.GetTrackId());
    ASSERT_EQ(scenarioWaypoints.back().GetLaneId(), finalPos.GetLaneId());
    ASSERT_NEAR(scenarioWaypoints.back().GetS(), finalPos.GetS(), 10);

    delete se;
}

TEST_F(FollowRouteControllerTest, FollowRouteSetParameters)
{
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/follow_route_set_parameters.xosc");
    ASSERT_NE(se, nullptr);
    if (se->GetInitStatus() != 0)
    {
        delete se;
        GTEST_FAIL();
    }

    scenarioengine::ControllerFollowRoute *controller = static_cast<scenarioengine::ControllerFollowRoute *>(
        se->entities_.object_[0]->GetAssignedControllerOftype(scenarioengine::Controller::Type::CONTROLLER_TYPE_FOLLOW_ROUTE));
    ASSERT_NE(controller, nullptr);
    ASSERT_NEAR(controller->GetMinDistForCollision(), 69, 0.01);
    ASSERT_NEAR(controller->GetLaneChangeTime(), 420, 0.01);

    delete se;
}

// Uncomment to print log output to console
// #define LOG_TO_CONSOLE

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

    // testing::GTEST_FLAG(filter) = "*PerformSingleLaneChange*";

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}