#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <stdexcept>
#include <filesystem>

#include "pugixml.hpp"
#include "simple_expr.h"
#include "LaneIndependentRouter.hpp"

#define TRIG_ERR_MARGIN 0.001

using namespace roadmanager;

static void log_callback(const char *str);

TEST(PathfinderTest, FindPathTest1)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("~/esmini/resources/xodr/multi_intersections.xodr");
    OpenDrive *odr = Position::GetOpenDrive();
    ASSERT_NE(odr, nullptr);

    Position start(202, 2, 100, 0);
    Position target(209, 1, 20, 0);

    LaneIndependentRouter router(odr);
    std::vector<Node *> path = router.CalculatePath(start, target);

    ASSERT_FALSE(path.empty());
    ASSERT_EQ(path.back()->road->GetId(), 209);
}

// TEST(PathfinderTest, FindPathTest2)
// {
//     Position::GetOpenDrive()->LoadOpenDriveFile("~/esmini/resources/xodr/multi_intersections.xodr");
//     OpenDrive *odr = Position::GetOpenDrive();

//     ASSERT_NE(odr, nullptr);

//     ScenarioEngine *se = new ScenarioEngine("../../controller_follow_route_test.xosc");
//     ASSERT_NE(se, nullptr);

//     Position start(217, -1, 50, 0);
//     Position target(275, -1, 50, 0);

//     scenarioengine::Controller::InitArgs args;
//     args.name = "ControllerFollowRoute";
//     args.type = ControllerFollowRoute::GetTypeNameStatic();
//     args.parameters = 0;
//     args.properties = new OSCProperties();
//     args.gateway = se->getScenarioGateway();
//     ControllerFollowRoute *controllerFollowRoute = (ControllerFollowRoute *)InstantiateControllerFollowRoute(&args);
//     delete se->entities_.object_[0]->controller_;
//     delete args.properties;

//     controllerFollowRoute->Assign(se->entities_.object_[0]);
//     se->scenarioReader->controller_[0] = controllerFollowRoute;
//     se->entities_.object_[0]->controller_ = controllerFollowRoute;

//     std::vector<Node *> path = controllerFollowRoute->CalculatePath(start, target);

//     ASSERT_FALSE(path.empty());
//     ASSERT_EQ(path.back()->road->GetId(), 275);
// }

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