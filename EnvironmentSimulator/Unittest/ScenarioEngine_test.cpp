#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <stdexcept>

#include "ScenarioEngine.hpp"
#include "pugixml.hpp"
#include "simple_expr.h"

using namespace roadmanager;
using namespace scenarioengine;

#define TRIG_ERR_MARGIN 0.001

TEST(DistanceTest, CalcDistanceVariations)
{
    double dist = 0.0;
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/curve_r100.xodr");
    OpenDrive* odr = Position::GetOpenDrive();

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    Position pos0 = Position(0, -1, 400.0, 0);
    pos0.SetHeading(0.0);

    // another point some meters ahead, still on the same straight segment
    Position pos1 = Position(0, -1, 410.0, 0);
    pos1.SetHeading(0.0);
    ASSERT_EQ(pos0.Distance(&pos1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, dist), 0);
    EXPECT_NEAR(dist, 10.0, 1e-5);

    // same point but measure now in road coordinates
    Object obj0(Object::Type::VEHICLE);
    obj0.boundingbox_.center_ = { 1.0, 0.0, 0.0 };
    obj0.boundingbox_.dimensions_ = { 2.0, 2.0, 2.0 };
    obj0.pos_ = pos0;

    Object obj1(Object::Type::VEHICLE);
    obj1.pos_ = pos1;
    obj1.boundingbox_.center_ = { 1.0, 0.0, 0.0 };
    obj1.boundingbox_.dimensions_ = { 2.0, 2.0, 2.0 };

    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist), 0);
    EXPECT_NEAR(dist, 8.0, 1e-5);

    // Modify boundingbox center
    obj1.boundingbox_.center_ = { 1.5, 0.0, 0.0 };
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist), 0);
    EXPECT_NEAR(dist, 8.5, 1e-5);

    // Move object to curve
    obj0.pos_.SetLanePos(0, -1, 510.0, 0.0);
    obj1.pos_.SetLanePos(0, -1, 550.0, 2.5);

    // ref point to ref point (no freespace)
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, false, dist), 0);
    EXPECT_NEAR(dist, 40.0, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, false, dist), 0);
    EXPECT_NEAR(dist, 2.5, 1e-5);

    // freespace
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist), 0);
    EXPECT_NEAR(dist, 38.510726, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.468766, 1e-5);

    // freespace with overlap
    obj0.pos_.SetAlignMode(Position::ALIGN_MODE::ALIGN_HARD);
    obj0.pos_.SetLanePos(0, -1, 549.0, 0.0);
    obj1.pos_.SetAlignMode(Position::ALIGN_MODE::ALIGN_HARD);
    obj1.pos_.SetLanePos(0, -1, 550.0, 0.0);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.0, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.0, 1e-5);

    // Create a lateral gap
    obj1.pos_.SetLanePos(0, -1, 550.0, 2.5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, false, dist), 0);
    EXPECT_NEAR(dist, 2.5, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.4687658, 1e-5);

    // Move to straight segment and do same measurement
    obj0.pos_.SetLanePos(0, -1, 50.0, 0.0);
    obj1.pos_.SetLanePos(0, -1, 51.0, 2.5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, false, dist), 0);
    EXPECT_NEAR(dist, 2.5, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.5, 1e-5);

    // Move second object behind first one
    obj0.pos_.SetLanePos(0, -1, 50.0, 0.0);
    obj1.pos_.SetLanePos(0, -1, 49.0, 2.5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, false, dist), 0);
    EXPECT_NEAR(dist, 2.5, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.5, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, false, dist), 0);
    EXPECT_NEAR(dist, -1.0, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.0, 1e-5);

    // Move to other side of road, sign of relative distances should be the same
    obj0.pos_.SetLanePos(0, 1, 49.0, 0.0);
    obj0.pos_.SetHeadingRelative(M_PI);
    obj1.pos_.SetLanePos(0, 1, 50.0, -2.5);
    obj1.pos_.SetHeadingRelative(M_PI);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, false, dist), 0);
    EXPECT_NEAR(dist, 2.5, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.5, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, false, dist), 0);
    EXPECT_NEAR(dist, -1.0, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist), 0);
    EXPECT_NEAR(dist, 0.0, 1e-5);
}

TEST(DistanceTest, CalcDistancePoint)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/curve_r100.xodr");
    OpenDrive* odr = Position::GetOpenDrive();

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    Position pos0 = Position(0, -1, 400.0, 0);
    pos0.SetHeading(0.0);

    Object obj0(Object::Type::VEHICLE);
    obj0.boundingbox_.center_ = { 1.0, 0.0, 0.0 };
    obj0.boundingbox_.dimensions_ = { 2.0, 2.0, 2.0 };
    obj0.pos_ = pos0;

    // Measure from X, Y point to object in road coordinates
    double latDist = 0.0;
    double longDist = 0.0;

    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos0.GetX() + 20.0, pos0.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, 18.0, 1e-5);

    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos0.GetX() - 20.0, pos0.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, -20.0, 1e-5);

    obj0.boundingbox_.dimensions_ = { 2.0, 5.0, 2.0 };
    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos0.GetX() - 20.0, pos0.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, -18.5, 1e-5);

    obj0.boundingbox_.center_ = { 2.0, 4.0, 0.0 };
    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos0.GetX() - 20.0, pos0.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, -19.5, 1e-5);

    obj0.boundingbox_.center_ = { 2.0, 4.0, 0.0 };
    Position tmpPos(0, 600, obj0.pos_.GetT());
    EXPECT_NEAR(tmpPos.GetX(), 585.438756, 1e-5);
    EXPECT_NEAR(tmpPos.GetY(), 45.140405, 1e-5);
    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(tmpPos.GetX(), tmpPos.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, 195.5, 1e-5);
    EXPECT_NEAR(latDist, 3.0, 1e-5);
}

TEST(DistanceTest, CalcDistancePointAcrossIntersection)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive* odr = Position::GetOpenDrive();

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    Position pos0 = Position(0, 1, 10.0, 0);
    pos0.SetHeading(0.0);

    Object obj0(Object::Type::VEHICLE);
    obj0.boundingbox_.center_ = { 1.0, 0.0, 0.0 };
    obj0.boundingbox_.dimensions_ = { 2.0, 2.0, 2.0 };
    obj0.pos_ = pos0;

    // another point some meters ahead, still on the same straight segment
    Position pos1 = Position(2, 1, 290.0, 0);

    // Measure from X, Y point to object in road coordinates
    double latDist = 0.0;
    double longDist = 0.0;

    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos1.GetX(), pos1.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, -38.587016, 1e-5);
    EXPECT_NEAR(latDist, 0.221272, 1e-5);

}

TEST(TrajectoryTest, EnsureContinuation)
{
    double dt = 0.01;
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/trajectory-continuity.xosc");
    ASSERT_NE(se, nullptr);

    for (int i = 0; i < (int)(1.0/dt); i++)
    {
        se->step(dt);
        se->prepareOSIGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities.object_[0]->pos_.GetX(), 4.95, 1e-5);
    ASSERT_NEAR(se->entities.object_[0]->pos_.GetY(), -1.535, 1e-5);

    for (int i = 0; i < (int)(2.0 / dt); i++)
    {
        se->step(dt);
        se->prepareOSIGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities.object_[0]->pos_.GetX(), 14.92759, 1e-5);
    ASSERT_NEAR(se->entities.object_[0]->pos_.GetY(), -1.18333, 1e-5);

    for (int i = 0; i < (int)(1.5 / dt); i++)
    {
        se->step(dt);
        se->prepareOSIGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities.object_[0]->pos_.GetX(), 21.31489, 1e-5);
    ASSERT_NEAR(se->entities.object_[0]->pos_.GetY(), 2.56606, 1e-5);
}

TEST(ExpressionTest, EnsureResult)
{
    ASSERT_FLOAT_EQ(eval_expr("1 + 1"), 2.0f);
    ASSERT_FLOAT_EQ(eval_expr("5 * 10 + 1"), 51.0f);
    ASSERT_FLOAT_EQ(eval_expr("5 * (10 + 1)"), 55.0f);
    ASSERT_FLOAT_EQ(eval_expr("15/3.5"), 15.0f / 3.5f);
    ASSERT_FLOAT_EQ(eval_expr("15 % 6"), 3.0f);
    ASSERT_FLOAT_EQ(eval_expr("-15 % 6"), -3.0f);
    ASSERT_FLOAT_EQ(eval_expr("1 == 1"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("1 == 2"), 0.0f);
    ASSERT_FLOAT_EQ(eval_expr("(4 == 4) && (10 == 10)"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("4 == 4 && 10 == 10"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("4 == 4 && 9 < 10"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("4 == 4 || 11 == 10"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("4 == 3 || 9 < 8"), 0.0f);
    ASSERT_FLOAT_EQ(eval_expr("ceil(11.1) == 12"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("round(11.1) == 11"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("floor(11.9) == 11"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("ceil(-11.1) == -11"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("round(-11.1) == -11"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("floor(-11.9) == -12"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("pow(2,3)"), 8.0f);
    ASSERT_FLOAT_EQ(eval_expr("2**3"), 8.0f);
}

TEST(OptionsTest, TestOptionHandling)
{
    SE_Options opt;

    // define arguments
    opt.AddOption("osc_file", "Scenario file", "filename");
    opt.AddOption("odr_file", "Roadnetwork file", "filename", "default_road.xodr");
    opt.AddOption("option2", "Some value", "value");
    opt.AddOption("option3", "Some rate", "rate", "55");
    opt.AddOption("option4", "Some temp", "temp");
    opt.AddOption("option5", "Some speed", "speed");
    opt.AddOption("window", "Visualize the scenario");
    // opt.PrintUsage();

    // set arguments
    const char* args[] = {
        "my_app",
        "--osc_file",
        "my_scenario.xosc",
        "--odr_file",
        "my_road.xodr",
        "--window",
        "--option2",
        "--option3",
        "--option4"
    };
    int argc = sizeof(args) / sizeof(char*);
    char** argv;
    argv = (char**)malloc(argc * sizeof(char*));
    for (int i = 0; i < argc; i++)
    {
        argv[i] = (char*)malloc(strlen(args[i]) + 1);
        strncpy(argv[i], args[i], strlen(args[i]) + 1);
    }

    opt.ParseArgs(&argc, argv);

    ASSERT_EQ(opt.GetOptionSet("no_arg"), false);
    ASSERT_EQ(opt.GetOptionSet("osc_file"), true);
    ASSERT_EQ(opt.GetOptionSet("window"), true);
    ASSERT_EQ(opt.GetOptionSet("option2"), false);
    ASSERT_EQ(opt.GetOptionSet("option3"), true);
    ASSERT_EQ(opt.GetOptionSet("option4"), false);
    ASSERT_EQ(opt.GetOptionSet("option5"), false);
    ASSERT_EQ(opt.GetOptionArg("window"), "");
    ASSERT_EQ(opt.GetOptionArg("osc_file"), "my_scenario.xosc");
    ASSERT_EQ(opt.GetOptionArg("odr_file"), "my_road.xodr");
    ASSERT_EQ(opt.GetOptionArg("option3"), "55");

    // Clean up
    for (int i = 0; i < argc; i++)
    {
        delete argv[i];
    }
    delete argv;
}

TEST(ParameterTest, ResolveParameterTest)
{
    Parameters params;

    params.parameterDeclarations_.Parameter.push_back( { "speed", OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE, {0, 5.0, "5.0", false} });
    params.parameterDeclarations_.Parameter.push_back({ "acc", OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE, {0, 3.0, "3.0", false} });
    params.parameterDeclarations_.Parameter.push_back({ "turnsignal", OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE, {0, 0.0, "true", true} });

    ASSERT_EQ(params.ResolveParametersInString("$speed + 1.0"), "5.0 + 1.0");
    ASSERT_EQ(params.ResolveParametersInString("$speed $acc"), "5.0 3.0");
    ASSERT_EQ(params.ResolveParametersInString("$speed $acc "), "5.0 3.0 ");
    ASSERT_EQ(params.ResolveParametersInString(" $speed $acc "), " 5.0 3.0 ");
    ASSERT_EQ(params.ResolveParametersInString(" speed $acc "), " speed 3.0 ");
    ASSERT_EQ(params.ResolveParametersInString(" $turnsignal "), " true ");
}

TEST(ParameterTest, ParseParameterTest)
{
    // Create parameter declarations
    pugi::xml_document xml_doc;
    pugi::xml_node paramDeclsNode = xml_doc.append_child("paramDeclsNode");

    pugi::xml_node paramDeclNode0 = paramDeclsNode.append_child("paramDeclNode0");
    paramDeclNode0.append_attribute("name") = "param0";
    paramDeclNode0.append_attribute("parameterType") = "double";
    paramDeclNode0.append_attribute("value") = "17.0";

    pugi::xml_node paramDeclNode1 = paramDeclsNode.append_child("paramDeclNode1");
    paramDeclNode1.append_attribute("name") = "param1";
    paramDeclNode1.append_attribute("parameterType") = "boolean";
    paramDeclNode1.append_attribute("value") = "true";

    Parameters params;
    params.addParameterDeclarations(paramDeclsNode);

    // Create an XML element with attributes referring to parameters
    pugi::xml_node someNode0 = xml_doc.append_child("someNode0");
    someNode0.append_attribute("speed") = "5.1";
    someNode0.append_attribute("acc") = "$param0";
    someNode0.append_attribute("attr2") = "${$param0 + 0.5}";
    someNode0.append_attribute("attr3") = "${$param1 && (1==1)}";
    someNode0.append_attribute("attr4") = "${$param1 || (0==1)}";
    someNode0.append_attribute("attr5") = "${$param1 and (0==1)}";
    someNode0.append_attribute("attr6") = "${$param1 and not(0==1)}";
    someNode0.append_attribute("attr7") = "${$param1 and not (0==1)}";
    someNode0.append_attribute("attr8") = "${$param1 and (26 == $param0 + 9)}";
    someNode0.append_attribute("attr9") = "${(2 + ($param0- 9)/2)/3}";

    // verify correct parameter lookup
    ASSERT_EQ(params.ReadAttribute(someNode0, "speed", false), "5.1");
    ASSERT_EQ(params.ReadAttribute(someNode0, "acc", false), "17.0");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr2", false), "17.500000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr3", false), "1.000000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr4", false), "1.000000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr5", false), "0.000000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr6", false), "1.000000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr7", false), "1.000000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr8", false), "1.000000");
    ASSERT_EQ(params.ReadAttribute(someNode0, "attr9", false), "2.000000");
}

TEST(JunctionTest, JunctionSelectorTest)
{
    double dt = 0.01;

    double angles[] = { 3 * M_PI_2, -M_PI_2, 0.0, M_PI_2 };
    int roadIds[] = { 1, 1, 2, 3 };
    double durations[] = { 2.5, 2.5, 2.6, 2.8 };  // Make sure car gets gets out of the intersection

    for (int i = 0; i < sizeof(angles) / sizeof(double); i++)
    {
        ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/junction-selector.xosc");
        ASSERT_NE(se, nullptr);

        // Turn always right
        se->entities.object_[0]->SetJunctionSelectorStrategy(roadmanager::Junction::JunctionStrategyType::SELECTOR_ANGLE);
        se->entities.object_[0]->SetJunctionSelectorAngle(angles[i]);
        while (se->getSimulationTime() < durations[i] && se->GetQuitFlag() != true)
        {
            se->step(dt);
        }
        ASSERT_EQ(se->entities.object_[0]->pos_.GetTrackId(), roadIds[i]);
        delete se;
    }
}

TEST(ConditionTest, CollisionTest)
{
    double dt = 0.01;

    double timestamps[] = { 5.42, 5.43, 6.28, 6.29, 7.02, 8.67 };

    // Initialize the scenario and disable interactive controller
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/test-collision-detection.xosc", true);
    ASSERT_NE(se, nullptr);

    while (se->getSimulationTime() < timestamps[0] && se->GetQuitFlag() != true)
    {
        se->step(dt);
    }
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[1]), false);
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[2]), false);

    while (se->getSimulationTime() < timestamps[1] && se->GetQuitFlag() != true)
    {
        se->step(dt);
    }
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[1]), false);
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[2]), true);

    while (se->getSimulationTime() < timestamps[2] && se->GetQuitFlag() != true)
    {
        se->step(dt);
    }
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[1]), false);
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[2]), true);

    while (se->getSimulationTime() < timestamps[3] && se->GetQuitFlag() != true)
    {
        se->step(dt);
    }
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[1]), true);
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[2]), true);

    while (se->getSimulationTime() < timestamps[4] && se->GetQuitFlag() != true)
    {
        se->step(dt);
    }
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[1]), true);
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[2]), false);

    while (se->getSimulationTime() < timestamps[5] && se->GetQuitFlag() != true)
    {
        se->step(dt);
    }
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[1]), false);
    ASSERT_EQ(se->entities.object_[0]->Collision(se->entities.object_[2]), false);

    delete se;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}