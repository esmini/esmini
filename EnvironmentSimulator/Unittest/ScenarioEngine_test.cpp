#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <stdexcept>

#include "ScenarioEngine.hpp"
#include "ScenarioReader.hpp"
#include "ControllerUDPDriver.hpp"
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

TEST(DistanceTest, CalcEntityDistanceFreespace)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/straight_500m.xodr");
    OpenDrive* odr = Position::GetOpenDrive();

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    Object obj0(Object::Type::VEHICLE);
    obj0.boundingbox_.center_ = { 1.5, 0.0, 0.0 };
    obj0.boundingbox_.dimensions_ = { 2.0, 5.0, 2.0 };
    obj0.pos_.SetLanePos(1, -1, 20.0, 0);
    obj0.pos_.SetHeading(0.0);

    Object obj1(Object::Type::VEHICLE);
    obj1.boundingbox_.center_ = { 1.5, 0.0, 0.0 };
    obj1.boundingbox_.dimensions_ = { 2.0, 5.0, 2.0 };
    obj1.pos_.SetLanePos(1, -1, 30.0, 0);
    obj1.pos_.SetHeading(0.0);

    // Measure from X, Y point to object in cartesian coordinates
    double latDist = 0.0;
    double longDist = 0.0;
    double dist = 0.0;
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, 0, 0), false);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), false);
    EXPECT_NEAR(latDist, 0.0, 1e-5);
    EXPECT_NEAR(longDist, 5.0, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 5.0, 1e-3);

    obj1.pos_.SetLanePos(1, -1, 15.1, 1.9);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, 0, 0), true);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), true);
    EXPECT_NEAR(latDist, 0.0, 1e-5);
    EXPECT_NEAR(longDist, 0.0, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 0.0, 1e-3);

    obj1.pos_.SetLanePos(1, -1, 15.1, 2.9);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, 0, 0), false);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), false);
    EXPECT_NEAR(latDist, 0.9, 1e-5);
    EXPECT_NEAR(longDist, 0.0, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 0.9, 1e-3);

    obj1.pos_.SetLanePos(1, -1, 10.0, 0.0);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, 0, 0), false);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), false);
    EXPECT_NEAR(latDist, 0.0, 1e-5);
    EXPECT_NEAR(longDist, -5.0, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 5.0, 1e-3);

    obj1.pos_.SetLanePos(1, -1, 10.0, 0.0);
    obj1.pos_.SetHeadingRelative(M_PI);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, 0, 0), false);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), false);
    EXPECT_NEAR(latDist, 0.0, 1e-5);
    EXPECT_NEAR(longDist, -8.0, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 8.0, 1e-3);

    obj1.pos_.SetLanePos(1, -1, 30.0, 5.0);
    obj1.pos_.SetHeadingRelative(0.5);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, 0, 0), false);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), false);
    EXPECT_NEAR(latDist, 2.64299, 1e-5);
    EXPECT_NEAR(longDist, 4.64299, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 6.183198, 1e-3);

    obj1.pos_.SetHeadingRelative(-0.5);
    EXPECT_EQ(obj0.CollisionAndRelativeDistLatLong(&obj1, &latDist, &longDist), false);
    EXPECT_NEAR(latDist, 1.204715, 1e-5);
    EXPECT_NEAR(longDist, 4.64299, 1e-5);
    EXPECT_NEAR(dist = obj0.FreeSpaceDistance(&obj1, &latDist, &longDist), 5.876278, 1e-3);
}

TEST(TrajectoryTest, EnsureContinuation)
{
    double dt = 0.01;
    ScenarioEngine *se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/trajectory-continuity.xosc");
    ASSERT_NE(se, nullptr);

    for (int i = 0; i < (int)(1.0/dt); i++)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetX(), 4.95, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.535, 1e-5);

    for (int i = 0; i < (int)(2.0 / dt); i++)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetX(), 14.92759, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.18333, 1e-5);

    for (int i = 0; i < (int)(1.5 / dt); i++)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetX(), 21.32304, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetY(), 2.553967, 1e-5);

    for (int i = 0; i < (int)(1.0 / dt); i++)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetX(), 26.13539, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetY(), 2.917931, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetH(), 0.0, 1e-5);
}

TEST(ExpressionTest, EnsureResult)
{
    ASSERT_DOUBLE_EQ(eval_expr("1 + 1"), 2.0);
    ASSERT_DOUBLE_EQ(eval_expr("5 * 10 + 1"), 51.0);
    ASSERT_DOUBLE_EQ(eval_expr("5 * (10 + 1)"), 55.0);
    ASSERT_DOUBLE_EQ(eval_expr("15/3.5"), 15.0f / 3.5);
    ASSERT_DOUBLE_EQ(eval_expr("15 % 6"), 3.0);
    ASSERT_DOUBLE_EQ(eval_expr("-15 % 6"), -3.0);
    ASSERT_DOUBLE_EQ(eval_expr("1 == 1"), 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("1 == 2"), 0.0);
    ASSERT_DOUBLE_EQ(eval_expr("(4 == 4) && (10 == 10)"), 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("4 == 4 && 10 == 10"), 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("4 == 4 && 9 < 10"), 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("4 == 4 || 11 == 10"), 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("4 == 3 || 9 < 8"), 0.0);
    ASSERT_DOUBLE_EQ(eval_expr("ceil(11.1) == 12"), 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("round(11.1) == 11"), 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("floor(11.9) == 11"), 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("ceil(-11.1) == -11"), 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("round(-11.1) == -11"), 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("floor(-11.9) == -12"), 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("pow(2,3)"), 8.0);
    ASSERT_DOUBLE_EQ(eval_expr("2**3"), 8.0);
    ASSERT_DOUBLE_EQ(eval_expr("13.88888888888889 - 1.0"), 12.88888888888889);
    ASSERT_DOUBLE_EQ(eval_expr("13.88888888888889 - 0.0"), 13.88888888888889);
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
        "option2Value",
        "--option2",
        "option2Value2",
        "--option3",
        "--option4"
    };
    int argc = sizeof(args) / sizeof(char*);
    char** argv;
    argv = (char**)malloc(argc * sizeof(char*));
    for (int i = 0; i < argc; i++)
    {
        size_t len = strlen(args[i]);
        argv[i] = (char*)malloc((len + 1) * sizeof(char*));
        strncpy(argv[i], args[i], len + 1);
    }

    ASSERT_EQ(opt.ParseArgs(&argc, argv), -1);

    ASSERT_EQ(opt.GetOptionSet("no_arg"), false);
    ASSERT_EQ(opt.GetOptionSet("osc_file"), true);
    ASSERT_EQ(opt.GetOptionSet("window"), true);
    ASSERT_EQ(opt.GetOptionSet("option2"), true);
    ASSERT_EQ(opt.GetOptionSet("option3"), true);
    ASSERT_EQ(opt.GetOptionSet("option4"), false);
    ASSERT_EQ(opt.GetOptionSet("option5"), false);
    ASSERT_EQ(opt.GetOptionArg("window"), "");
    ASSERT_EQ(opt.GetOptionArg("osc_file"), "my_scenario.xosc");
    ASSERT_EQ(opt.GetOptionArg("odr_file"), "my_road.xodr");
    ASSERT_EQ(opt.GetOptionArg("option2"), "option2Value");
    ASSERT_EQ(opt.GetOptionArg("option2", 1), "option2Value2");
    ASSERT_EQ(opt.GetOptionArg("option3"), "55");

    // test without last argument, should return OK
    argc = sizeof(args-1) / sizeof(char*);
    ASSERT_EQ(opt.ParseArgs(&argc, argv), 0);

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

// Test junction selector functionality
// Utilizing fabriksgatan 4 way intersection
// Car will always drive on road 0, north towards the intersection
// 4 loops:
//   1. 270 degrees -> take right (road 1)
//   2. -90 degrees -> take right (road 1)
//   3. 0 degrees -> go straight (road 2)
//   4. 90 degrees -> go left (road 3)

TEST(JunctionTest, JunctionSelectorTest)
{
    double dt = 0.01;

    double angles[] = { 3 * M_PI_2, -M_PI_2, 0.0, M_PI_2 };
    int roadIds[] = { 1, 1, 2, 3 };
    double durations[] = { 2.5, 2.5, 2.6, 2.8 };  // Make sure car gets gets out of the intersection

    for (int i = 0; i < sizeof(angles) / sizeof(double); i++)
    {
        ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/junction-selector.xosc");
        se->step(0.0);
        se->prepareGroundTruth(0.0);
        ASSERT_NE(se, nullptr);

        se->entities_.object_[0]->SetJunctionSelectorStrategy(roadmanager::Junction::JunctionStrategyType::SELECTOR_ANGLE);
        se->entities_.object_[0]->SetJunctionSelectorAngle(angles[i]);
        while (se->getSimulationTime() < durations[i] && se->GetQuitFlag() != true)
        {
            se->step(dt);
            se->prepareGroundTruth(dt);
        }
        ASSERT_EQ(se->entities_.object_[0]->pos_.GetTrackId(), roadIds[i]);
        delete se;
    }
}

TEST(ConditionTest, CollisionTest)
{
    double dt = 0.01;
    double timestamps[] = { 5.24, 5.25, 6.25, 6.26, 7.10, 8.78 };

    ASSERT_EQ(SE_Env::Inst().GetCollisionDetection(), false);  // Should be disabled by default

    SE_Env::Inst().SetCollisionDetection(true);

    // Initialize the scenario and disable interactive controller
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/test-collision-detection.xosc", true);
    se->step(0.0);
    se->prepareGroundTruth(0.0);
    ASSERT_NE(se, nullptr);
    ASSERT_EQ(SE_Env::Inst().GetCollisionDetection(), true);  // Should be enabled by now

    while (se->getSimulationTime() < timestamps[0] - SMALL_NUMBER && se->GetQuitFlag() != true)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_EQ(se->entities_.object_[0]->collisions_.size(), 0);
    EXPECT_EQ(se->entities_.object_[1]->collisions_.size(), 0);
    EXPECT_EQ(se->entities_.object_[2]->collisions_.size(), 0);

    while (se->getSimulationTime() < timestamps[1] - SMALL_NUMBER && se->GetQuitFlag() != true)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_EQ(se->entities_.object_[0]->collisions_.size(), 1);
    EXPECT_EQ(se->entities_.object_[0]->collisions_[0], se->entities_.object_[2]);
    EXPECT_EQ(se->entities_.object_[1]->collisions_.size(), 0);
    EXPECT_EQ(se->entities_.object_[2]->collisions_.size(), 1);
    EXPECT_EQ(se->entities_.object_[2]->collisions_[0], se->entities_.object_[0]);

    while (se->getSimulationTime() < timestamps[2] - SMALL_NUMBER && se->GetQuitFlag() != true)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_EQ(se->entities_.object_[0]->collisions_.size(), 1);
    EXPECT_EQ(se->entities_.object_[0]->collisions_[0], se->entities_.object_[2]);
    EXPECT_EQ(se->entities_.object_[1]->collisions_.size(), 0);
    EXPECT_EQ(se->entities_.object_[2]->collisions_.size(), 1);
    EXPECT_EQ(se->entities_.object_[2]->collisions_[0], se->entities_.object_[0]);

    while (se->getSimulationTime() < timestamps[3] - SMALL_NUMBER && se->GetQuitFlag() != true)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_EQ(se->entities_.object_[0]->collisions_.size(), 2);
    EXPECT_EQ(se->entities_.object_[0]->collisions_[0], se->entities_.object_[2]);
    EXPECT_EQ(se->entities_.object_[0]->collisions_[1], se->entities_.object_[1]);
    EXPECT_EQ(se->entities_.object_[1]->collisions_.size(), 1);
    EXPECT_EQ(se->entities_.object_[1]->collisions_[0], se->entities_.object_[0]);
    EXPECT_EQ(se->entities_.object_[2]->collisions_.size(), 1);
    EXPECT_EQ(se->entities_.object_[2]->collisions_[0], se->entities_.object_[0]);

    while (se->getSimulationTime() < timestamps[4] - SMALL_NUMBER && se->GetQuitFlag() != true)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_EQ(se->entities_.object_[0]->Collision(se->entities_.object_[1]), true);
    EXPECT_EQ(se->entities_.object_[0]->Collision(se->entities_.object_[2]), false);
    EXPECT_EQ(se->entities_.object_[0]->collisions_.size(), 1);
    EXPECT_EQ(se->entities_.object_[0]->collisions_[0], se->entities_.object_[1]);
    EXPECT_EQ(se->entities_.object_[1]->collisions_.size(), 1);
    EXPECT_EQ(se->entities_.object_[1]->collisions_[0], se->entities_.object_[0]);
    EXPECT_EQ(se->entities_.object_[2]->collisions_.size(), 0);

    while (se->getSimulationTime() < timestamps[5] - SMALL_NUMBER && se->GetQuitFlag() != true)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_EQ(se->entities_.object_[0]->Collision(se->entities_.object_[1]), false);
    EXPECT_EQ(se->entities_.object_[0]->Collision(se->entities_.object_[2]), false);
    EXPECT_EQ(se->entities_.object_[0]->collisions_.size(), 0);
    EXPECT_EQ(se->entities_.object_[1]->collisions_.size(), 0);
    EXPECT_EQ(se->entities_.object_[2]->collisions_.size(), 0);

    delete se;
}

TEST(ControllerTest, UDPDriverModelTestAsynchronous)
{
    double dt = 0.01;

    ScenarioEngine* se = new ScenarioEngine("../../../scripts/udp_driver/two_cars_in_open_space.xosc");
    ASSERT_NE(se, nullptr);
    ASSERT_EQ(se->entities_.object_.size(), 2);

    // Replace controllers
    for (int i = 0; i < 2; i++)
    {
        scenarioengine::Controller::InitArgs args;
        args.name = "UDPDriverModel Controller";
        args.type = ControllerUDPDriver::GetTypeNameStatic();
        args.parameters = 0;
        args.gateway = se->getScenarioGateway();
        args.properties = new OSCProperties();
        OSCProperties::Property property;
        property.name_ = "port";
        property.value_ = std::to_string(0);
        args.properties->property_.push_back(property);
        property.name_ = "basePort";
        property.value_ = std::to_string(61900);
        args.properties->property_.push_back(property);
        property.name_ = "inputMode";
        property.value_ = "vehicleStateXYH";
        args.properties->property_.push_back(property);
        ControllerUDPDriver* controller = (ControllerUDPDriver*)InstantiateControllerUDPDriver(&args);

        delete se->entities_.object_[i]->controller_;
        delete args.properties;

        controller->Assign(se->entities_.object_[i]);
        se->scenarioReader->controller_[i] = controller;
        se->entities_.object_[i]->controller_ = controller;
    }

    // assign controllers
    se->step(dt);

    // stimulate driver input
    UDPClient* udpClient= new UDPClient(61900, "127.0.0.1");

    ControllerUDPDriver::DMMessage msg;

    msg.header.frameNumber = 0;
    msg.header.version = 1;
    msg.header.objectId = 0;
    msg.header.inputMode = static_cast<int>(ControllerUDPDriver::InputMode::VEHICLE_STATE_XYH);

    msg.message.stateXYH.x = 20.0;
    msg.message.stateXYH.y = 30.0;
    msg.message.stateXYH.h = 0.3;
    msg.message.stateXYH.speed = 30.0;
    msg.message.stateXYH.wheelAngle = 0.1;
    msg.message.stateXYH.deadReckon = 0;

    udpClient->Send((char*)&msg, sizeof(msg));

    // Make sure last message is applied
    msg.message.stateXYZHPR.y = 40;
    udpClient->Send((char*)&msg, sizeof(msg));

    // read messages and report updated states
    se->step(dt);
    // another step for scenarioengine to fetch and apply updated states
    se->step(dt);

    EXPECT_DOUBLE_EQ(se->entities_.object_[0]->pos_.GetY(), 40.0);

    // another step, make sure no dead-reckoning happen
    se->step(dt);
    EXPECT_DOUBLE_EQ(se->entities_.object_[0]->pos_.GetX(), 20.0);
    EXPECT_DOUBLE_EQ(se->entities_.object_[0]->pos_.GetY(), 40.0);

    // now, do not update position but enable dead reckoning
    msg.message.stateXYH.deadReckon = 1;
    udpClient->Send((char*)&msg, sizeof(msg));
    se->step(dt);
    se->step(dt);
    EXPECT_DOUBLE_EQ(se->entities_.object_[0]->pos_.GetX(), 20.0);
    se->step(dt);
    // Now, the dead reckoning should have kicked in
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 20.287, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), 40.089, 1E-3);

    delete se;
    delete udpClient;
}

TEST(ControllerTest, UDPDriverModelTestSynchronous)
{
    double dt = 0.01;

    ScenarioEngine* se = new ScenarioEngine("../../../scripts/udp_driver/two_cars_in_open_space.xosc");
    ASSERT_NE(se, nullptr);
    ASSERT_EQ(se->entities_.object_.size(), 2);

    // Replace controllers
    for (int i = 0; i < 2; i++)
    {
        scenarioengine::Controller::InitArgs args;
        args.name = "UDPDriverModel Controller";
        args.type = ControllerUDPDriver::GetTypeNameStatic();
        args.parameters = 0;
        args.gateway = se->getScenarioGateway();
        args.properties = new OSCProperties();
        OSCProperties::Property property;
        property.name_ = "execMode";
        property.value_ = "synchronous";
        args.properties->property_.push_back(property);
        property.name_ = "port";
        property.value_ = std::to_string(0);
        args.properties->property_.push_back(property);
        property.name_ = "basePort";
        property.value_ = std::to_string(61910);
        args.properties->property_.push_back(property);
        property.name_ = "inputMode";
        property.value_ = "vehicleStateXYZHPR";
        args.properties->property_.push_back(property);
        property.name_ = "timoutMs";
        property.value_ = std::to_string(500);
        args.properties->property_.push_back(property);
        ControllerUDPDriver* controller = (ControllerUDPDriver*)InstantiateControllerUDPDriver(&args);

        delete se->entities_.object_[i]->controller_;
        delete args.properties;

        controller->Assign(se->entities_.object_[i]);
        se->scenarioReader->controller_[i] = controller;
        se->entities_.object_[i]->controller_ = controller;
    }

    // assign controllers
    se->step(dt);

    // stimulate driver input
    UDPClient* udpClient = new UDPClient(61910, "127.0.0.1");

    ControllerUDPDriver::DMMessage msg;

    msg.header.frameNumber = 0;
    msg.header.version = 1;
    msg.header.objectId = 0;
    msg.header.inputMode = static_cast<int>(ControllerUDPDriver::InputMode::VEHICLE_STATE_XYZHPR);

    msg.message.stateXYZHPR.h = 0.3;
    msg.message.stateXYZHPR.x = 20.0;
    msg.message.stateXYZHPR.y = 30.0;
    msg.message.stateXYZHPR.deadReckon = 0;

    udpClient->Send((char*)&msg, sizeof(msg));

    // Put another message on queue of first vehicle
    msg.header.frameNumber++;
    msg.message.stateXYZHPR.y = 40;
    udpClient->Send((char*)&msg, sizeof(msg));

    // read messages and report updated states
    se->step(dt);

    // another step for scenarioengine to fetch and apply updated states
    se->step(dt);

    // In synchronous mode one message is consumed each time step
    // Expect the first message to be applied, the second has not
    // yet been processed
    EXPECT_DOUBLE_EQ(se->entities_.object_[0]->pos_.GetY(), 30.0);

    // another step for scenarioengine to fetch and apply the second message
    se->step(dt);
    EXPECT_DOUBLE_EQ(se->entities_.object_[0]->pos_.GetY(), 40.0);

    // second vehicle has not been updated (no message sent)
    se->step(dt);
    EXPECT_DOUBLE_EQ(se->entities_.object_[1]->pos_.GetY(), 6.5);

    // Create a sender for second vehicle as well
    UDPClient* udpClient2 = new UDPClient(61911, "127.0.0.1");

    msg.header.frameNumber = 0;
    msg.header.version = 1;
    msg.header.objectId = 1;
    msg.header.inputMode = static_cast<int>(ControllerUDPDriver::InputMode::VEHICLE_STATE_XYZHPR);

    msg.message.stateXYZHPR.h = 0.3;
    msg.message.stateXYZHPR.x = 90.0;
    msg.message.stateXYZHPR.y = -10.0;
    msg.message.stateXYZHPR.deadReckon = 0;

    udpClient2->Send((char*)&msg, sizeof(msg));

    msg.header.frameNumber = 2;
    msg.header.objectId = 0;
    msg.message.stateXYZHPR.x = 150.0;
    udpClient->Send((char*)&msg, sizeof(msg));

    se->step(dt);
    se->step(dt);
    EXPECT_DOUBLE_EQ(se->entities_.object_[0]->pos_.GetX(), 150.0);
    EXPECT_DOUBLE_EQ(se->entities_.object_[1]->pos_.GetX(), 90.0);
    EXPECT_DOUBLE_EQ(se->entities_.object_[1]->pos_.GetY(), -10.0);

    delete se;
    delete udpClient;
}

TEST(RoadOrientationTest, TestElevationPitchRoll)
{
    double dt = 0.1;

    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/elevations.xosc");
    ASSERT_NE(se, nullptr);
    ASSERT_EQ(se->entities_.object_.size(), 4);

    // Fast forward
    while (se->getSimulationTime() < (5.0 - SMALL_NUMBER))
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    // Check vehicle orientation
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetZ(), -0.568177, 1e-5);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetP(), 0.0, 1e-5);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetR(), 0.37917, 1e-5);

    // Fast forward
    while (se->getSimulationTime() < (6.0 - SMALL_NUMBER))
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    EXPECT_NEAR(se->entities_.object_[1]->pos_.GetZ(), 0.47815, 1e-5);
    EXPECT_NEAR(se->entities_.object_[1]->pos_.GetP(), 0.0, 1e-5);
    EXPECT_NEAR(se->entities_.object_[1]->pos_.GetR(), 5.96641, 1e-5);

    EXPECT_NEAR(se->entities_.object_[2]->pos_.GetZ(), 13.24676, 1e-5);
    EXPECT_NEAR(se->entities_.object_[2]->pos_.GetP(), 0.27808, 1e-5);
    EXPECT_NEAR(se->entities_.object_[2]->pos_.GetR(), 0, 1e-5);
}

TEST(ActionDynamicsTest, TestDynamicsTimeDimension)
{
    OSCPrivateAction::TransitionDynamics td;
    double p_target = 0.0;
    double v_start = 0.0;
    double v_target = 0.0;

    td.dimension_ = OSCPrivateAction::DynamicsDimension::TIME;
    td.shape_ = OSCPrivateAction::DynamicsShape::LINEAR;

    p_target = 10.0;
    v_start = 0.0;
    v_target = 100.0;
    td.SetParamTargetVal(p_target);
    td.SetStartVal(v_start);
    td.SetTargetVal(v_target);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(1.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (1.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));
    td.Step(4.0);  // to 5.0s
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (5.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));
    td.Step(5.0);  // to 10.0s
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (10.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));

    p_target = 5.0;
    v_start = 10.0;
    v_target = 50.0;
    td.Reset();
    td.SetParamTargetVal(p_target);
    td.SetStartVal(v_start);
    td.SetTargetVal(v_target);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(1.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (1.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));
    td.Step(2.0);  // to 3.0s
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (3.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));
    td.Step(2.0);  // to 5.0s
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (5.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));

    p_target = 8.0;
    v_start = 10.0;
    v_target = -50.0;
    td.Reset();
    td.SetParamTargetVal(p_target);
    td.SetStartVal(v_start);
    td.SetTargetVal(v_target);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(1.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (1.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));
    td.Step(4.0);  // to 5.0s
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (5.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));
    td.Step(3.0);  // to 8.0s
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (8.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));

    td.dimension_ = OSCPrivateAction::DynamicsDimension::TIME;
    td.shape_ = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
    td.Reset();
    p_target = 10.0;
    v_start = 0.0;
    v_target = 100.0;
    td.SetParamTargetVal(p_target);
    td.SetStartVal(v_start);
    td.SetTargetVal(v_target);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(1.0);
    EXPECT_NEAR(td.Evaluate(), 2.44717, 1e-5);
    td.Step(4.0);  // to 5.0s
    EXPECT_NEAR(td.Evaluate(), 50.00000, 1e-5);
    td.Step(2.0);  // to 7.0s
    EXPECT_NEAR(td.Evaluate(), 79.38926, 1e-5);
    td.Step(3.0);  // to 10.0s
    EXPECT_NEAR(td.Evaluate(), 100.00000, 1e-5);

    td.dimension_ = OSCPrivateAction::DynamicsDimension::TIME;
    td.shape_ = OSCPrivateAction::DynamicsShape::CUBIC;
    td.Reset();
    p_target = 10.0;
    v_start = 110.0;
    v_target = 10.0;
    td.SetParamTargetVal(p_target);
    td.SetStartVal(v_start);
    td.SetTargetVal(v_target);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(1.0);
    EXPECT_NEAR(td.Evaluate(), 107.20000, 1e-5);
    td.Step(4.0);  // to 5.0s
    EXPECT_NEAR(td.Evaluate(), 60.00000, 1e-5);
    td.Step(2.0);  // to 7.0s
    EXPECT_NEAR(td.Evaluate(), 31.60000, 1e-5);
    td.Step(3.0);  // to 10.0s
    EXPECT_NEAR(td.Evaluate(), 10.00000, 1e-5);
}

TEST(ActionDynamicsTest, TestDynamicsDistanceDimension)
{
    OSCPrivateAction::TransitionDynamics td;
    double p_target = 0.0;
    double v_start = 0.0;
    double v_target = 0.0;

    td.dimension_ = OSCPrivateAction::DynamicsDimension::DISTANCE;
    td.shape_ = OSCPrivateAction::DynamicsShape::LINEAR;

    p_target = 100.0;
    v_start = 0.0;
    v_target = 100.0;
    td.SetParamTargetVal(p_target);
    td.SetStartVal(v_start);
    td.SetTargetVal(v_target);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(10.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (10.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));
    td.Step(40.0);  // to 50.0m
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (50.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));
    td.Step(50.0);  // to 100.0m
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (100.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));

    p_target = 50.0;
    v_start = 10.0;
    v_target = 50.0;
    td.Reset();
    td.SetParamTargetVal(p_target);
    td.SetStartVal(v_start);
    td.SetTargetVal(v_target);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(10.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (10.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));
    td.Step(20.0);  // to 30.0m
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (30.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));
    td.Step(20.0);  // to 50.0m
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (50.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));

    p_target = 80.0;
    v_start = 10.0;
    v_target = -50.0;
    td.Reset();
    td.SetParamTargetVal(p_target);
    td.SetStartVal(v_start);
    td.SetTargetVal(v_target);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(10.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (10.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));
    td.Step(40.0);  // to 5.0m
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (50.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));
    td.Step(30.0);  // to 8.0m
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal() + (80.0 / td.GetParamTargetVal()) * (td.GetTargetVal() - td.GetStartVal()));

    td.dimension_ = OSCPrivateAction::DynamicsDimension::DISTANCE;
    td.shape_ = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
    td.Reset();
    td.SetParamTargetVal(100.0);
    td.SetStartVal(0.0);
    td.SetTargetVal(100.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(10.0);
    EXPECT_NEAR(td.Evaluate(), 2.44717, 1e-5);
    td.Step(40.0);  // to 50.0
    EXPECT_NEAR(td.Evaluate(), 50.00000, 1e-5);
    td.Step(20.0);  // to 70.0m
    EXPECT_NEAR(td.Evaluate(), 79.38926, 1e-5);
    td.Step(30.0);  // to 100.0m
    EXPECT_NEAR(td.Evaluate(), 100.00000, 1e-5);

    td.dimension_ = OSCPrivateAction::DynamicsDimension::DISTANCE;
    td.shape_ = OSCPrivateAction::DynamicsShape::CUBIC;
    td.Reset();
    td.SetParamTargetVal(100.0);
    td.SetStartVal(110.0);
    td.SetTargetVal(10.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(10.0);
    EXPECT_NEAR(td.Evaluate(), 107.20000, 1e-5);
    td.Step(40.0);  // to 50.0m
    EXPECT_NEAR(td.Evaluate(), 60.00000, 1e-5);
    td.Step(20.0);  // to 70.0m
    EXPECT_NEAR(td.Evaluate(), 31.60000, 1e-5);
    td.Step(30.0);  // to 100.0m
    EXPECT_NEAR(td.Evaluate(), 10.00000, 1e-5);
}

TEST(ActionDynamicsTest, TestDynamicsRateDimension)
{
    OSCPrivateAction::TransitionDynamics td;
    td.dimension_ = OSCPrivateAction::DynamicsDimension::RATE;
    td.shape_ = OSCPrivateAction::DynamicsShape::LINEAR;

    td.SetParamTargetVal(10.0);
    td.SetStartVal(0.0);
    td.SetTargetVal(100.0);
    td.SetRate(2.0);
    EXPECT_DOUBLE_EQ(50.0, td.GetParamTargetVal());  // param target is overriden by SetRate()
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(1.0);
    EXPECT_NEAR(td.Evaluate(), 2.00000, 1e-5);
    td.Step(4.0);  // to 5.0s
    EXPECT_NEAR(td.Evaluate(), 10.00000, 1e-5);
    td.Step(5.0);  // to 10.0s
    EXPECT_NEAR(td.Evaluate(), 20.00000, 1e-5);

    td.Reset();
    td.SetStartVal(10.0);
    td.SetTargetVal(50.0);
    td.SetRate(2.0);
    EXPECT_DOUBLE_EQ(td.GetStartVal(), 10.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), 10.0);
    td.Step(1.0);
    EXPECT_NEAR(td.Evaluate(), 12.00000, 1e-5);
    td.Step(2.0);  // to 3.0s
    EXPECT_NEAR(td.Evaluate(), 16.00000, 1e-5);
    td.Step(2.0);  // to 5.0s
    EXPECT_NEAR(td.Evaluate(), 20.00000, 1e-5);

    td.Reset();
    td.SetStartVal(10.0);
    td.SetTargetVal(50.0);
    EXPECT_DOUBLE_EQ(td.GetStartVal(), 10.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), 10.0);
    td.SetRate(-2.0);  // sign of rate should not matter
    td.Step(1.0);
    EXPECT_NEAR(td.Evaluate(), 12.00000, 1e-5);
    td.Step(2.0);  // to 3.0s
    EXPECT_NEAR(td.Evaluate(), 16.00000, 1e-5);
    td.Step(2.0);  // to 5.0s
    EXPECT_NEAR(td.Evaluate(), 20.00000, 1e-5);

    td.Reset();
    td.SetStartVal(10.0);
    td.SetTargetVal(-50.0);
    td.SetRate(5.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(10.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), -40.0);
    td.Step(40.0);  // to 50.0s
    EXPECT_DOUBLE_EQ(td.Evaluate(), -240.0);
    td.Step(30.0);  // to 80.0s
    EXPECT_DOUBLE_EQ(td.Evaluate(), -390.0);

    td.dimension_ = OSCPrivateAction::DynamicsDimension::RATE;
    td.shape_ = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
    td.Reset();
    td.SetStartVal(0.0);
    td.SetTargetVal(100.0);
    td.SetRate(5.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(5.0);
    EXPECT_NEAR(td.Evaluate(), 6.12087, 1e-5);
    td.Step(10.0);
    EXPECT_NEAR(td.Evaluate(), 46.46314, 1e-5);

    td.dimension_ = OSCPrivateAction::DynamicsDimension::RATE;
    td.shape_ = OSCPrivateAction::DynamicsShape::CUBIC;
    td.Reset();
    td.SetStartVal(0.0);
    td.SetTargetVal(100.0);
    td.SetRate(5.0);
    EXPECT_DOUBLE_EQ(td.Evaluate(), td.GetStartVal());
    td.Step(5.0);
    EXPECT_NEAR(td.Evaluate(), 7.40741, 1e-5);
    td.Step(10.0);
    EXPECT_NEAR(td.Evaluate(), 50.0000, 1e-5);
}

TEST(OrientationTest, TestRelativeRoadHeading)
{
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/four_roads.xosc");
    ASSERT_NE(se, nullptr);

    se->step(0.1);
    se->prepareGroundTruth(0.1);

    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetX(), 5.000, 1e-3);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetY(), 1.535, 1e-3);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetH(), 0.000, 1e-3);

    ASSERT_NEAR(se->entities_.object_[1]->pos_.GetX(), -1.531, 1e-3);
    ASSERT_NEAR(se->entities_.object_[1]->pos_.GetY(), -24.999, 1e-3);
    ASSERT_NEAR(se->entities_.object_[1]->pos_.GetH(), 1.570, 1e-3);

    ASSERT_NEAR(se->entities_.object_[2]->pos_.GetX(), -5.000, 1e-3);
    ASSERT_NEAR(se->entities_.object_[2]->pos_.GetY(), -41.535, 1e-3);
    ASSERT_NEAR(se->entities_.object_[2]->pos_.GetH(), 3.142, 1e-3);

    ASSERT_NEAR(se->entities_.object_[3]->pos_.GetX(), 1.539, 1e-3);
    ASSERT_NEAR(se->entities_.object_[3]->pos_.GetY(), -54.999, 1e-3);
    ASSERT_NEAR(se->entities_.object_[3]->pos_.GetH(), 4.713, 1e-3);
}

TEST(SpeedProfileTest, TestSpeedProfileFirstEntryOffset)
{
    LongSpeedProfileAction sp_action;
    DynamicConstraints dynamics; // initalized with default values
    LongSpeedProfileAction::Entry entry;

    Object obj(Object::Type::VEHICLE);
    obj.SetSpeed(10.0);
    double sim_time = 0.0, dt = 0.0;

    sp_action.following_mode_ = FollowingMode::POSITION;
    sp_action.dynamics_ = dynamics;
    sp_action.object_ = &obj;

    ASSERT_EQ(sp_action.entity_ref_, nullptr);
    ASSERT_EQ(sp_action.segment_.size(), 0);

    // Add entries
    entry.speed_ = 4.0;
    entry.time_ = 2.0;
    sp_action.AddEntry(entry);

    sp_action.Start(0.0, 0.1);

    // Evaluate at a time before first entry time, speed should interpolate towards first entry
    sp_action.Step(1.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 7.30, 1E-5);
    sim_time += dt;
}

TEST(SpeedProfileTest, TestSpeedProfileLinear)
{
    LongSpeedProfileAction sp_action;
    DynamicConstraints dynamics; // initalized with default values
    LongSpeedProfileAction::Entry entry;

    Object obj(Object::Type::VEHICLE);
    obj.SetSpeed(10.0);
    double sim_time = 5.0;

    sp_action.following_mode_ = FollowingMode::POSITION;
    sp_action.dynamics_ = dynamics;
    sp_action.object_ = &obj;

    ASSERT_EQ(sp_action.entity_ref_, nullptr);
    ASSERT_EQ(sp_action.entry_.size(), 0);

    // One entry at time = 10
    sp_action.AddEntry(LongSpeedProfileAction::Entry(10, 0.0));
    sp_action.Start(sim_time);
    EXPECT_NEAR(sp_action.GetSpeed(), 10.0, 1E-5);
    sp_action.Step(sim_time);
    EXPECT_NEAR(sp_action.GetSpeed(), 10.0, 1E-5);
    sp_action.Step(8.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 7.0, 1E-5);
    sp_action.Step(14.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 1.0, 1E-5);
    sp_action.Step(17.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 0.0, 1E-5);

    // Add another entry
    sp_action.AddEntry(LongSpeedProfileAction::Entry(5, 30.0));
    sp_action.Start(5.0);
    sp_action.Step(17.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 12.0, 1E-5);
    sp_action.Step(20.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 30.0, 1E-5);
    sp_action.Step(25.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 30.0, 1E-5);

    // Duplicate entry time
    sp_action.object_->SetSpeed(10);
    sp_action.AddEntry(LongSpeedProfileAction::Entry(0, 40.0));  // 15, 40
    sp_action.Start(0.0);
    sp_action.Step(14.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 24.0, 1E-5);
    sp_action.Step(15.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 40.0, 1E-5);
    sp_action.Step(16.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 40.0, 1E-5);
}

TEST(SpeedProfileTest, TestSpeedProfileConstraints)
{
    LongSpeedProfileAction sp_action;
    LongSpeedProfileAction::Entry entry;
    DynamicConstraints dynamics;

    dynamics.max_acceleration_ = 4.0;
    dynamics.max_acceleration_rate_ = 1.0;
    dynamics.max_deceleration_ =5.0;
    dynamics.max_deceleration_rate_ = 2.0;
    dynamics.max_speed_ = 30.0;

    Object obj(Object::Type::VEHICLE);
    obj.SetSpeed(10.0);

    sp_action.following_mode_ = FollowingMode::POSITION;
    sp_action.dynamics_ = dynamics;
    sp_action.object_ = &obj;

    ASSERT_EQ(sp_action.entity_ref_, nullptr);
    ASSERT_EQ(sp_action.entry_.size(), 0);

    // Add some entries
    sp_action.AddEntry(LongSpeedProfileAction::Entry(0.0, 0.0));
    sp_action.AddEntry(LongSpeedProfileAction::Entry(10.0, 10.0));
    sp_action.AddEntry(LongSpeedProfileAction::Entry(10.0, 5.0));
    sp_action.Start(0.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 10.0, 1E-5);
    sp_action.Step(0.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 0.0, 1E-5);
    sp_action.Step(1.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 1.0, 1E-5);
    sp_action.Step(11.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 9.5, 1E-5);

    obj.SetSpeed(10.0);
    sp_action.following_mode_ = FollowingMode::FOLLOW;
    sp_action.Start(0.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 10.0, 1E-5);
    sp_action.Step(0.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 10.0, 1E-5);
    sp_action.Step(9.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 10.0, 1E-5);
    sp_action.Step(11.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 9.5, 1E-5);
}

TEST(SpeedProfileTest, TestSpeedProfileSingleEntry)
{
    LongSpeedProfileAction sp_action;
    LongSpeedProfileAction::Entry entry;

    sp_action.dynamics_.max_acceleration_ = 4.0;
    sp_action.dynamics_.max_acceleration_rate_ = 1.0;
    sp_action.dynamics_.max_deceleration_ = 5.0;
    sp_action.dynamics_.max_deceleration_rate_ = 2.0;
    sp_action.dynamics_.max_speed_ = 30.0;

    Object obj(Object::Type::VEHICLE);
    obj.SetSpeed(1.0);

    sp_action.following_mode_ = FollowingMode::FOLLOW;
    sp_action.object_ = &obj;

    ASSERT_EQ(sp_action.entity_ref_, nullptr);
    ASSERT_EQ(sp_action.entry_.size(), 0);

    // Add some entries
    sp_action.AddEntry(LongSpeedProfileAction::Entry(10.0, 20.0));
    sp_action.Start(0.0);
    sp_action.Step(0.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 1.0, 1E-5);
    sp_action.Step(9.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 19.0, 1E-5);
    sp_action.Step(12.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 20.0, 1E-5);

    sp_action.dynamics_.max_acceleration_ = 1.0;
    obj.SetSpeed(1.0);
    sp_action.Start(0.0);
    sp_action.Step(0.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 1.0, 1E-5);
    sp_action.Step(9.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 9.5, 1E-5);
    sp_action.Step(17.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 17.5, 1E-5);
    sp_action.Step(20.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 20.0, 1E-5);

    sp_action.entry_[0].speed_ = 0.0;

    obj.SetSpeed(19.0);
    sp_action.Start(0.0);
    sp_action.Step(0.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 19.0, 1E-5);
    sp_action.Step(4.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 11.13664, 1E-5);
    sp_action.Step(8.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 2.0, 1E-5);
    sp_action.Step(10.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 0.0, 1E-5);
}

TEST(SpeedProfileTest, TestSpeedProfileNoTime)
{
    LongSpeedProfileAction sp_action;
    LongSpeedProfileAction::Entry entry;

    Object obj(Object::Type::VEHICLE);

    sp_action.object_ = &obj;
    ASSERT_EQ(sp_action.entity_ref_, nullptr);
    ASSERT_EQ(sp_action.entry_.size(), 0);
    sp_action.dynamics_.max_acceleration_ = 5.0;
    sp_action.dynamics_.max_deceleration_ = 10.0;
    sp_action.dynamics_.max_acceleration_rate_ = 5.0;
    sp_action.dynamics_.max_deceleration_rate_ = 5.0;
    sp_action.dynamics_.max_speed_ = 30.0;

    // Add only one entry
    sp_action.AddEntry(LongSpeedProfileAction::Entry(-1.0, 10.0));

    sp_action.following_mode_ = FollowingMode::POSITION;
    obj.SetSpeed(0.0);
    sp_action.Start(0.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 0.0, 1E-5);
    sp_action.Step(2.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 10.0, 1E-5);

    sp_action.following_mode_ = FollowingMode::FOLLOW;
    obj.SetSpeed(0.0);
    sp_action.Start(0.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 0.0, 1E-5);
    sp_action.Step(2.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 7.5, 1E-5);

    // Two entries
    sp_action.entry_.clear();
    sp_action.AddEntry(LongSpeedProfileAction::Entry(0.0, 0.0));
    sp_action.AddEntry(LongSpeedProfileAction::Entry(-1.0, 10.0));

    obj.SetSpeed(0.0);
    sp_action.following_mode_ = FollowingMode::FOLLOW;
    sp_action.Start(0.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 0.0, 1E-5);
    sp_action.Step(2.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 7.5, 1E-5);
    sp_action.Step(4.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 10.0, 1E-5);
}

TEST(SpeedProfileTest, TestSpeedProfileFromNonZeroTime)
{
    LongSpeedProfileAction sp_action;
    LongSpeedProfileAction::Entry entry;

    Object obj(Object::Type::VEHICLE);

    sp_action.object_ = &obj;
    ASSERT_EQ(sp_action.entity_ref_, nullptr);
    ASSERT_EQ(sp_action.entry_.size(), 0);
    sp_action.dynamics_.max_acceleration_ = 4.0;
    sp_action.dynamics_.max_deceleration_ = 10.0;
    sp_action.dynamics_.max_acceleration_rate_ = 5.0;
    sp_action.dynamics_.max_deceleration_rate_ = 4.0;
    sp_action.dynamics_.max_speed_ = 30.0;

    // Add some entries
    sp_action.AddEntry(LongSpeedProfileAction::Entry(0.0, 0.0));
    sp_action.AddEntry(LongSpeedProfileAction::Entry(4.0, 6.0));

    for (int i = 0; i < 2; i++)
    {
        if (i == 0)
        {
            sp_action.following_mode_ = FollowingMode::POSITION;
        }
        else
        {
            sp_action.following_mode_ = FollowingMode::FOLLOW;
        }

        obj.SetSpeed(0.0);

        sp_action.Start(3.0);
        EXPECT_NEAR(sp_action.GetSpeed(), 0.0, 1E-5);
        sp_action.Step(3.2);
        EXPECT_NEAR(sp_action.GetSpeed(), i == 0 ? 0.3 : 0.1, 1E-5);
        sp_action.Step(4.0);
        EXPECT_NEAR(sp_action.GetSpeed(), i == 0 ? 1.5 : 1.38033, 1E-5);
        sp_action.Step(5.0);
        EXPECT_NEAR(sp_action.GetSpeed(), i == 0 ? 3.0 : 3.03419, 1E-5);
        sp_action.Step(6.0);
        EXPECT_NEAR(sp_action.GetSpeed(), i == 0 ? 4.5 : 4.68805, 1E-5);
        sp_action.Step(8.0);
        EXPECT_NEAR(sp_action.GetSpeed(), 6.0, 1E-5);
    }
}

TEST(SpeedProfileTest, TestSpeedProfileNonZeroInitalAcc)
{
    LongSpeedProfileAction sp_action;
    DynamicConstraints dynamics; // initalized with default values
    LongSpeedProfileAction::Entry entry;

    Object obj(Object::Type::VEHICLE);
    obj.SetSpeed(10.0);
    obj.SetAcc(1.0, 0.0, 0.0);

    sp_action.following_mode_ = FollowingMode::FOLLOW;
    sp_action.dynamics_ = dynamics;
    sp_action.object_ = &obj;

    sp_action.dynamics_.max_acceleration_ = 4.0;
    sp_action.dynamics_.max_deceleration_ = 10.0;
    sp_action.dynamics_.max_acceleration_rate_ = 5.0;
    sp_action.dynamics_.max_deceleration_rate_ = 4.0;
    sp_action.dynamics_.max_speed_ = 30.0;


    ASSERT_EQ(sp_action.entity_ref_, nullptr);
    ASSERT_EQ(sp_action.segment_.size(), 0);

    // Add entries
    entry.speed_ = 4.0;
    entry.time_ = 5.0;
    sp_action.AddEntry(entry);

    sp_action.Start(0.0, 0.1);

    // Evaluate small time step ahead. Although the speed profile entry slope is negative,
    // the speed is still increasing due to inital positive acceleration.
    sp_action.Step(0.4);
    EXPECT_NEAR(sp_action.GetSpeed(), 10.12, 1E-5);

    // Evaluate at end of the speed profile. Acceleration should approach target value.
    sp_action.Step(5.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 4.025, 1E-3);

    // Evaluate slightly past end of the speed profile. Acceleration should have reached the target value.
    sp_action.Step(5.2);
    EXPECT_NEAR(sp_action.GetSpeed(), 4.0, 1E-3);
}

// Uncomment to print log output to console
//#define LOG_TO_CONSOLE

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

    //testing::GTEST_FLAG(filter) = "*UDPDriverModelTestAsynchronous";

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
