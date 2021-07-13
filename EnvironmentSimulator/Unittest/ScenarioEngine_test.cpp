#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <stdexcept>

#include "ScenarioEngine.hpp"
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
    ASSERT_FLOAT_EQ(eval_expr("15/3.5"), 15.0f/3.5f);
    ASSERT_FLOAT_EQ(eval_expr("15 % 6"), 3.0f);
    ASSERT_FLOAT_EQ(eval_expr("-15 % 6"), -3.0f);
    ASSERT_FLOAT_EQ(eval_expr("1 == 1"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("1 == 2"), 0.0f);
    ASSERT_FLOAT_EQ(eval_expr("(4 == 4) && (10 == 10)"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("4 == 4 && 10 == 10"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("4 == 4 && 9 < 10"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("ceil(11.1) == 12"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("round(11.1) == 11"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("floor(11.9) == 11"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("ceil(-11.1) == -11"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("round(-11.1) == -11"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("floor(-11.9) == -12"), 1.0f);
    ASSERT_FLOAT_EQ(eval_expr("pow(2,3)"), 8.0f);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}