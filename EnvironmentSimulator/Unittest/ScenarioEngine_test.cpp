#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <stdexcept>
#include <array>

#include "CommonMini.hpp"
#include "ScenarioEngine.hpp"
#include "ScenarioReader.hpp"
#include "ControllerUDPDriver.hpp"
#include "ControllerLooming.hpp"
#include "ControllerALKS_R157SM.hpp"
#include "ControllerInteractive.hpp"
#include "OSCParameterDistribution.hpp"
#include "pugixml.hpp"
#include "simple_expr.h"
#include "TestHelper.hpp"

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
    Vehicle obj0;
    obj0.boundingbox_.center_     = {1.0, 0.0, 0.0};
    obj0.boundingbox_.dimensions_ = {2.0, 2.0, 2.0};
    obj0.pos_                     = pos0;

    Vehicle obj1;
    obj1.pos_                     = pos1;
    obj1.boundingbox_.center_     = {1.0, 0.0, 0.0};
    obj1.boundingbox_.dimensions_ = {2.0, 2.0, 2.0};

    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist), 0);
    EXPECT_NEAR(dist, 8.0, 1e-5);

    // Modify boundingbox center
    obj1.boundingbox_.center_ = {1.5, 0.0, 0.0};
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
    obj0.pos_.SetMode(Position::PosModeType::UPDATE,
                      Position::PosMode::Z_REL | Position::PosMode::H_REL | Position::PosMode::P_REL | Position::PosMode::R_REL);
    obj0.pos_.SetLanePos(0, -1, 549.0, 0.0);
    obj0.pos_.SetMode(Position::PosModeType::UPDATE,
                      Position::PosMode::Z_REL | Position::PosMode::H_REL | Position::PosMode::P_REL | Position::PosMode::R_REL);
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
    EXPECT_NEAR(dist, -2.5, 1e-5);
    ASSERT_EQ(obj0.Distance(&obj1, CoordinateSystem::CS_ROAD, RelativeDistanceType::REL_DIST_LATERAL, true, dist), 0);
    EXPECT_NEAR(dist, -0.5, 1e-5);
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

    Vehicle obj0;
    obj0.boundingbox_.center_     = {1.0, 0.0, 0.0};
    obj0.boundingbox_.dimensions_ = {2.0, 2.0, 2.0};
    obj0.pos_                     = pos0;

    // Measure from X, Y point to object in road coordinates
    double latDist  = 0.0;
    double longDist = 0.0;

    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos0.GetX() + 20.0, pos0.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, 18.0, 1e-5);

    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos0.GetX() - 20.0, pos0.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, -20.0, 1e-5);

    obj0.boundingbox_.dimensions_ = {2.0, 5.0, 2.0};
    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos0.GetX() - 20.0, pos0.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, -18.5, 1e-5);

    obj0.boundingbox_.center_ = {2.0, 4.0, 0.0};
    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos0.GetX() - 20.0, pos0.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, -19.5, 1e-5);

    obj0.boundingbox_.center_ = {2.0, 4.0, 0.0};
    Position tmpPos(0, 600, obj0.pos_.GetT());
    EXPECT_NEAR(tmpPos.GetX(), 585.438756, 1e-5);
    EXPECT_NEAR(tmpPos.GetY(), 45.140405, 1e-5);
    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(tmpPos.GetX(), tmpPos.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, 195.5, 1e-5);
    EXPECT_NEAR(latDist, -3.0, 1e-5);
}

TEST(DistanceTest, CalcDistancePointAcrossIntersection)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/fabriksgatan.xodr");
    OpenDrive* odr = Position::GetOpenDrive();

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 16);

    Position pos0 = Position(0, 1, 10.0, 0);
    pos0.SetHeading(0.0);

    Vehicle obj0;
    obj0.boundingbox_.center_     = {1.0, 0.0, 0.0};
    obj0.boundingbox_.dimensions_ = {2.0, 2.0, 2.0};
    obj0.pos_                     = pos0;

    // another point some meters ahead, still on the same straight segment
    Position pos1 = Position(2, 1, 290.0, 0);

    // Measure from X, Y point to object in road coordinates
    double latDist  = 0.0;
    double longDist = 0.0;

    ASSERT_EQ(obj0.FreeSpaceDistancePointRoadLane(pos1.GetX(), pos1.GetY(), &latDist, &longDist, CoordinateSystem::CS_ROAD), 0);
    EXPECT_NEAR(longDist, -38.58642, 1e-5);
    EXPECT_NEAR(latDist, -0.22127, 1e-5);
}

TEST(DistanceTest, CalcEntityDistanceFreespace)
{
    Position::GetOpenDrive()->LoadOpenDriveFile("../../../resources/xodr/straight_500m.xodr");
    OpenDrive* odr = Position::GetOpenDrive();

    ASSERT_NE(odr, nullptr);
    EXPECT_EQ(odr->GetNumOfRoads(), 1);

    Vehicle obj0;
    obj0.boundingbox_.center_     = {1.5, 0.0, 0.0};
    obj0.boundingbox_.dimensions_ = {2.0, 5.0, 2.0};
    obj0.pos_.SetLanePos(1, -1, 20.0, 0);
    obj0.pos_.SetHeading(0.0);

    Vehicle obj1;
    obj1.boundingbox_.center_     = {1.5, 0.0, 0.0};
    obj1.boundingbox_.dimensions_ = {2.0, 5.0, 2.0};
    obj1.pos_.SetLanePos(1, -1, 30.0, 0);
    obj1.pos_.SetHeading(0.0);

    // Measure from X, Y point to object in cartesian coordinates
    double latDist  = 0.0;
    double longDist = 0.0;
    double dist     = 0.0;
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

TEST(DistanceTest, DistanceWithTrailers)
{
    double dt       = 0.1;
    double distance = LARGE_NUMBER;

    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/distance_with_trailers.xosc");

    ASSERT_NE(se, nullptr);
    EXPECT_EQ(se->entities_.object_.size(), 7);

    // initialize
    se->step(0.0);
    se->prepareGroundTruth(0.0);

    ASSERT_EQ(se->entities_.object_[2]->GetType(), Object::Type::VEHICLE);
    ASSERT_EQ(se->entities_.object_[0]->GetType(), Object::Type::VEHICLE);
    ASSERT_EQ(se->entities_.object_[6]->GetType(), Object::Type::VEHICLE);
    ASSERT_EQ(se->entities_.object_[3]->GetType(), Object::Type::VEHICLE);

    Vehicle* car1            = static_cast<Vehicle*>(se->entities_.object_[2]);
    Vehicle* truck1          = static_cast<Vehicle*>(se->entities_.object_[0]);
    Vehicle* car2            = static_cast<Vehicle*>(se->entities_.object_[6]);
    Vehicle* truck2          = static_cast<Vehicle*>(se->entities_.object_[3]);
    Vehicle* truck2_trailer2 = static_cast<Vehicle*>(se->entities_.object_[5]);

    // first check some reference point distance variants

    // euclidian reference point distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, false, distance);
    EXPECT_NEAR(distance, 68.887, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, false, distance);
    EXPECT_NEAR(distance, 68.887, 1e-3);

    // longitudinal reference point distance, in entity coordinate system, between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, false, distance);
    EXPECT_NEAR(distance, 66.230, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, false, distance);
    EXPECT_NEAR(distance, 67.207, 1e-3);

    // longitudinal road reference point distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ROAD, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, false, distance);
    EXPECT_NEAR(distance, 70.0, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ROAD, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, false, distance);
    EXPECT_NEAR(distance, 70.0, 1e-3);

    // longitudinal lane reference point distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_LANE, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, false, distance);
    EXPECT_NEAR(distance, 70.0, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_LANE, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, false, distance);
    EXPECT_NEAR(distance, 70.0, 1e-3);

    // lateral road reference point distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ROAD, roadmanager::RelativeDistanceType::REL_DIST_LATERAL, false, distance);
    EXPECT_NEAR(distance, -3.07, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ROAD, roadmanager::RelativeDistanceType::REL_DIST_LATERAL, false, distance);
    EXPECT_NEAR(distance, 3.07, 1e-3);

    // lateral lane reference point distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_LANE, roadmanager::RelativeDistanceType::REL_DIST_LATERAL, false, distance);
    EXPECT_NEAR(distance, -3.07, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_LANE, roadmanager::RelativeDistanceType::REL_DIST_LATERAL, false, distance);
    EXPECT_NEAR(distance, 3.07, 1e-3);

    // then check some freespace variants

    // euclidian freespace distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 59.6319, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 59.6319, 1e-3);

    // longitudinal freespace distance, in entity coordinate system, between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 57.2550, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 58.2370, 1e-3);

    // longitudinal road freespace distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ROAD, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 60.9302, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ROAD, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 60.9302, 1e-3);

    // longitudinal lane freespace distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_LANE, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 60.9302, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_LANE, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 60.9302, 1e-3);

    // lateral road freespace distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ROAD, roadmanager::RelativeDistanceType::REL_DIST_LATERAL, true, distance);
    EXPECT_NEAR(distance, -0.692, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ROAD, roadmanager::RelativeDistanceType::REL_DIST_LATERAL, true, distance);
    EXPECT_NEAR(distance, 0.692, 1e-3);

    // lateral lane freespace distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_LANE, roadmanager::RelativeDistanceType::REL_DIST_LATERAL, true, distance);
    EXPECT_NEAR(distance, -0.692, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_LANE, roadmanager::RelativeDistanceType::REL_DIST_LATERAL, true, distance);
    EXPECT_NEAR(distance, 0.692, 1e-3);

    // move forward in time a bit
    while (se->getSimulationTime() < 1.2 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    EXPECT_NEAR(car1->pos_.GetX(), 513.193, 1e-3);
    EXPECT_NEAR(car1->pos_.GetY(), 2.423, 1e-3);

    // euclidian freespace distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 24.3254, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 24.3254, 1e-3);

    // longitudinal freespace distance, in entity coordinate system, between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 24.3250, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 24.1240, 1e-3);

    // move forward until car1 is very close to truck1
    while (se->getSimulationTime() < 2.0 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    EXPECT_NEAR(car1->pos_.GetX(), 489.205, 1e-3);
    EXPECT_NEAR(car1->pos_.GetY(), 1.535, 1e-3);

    // euclidian freespace distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 0.8050, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 0.8050, 1e-3);

    // longitudinal freespace distance, in entity coordinate system, between car1 and truck1, should now be same since straight road segment
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 0.2349, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 0.2349, 1e-3);

    // move forward until car1 is overlapping truck1 longitudinally
    while (se->getSimulationTime() < 2.1 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    // euclidian freespace distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 0.7700, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 0.7700, 1e-3);

    // longitudinal freespace distance, in entity coordinate system, between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 0.0, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 0.0, 1e-3);

    // move forward until car1 is behind truck1 longitudinally
    while (se->getSimulationTime() < 2.7 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    // euclidian freespace distance between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, -1.7937, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, -1.7937, 1e-3);

    // longitudinal freespace distance, in entity coordinate system, between car1 and truck1
    car1->Distance(truck1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, -1.6200, 1e-3);
    truck1->Distance(car1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, -1.6200, 1e-3);

    // move forward until car2 is close to truck2 longitudinally
    while (se->getSimulationTime() < 3.0 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    // euclidian freespace distance between car2 and truck2
    car2->Distance(truck2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 1.0500, 1e-3);
    truck2->Distance(car2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 1.0500, 1e-3);

    // longitudinal freespace distance, in entity coordinate system, between car2 and truck2
    // image: https://drive.google.com/file/d/12FFP8DBfNpJgCpjdIpk7_kbd6xczWy79/view?usp=sharing
    car2->Distance(truck2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 0.4782, 1e-3);
    truck2->Distance(car2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 0.8017, 1e-3);

    // move forward until car2 is overlapping truck2 longitudinally, and close laterally at corner of trailer
    while (se->getSimulationTime() < 3.2 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    // euclidian freespace distance between car2 and truck2
    car2->Distance(truck2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 0.6629, 1e-3);
    truck2->Distance(car2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 0.6629, 1e-3);

    // longitudinal freespace distance, in entity coordinate system, between car2 and truck2
    car2->Distance(truck2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 0.0, 1e-3);
    truck2->Distance(car2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 0.0, 1e-3);

    // move forward until car2 is at longitudinal middle of trailer body, hence less close laterally since in curve
    while (se->getSimulationTime() < 3.5 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    // euclidian freespace distance between car2 and truck2
    car2->Distance(truck2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 1.0767, 1e-3);
    truck2->Distance(car2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 1.0767, 1e-3);

    // longitudinal freespace distance, in entity coordinate system, between car2 and truck2
    car2->Distance(truck2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 0.0, 1e-3);
    truck2->Distance(car2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 0.0, 1e-3);

    // move forward until car2 is behind truck2
    while (se->getSimulationTime() < 4.1 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    // euclidian freespace distance between car2 and truck2
    car2->Distance(truck2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, -1.9655, 1e-3);
    truck2->Distance(car2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, -1.9655, 1e-3);

    // longitudinal freespace distance, in entity coordinate system, between car2 and truck2
    car2->Distance(truck2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, -0.9049, 1e-3);
    truck2->Distance(car2, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, -1.0600, 1e-3);

    // Finally test point distance with and without trailers involved
    truck1->Distance(468.8, 1.53, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, -2.1450, 1e-3);
    truck1->Distance(468.8, 1.53, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, -2.7778, 1e-3);
    truck2->Distance(468.8, 1.53, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, 122.8735, 1e-3);
    truck2->Distance(468.8, 1.53, roadmanager::CoordinateSystem::CS_ROAD, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 132.2590, 1e-3);

    truck2->Distance(597.83, 72.84, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, -3.3915, 1e-3);
    truck2_trailer2
        ->Distance(597.83, 72.84, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, true, distance);
    EXPECT_NEAR(distance, -3.3915, 1e-3);

    truck2
        ->Distance(597.83, 72.84, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, -2.1645, 1e-3);
    truck2_trailer2
        ->Distance(597.83, 72.84, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, -2.1645, 1e-3);

    truck2->Distance(597.83, 72.84, roadmanager::CoordinateSystem::CS_ROAD, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, -2.0509, 1e-3);
    truck2_trailer2
        ->Distance(597.83, 72.84, roadmanager::CoordinateSystem::CS_ROAD, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, -2.0509, 1e-3);

    car1->Distance(300.0, 1.53, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 122.2849, 1e-3);
    car1->Distance(420.0, 1.53, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, distance);
    EXPECT_NEAR(distance, 2.2849, 1e-3);

    delete se;
}

TEST(DistanceTest, TestTrajectoryDistance)
{
    double dt = 0.1;

    ScenarioEngine* se = new ScenarioEngine("../../../resources/xosc/lane-change_clothoid_based_trajectory.xosc");
    ASSERT_NE(se, nullptr);
    se->step(0.0);
    se->prepareGroundTruth(0.0);

    scenarioengine::Entities* entities = &se->entities_;
    ASSERT_NE(entities, nullptr);
    ASSERT_EQ(entities->object_.size(), 1);
    Object* obj0 = entities->object_[0];

    while (se->getSimulationTime() < 3.1 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }

    // Check car position at given time at end phase of the scenario
    // Correct position indicates all trajectories have been evaluated correctly
    EXPECT_NEAR(obj0->pos_.GetX(), 93.0109, 1E-3);
    EXPECT_NEAR(obj0->pos_.GetY(), -0.5720, 1E-3);
    EXPECT_NEAR(obj0->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(obj0->pos_.GetH(), 0.1388), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(obj0->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(obj0->pos_.GetR(), 0.0), 0.0, 1E-3);

    Object obj1(Object::Type::VEHICLE);
    obj1.pos_.SetTrackPos(1, 93, 0);

    double dist = LARGE_NUMBER;
    EXPECT_EQ(obj0->Distance(&obj1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, false, dist), 0);
    EXPECT_NEAR(dist, 0.5721, 1E-3);
    EXPECT_EQ(
        obj0->Distance(&obj1, roadmanager::CoordinateSystem::CS_TRAJECTORY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, false, dist),
        0);
    EXPECT_NEAR(dist, 0.0661, 1E-3);

    // test with same trajectory in both objects
    obj1.pos_.SetTrajectory(obj0->pos_.GetTrajectory()->Copy());
    obj1.pos_.SetTrajectoryS(obj0->pos_.GetTrajectoryS() + 0.9);
    EXPECT_EQ(
        obj0->Distance(&obj1, roadmanager::CoordinateSystem::CS_TRAJECTORY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, false, dist),
        0);
    EXPECT_NEAR(dist, 0.9, 1E-3);

    delete (obj1.pos_.GetTrajectory());  // normally deleted by followTrajectoryAction destructor
    obj1.pos_.SetTrajectory(nullptr);
    obj1.pos_.MoveAlongS(2.0);
    EXPECT_EQ(obj0->Distance(&obj1, roadmanager::CoordinateSystem::CS_ENTITY, roadmanager::RelativeDistanceType::REL_DIST_EUCLIDIAN, false, dist), 0);
    EXPECT_NEAR(dist, 2.8936, 1E-3);

    // while obj1 has passed beyond the trajectory, it's trajectory related pos will be stuck at end of the trajectory
    EXPECT_EQ(
        obj0->Distance(&obj1, roadmanager::CoordinateSystem::CS_TRAJECTORY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, false, dist),
        0);
    EXPECT_NEAR(dist, 1.1111, 1E-3);

    // trajectory distance with freespace not supported yet
    EXPECT_EQ(
        obj0->Distance(&obj1, roadmanager::CoordinateSystem::CS_TRAJECTORY, roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL, true, dist),
        -1);

    delete se;
}

TEST(TrajectoryTest, EnsureContinuation)
{
    double          dt = 0.01;
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/trajectory-continuity.xosc");
    ASSERT_NE(se, nullptr);

    for (int i = 0; i < static_cast<int>(1.0 / dt); i++)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetX(), 4.95, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.535, 1e-5);

    for (int i = 0; i < static_cast<int>(2.0 / dt); i++)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetX(), 14.92759, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.18333, 1e-5);

    for (int i = 0; i < static_cast<int>(1.5 / dt); i++)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetX(), 21.32304, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetY(), 2.553967, 1e-5);

    for (int i = 0; i < static_cast<int>(1.0 / dt); i++)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetX(), 26.13539, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetY(), 2.917931, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetH(), 0.0, 1e-5);

    delete se;
}

TEST(TrajectoryTest, PolyLineContinuosSpeed)
{
    double          dt = 0.05;
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/brake_by_trajectory_100-0.xosc");
    ASSERT_NE(se, nullptr);

    while (se->getSimulationTime() < 1.05)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 77.77778, 1e-5);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.535, 1e-5);

    while (se->getSimulationTime() < 1.10)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 79.154167, 1e-5);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.535, 1e-5);

    while (se->getSimulationTime() < 2.45)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 107.55053, 1e-5);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.535, 1e-5);

    while (se->getSimulationTime() < 2.50)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 108.21914, 1e-5);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.535, 1e-5);

    while (se->getSimulationTime() < 4.75)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 122.78807, 1e-5);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.535, 1e-5);

    while (se->getSimulationTime() < 4.80)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 122.78806, 1e-5);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.535, 1e-5);

    delete se;
}

TEST(TrajectoryTest, FollowTrajectoryReverse)
{
    double          dt = 0.05;
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/follow_trajectory_reverse.xosc");
    ASSERT_NE(se, nullptr);

    while (se->getSimulationTime() < 4.36)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 248.858, 1e-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), 1.465, 1e-3);
    EXPECT_NEAR(se->entities_.object_[0]->GetSpeed(), 0.0, 1e-3);

    while (se->getSimulationTime() < 8.76)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 201.098, 1e-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.468, 1e-3);
    EXPECT_NEAR(se->entities_.object_[0]->GetSpeed(), 22.000, 1e-3);

    while (se->getSimulationTime() < 9.96)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 174.700, 1e-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.535, 1e-3);
    EXPECT_NEAR(se->entities_.object_[0]->GetSpeed(), 22.000, 1e-3);

    delete se;
}

TEST(ExpressionTest, ConcatenateStrings)
{
    ExprReturnStruct rs;

    rs = eval_expr("2 + 45 + ab c +  5 +   61");
    EXPECT_EQ(rs.type, EXPR_RETURN_STRING);
    EXPECT_EQ(rs._string.len, 10);
    EXPECT_STREQ(rs._string.string, "245ab c561");
    clear_expr_result(&rs);

    rs = eval_expr("1 + a + 2 * 6");
    EXPECT_EQ(rs.type, EXPR_RETURN_STRING);
    EXPECT_EQ(rs._string.len, 7);
    EXPECT_STREQ(rs._string.string, "1a2 * 6");
    clear_expr_result(&rs);

    rs = eval_expr("AC/DC 23 + 1 + ABBA * 7");
    EXPECT_EQ(rs.type, EXPR_RETURN_STRING);
    EXPECT_EQ(rs._string.len, 17);
    EXPECT_STREQ(rs._string.string, "AC/DC 231ABBA * 7");
    clear_expr_result(&rs);

    rs = eval_expr("2 + 4 + a + 5 + 6");
    EXPECT_EQ(rs.type, EXPR_RETURN_STRING);
    EXPECT_EQ(rs._string.len, 5);
    EXPECT_STREQ(rs._string.string, "24a56");
    clear_expr_result(&rs);

    rs = eval_expr("1 + a + 2");
    EXPECT_EQ(rs.type, EXPR_RETURN_STRING);
    EXPECT_EQ(rs._string.len, 3);
    EXPECT_STREQ(rs._string.string, "1a2");
    clear_expr_result(&rs);

    rs = eval_expr("ab + cde");
    EXPECT_EQ(rs.type, EXPR_RETURN_STRING);
    EXPECT_EQ(rs._string.len, 5);
    EXPECT_STREQ(rs._string.string, "abcde");
    clear_expr_result(&rs);

    rs = eval_expr("123 + _cde");
    EXPECT_EQ(rs.type, EXPR_RETURN_STRING);
    EXPECT_EQ(rs._string.len, 7);
    EXPECT_STREQ(rs._string.string, "123_cde");
    clear_expr_result(&rs);

    rs = eval_expr("_abc + 456");
    EXPECT_EQ(rs.type, EXPR_RETURN_STRING);
    EXPECT_EQ(rs._string.len, 7);
    EXPECT_STREQ(rs._string.string, "_abc456");
    clear_expr_result(&rs);

    rs = eval_expr("a + 7");
    EXPECT_EQ(rs.type, EXPR_RETURN_STRING);
    EXPECT_EQ(rs._string.len, 2);
    EXPECT_STREQ(rs._string.string, "a7");
    clear_expr_result(&rs);

    rs = eval_expr("7 + a");
    EXPECT_EQ(rs.type, EXPR_RETURN_STRING);
    EXPECT_EQ(rs._string.len, 2);
    EXPECT_STREQ(rs._string.string, "7a");
    clear_expr_result(&rs);

    rs = eval_expr("1 + a + 2");
    EXPECT_EQ(rs.type, EXPR_RETURN_STRING);
    EXPECT_EQ(rs._string.len, 3);
    EXPECT_STREQ(rs._string.string, "1a2");
    clear_expr_result(&rs);

    // missing 1st operand returns nan
    rs = eval_expr(" + cde");
    EXPECT_EQ(rs.type, EXPR_RETURN_UNDEFINED);
    EXPECT_EQ(std::isnan(rs._double), true);
    clear_expr_result(&rs);

    // missing 2nd operand will still return first string
    rs = eval_expr("abc + ");
    EXPECT_EQ(rs.type, EXPR_RETURN_STRING);
    EXPECT_EQ(rs._string.len, 3);
    EXPECT_STREQ(rs._string.string, "abc");
    clear_expr_result(&rs);

    return;
}

TEST(ExpressionTest, EnsureResult)
{
    ASSERT_DOUBLE_EQ(eval_expr("1 + 1")._double, 2.0);
    ASSERT_DOUBLE_EQ(eval_expr("5 + 10 + 1")._double, 16.0);
    ASSERT_DOUBLE_EQ(eval_expr("5 * 10 + 1")._double, 51.0);
    ASSERT_DOUBLE_EQ(eval_expr("5 * (10 + 1)")._double, 55.0);
    ASSERT_DOUBLE_EQ(eval_expr("15/3.5")._double, 15.0 / 3.5);
    ASSERT_DOUBLE_EQ(eval_expr("15 % 6")._double, 3.0);
    ASSERT_DOUBLE_EQ(eval_expr("-15 % 6")._double, -3.0);
    ASSERT_DOUBLE_EQ(eval_expr("180 % 360")._double, 180.0);
    ASSERT_DOUBLE_EQ(eval_expr("-15 % 360")._double, -15.0);
    ASSERT_DOUBLE_EQ(eval_expr("345 % 360")._double, -15.0);
    ASSERT_DOUBLE_EQ(eval_expr("-345 % 360")._double, 15.0);
    ASSERT_DOUBLE_EQ(eval_expr("705 % 360")._double, -15.0);
    ASSERT_DOUBLE_EQ(eval_expr("-705 % 360")._double, 15.0);
    ASSERT_DOUBLE_EQ(eval_expr("1 == 1")._double, 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("1 == 2")._double, 0.0);
    ASSERT_DOUBLE_EQ(eval_expr("(4 == 4) && (10 == 10)")._double, 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("4 == 4 && 10 == 10")._double, 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("4 == 4 && 9 < 10")._double, 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("4 == 4 || 11 == 10")._double, 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("4 == 3 || 9 < 8")._double, 0.0);
    ASSERT_DOUBLE_EQ(eval_expr("ceil(11.1) == 12")._double, 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("round(11.1) == 11")._double, 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("floor(11.9) == 11")._double, 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("ceil(-11.1) == -11")._double, 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("round(-11.1) == -11")._double, 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("floor(-11.9) == -12")._double, 1.0);
    ASSERT_DOUBLE_EQ(eval_expr("pow(2,3)")._double, 8.0);
    ASSERT_DOUBLE_EQ(eval_expr("2**3")._double, 8.0);
    ASSERT_DOUBLE_EQ(eval_expr("13.88888888888889 - 1.0")._double, 12.88888888888889);
    ASSERT_DOUBLE_EQ(eval_expr("13.88888888888889 - 0.0")._double, 13.88888888888889);

    // round returns the integral value that is nearest to x, with halfway cases rounded away from zero.
    ASSERT_DOUBLE_EQ(eval_expr("round(-2.5)")._double, -2.0);
    ASSERT_DOUBLE_EQ(eval_expr("round(-3.5)")._double, -4.0);
    ASSERT_DOUBLE_EQ(eval_expr("round(2.5)")._double, 2.0);
    ASSERT_DOUBLE_EQ(eval_expr("round(3.5)")._double, 4.0);

    // additional expressions not specified in OSC <=1.2
    // may not work in other OpenSCENARIO compliant tools
    EXPECT_DOUBLE_EQ(eval_expr("min(7,7.1)")._double, 7.0);
    EXPECT_DOUBLE_EQ(eval_expr("max(7,7.1)")._double, 7.1);
    EXPECT_DOUBLE_EQ(eval_expr("min(-7,-7.1)")._double, -7.1);
    EXPECT_DOUBLE_EQ(eval_expr("max(-7,-7.1)")._double, -7.0);
    EXPECT_DOUBLE_EQ(eval_expr("sign(7)")._double, 1);
    EXPECT_DOUBLE_EQ(eval_expr("sign(-7)")._double, -1);
    EXPECT_NEAR(eval_expr("sin(1.1)")._double, 0.89120, 1e-5);
    EXPECT_NEAR(eval_expr("sin(7.0)")._double, 0.65698, 1e-5);
    EXPECT_NEAR(eval_expr("cos(-2.0)")._double, -0.41614, 1e-5);
    EXPECT_NEAR(eval_expr("atan(20.0)")._double, 1.52083, 1e-5);
    EXPECT_NEAR(eval_expr("atan(-20.0)")._double, -1.52083, 1e-5);
    EXPECT_NEAR(eval_expr("asin(0.5)")._double, 0.523598, 1e-5);
    EXPECT_NEAR(eval_expr("acos(-0.5)")._double, 2.09440, 1e-5);
    EXPECT_NEAR(eval_expr("abs(-0.5)")._double, 0.5, 1e-5);
    EXPECT_NEAR(eval_expr("abs(0.5)")._double, 0.5, 1e-5);
    EXPECT_NEAR(eval_expr("abs(2.9)")._double, 2.9, 1e-5);
    EXPECT_NEAR(eval_expr("abs(-2.9)")._double, 2.9, 1e-5);

    // constants
    EXPECT_NEAR(eval_expr("pi")._double, 3.141592, 1e-5);
    EXPECT_NEAR(eval_expr("e")._double, 2.718281, 1e-5);
    EXPECT_NEAR(eval_expr("2*pi")._double, 6.283185, 1e-5);
    EXPECT_NEAR(eval_expr("2 * e")._double, 5.436563, 1e-5);
    EXPECT_NEAR(eval_expr("2*pi*pi+e")._double, 22.457490, 1e-5);
    EXPECT_NEAR(eval_expr("pow(pi,3) / e")._double, 11.406571, 1e-5);
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
    std::array<const char*, 12> args = {"my_app",
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
                                        "--option4"};
    int                         argc = static_cast<int>(args.size());

    ASSERT_EQ(opt.ParseArgs(argc, args.data()), -1);

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
    int argc_minus_one = static_cast<int>(args.size() - 1);
    ASSERT_EQ(opt.ParseArgs(argc_minus_one, args.data()), 0);
}

TEST(ParameterTest, ResolveParameterTest)
{
    Parameters params;

    params.parameterDeclarations_.Parameter.push_back({"speed", OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE, {0, 5.0, "5.0", false}});
    params.parameterDeclarations_.Parameter.push_back({"acc", OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE, {0, 3.0, "3.0", false}});
    params.parameterDeclarations_.Parameter.push_back(
        {"turnsignal", OSCParameterDeclarations::ParameterType::PARAM_TYPE_DOUBLE, {0, 0.0, "true", true}});

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
    pugi::xml_node     paramDeclsNode = xml_doc.append_child("paramDeclsNode");

    pugi::xml_node paramDeclNode0                    = paramDeclsNode.append_child("paramDeclNode0");
    paramDeclNode0.append_attribute("name")          = "param0";
    paramDeclNode0.append_attribute("parameterType") = "double";
    paramDeclNode0.append_attribute("value")         = "17.0";

    pugi::xml_node paramDeclNode1                    = paramDeclsNode.append_child("paramDeclNode1");
    paramDeclNode1.append_attribute("name")          = "param1";
    paramDeclNode1.append_attribute("parameterType") = "boolean";
    paramDeclNode1.append_attribute("value")         = "true";

    Parameters params;
    params.addParameterDeclarations(paramDeclsNode);

    // Create an XML element with attributes referring to parameters
    pugi::xml_node someNode0            = xml_doc.append_child("someNode0");
    someNode0.append_attribute("speed") = "5.1";
    someNode0.append_attribute("acc")   = "$param0";
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

    double angles[]    = {3 * M_PI_2, -M_PI_2, 0.0, M_PI_2};
    int    roadIds[]   = {1, 1, 2, 3};
    double durations[] = {2.5, 2.5, 2.6, 2.8};  // Make sure car gets gets out of the intersection

    for (int i = 0; i < static_cast<int>(sizeof(angles) / sizeof(double)); i++)
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
    double dt           = 0.01;
    double timestamps[] = {5.24, 5.25, 6.25, 6.26, 7.10, 8.78};

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
    for (size_t i = 0; i < 2; i++)
    {
        scenarioengine::Controller::InitArgs args;
        args.name       = "UDPDriverModel Controller";
        args.type       = ControllerUDPDriver::GetTypeNameStatic();
        args.parameters = 0;
        args.gateway    = se->getScenarioGateway();
        args.properties = new OSCProperties();
        OSCProperties::Property property;
        property.name_  = "port";
        property.value_ = std::to_string(0);
        args.properties->property_.push_back(property);
        property.name_  = "basePort";
        property.value_ = std::to_string(61900);
        args.properties->property_.push_back(property);
        property.name_  = "inputMode";
        property.value_ = "vehicleStateXYH";
        args.properties->property_.push_back(property);
        ControllerUDPDriver* controller = reinterpret_cast<ControllerUDPDriver*>(InstantiateControllerUDPDriver(&args));

        for (auto ctrl : se->entities_.object_[i]->controllers_)
        {
            se->entities_.object_[i]->UnassignController(ctrl);
            delete ctrl;
        }
        delete args.properties;
        se->scenarioReader->controller_[i] = controller;
        se->entities_.object_[i]->AssignController(controller);
        controller->LinkObject(se->entities_.object_[i]);
    }

    // assign controllers
    se->step(dt);

    // stimulate driver input
    UDPClient* udpClient = new UDPClient(61900, "127.0.0.1");

    ControllerUDPDriver::DMMessage msg;

    msg.header.frameNumber = 0;
    msg.header.version     = 1;
    msg.header.objectId    = 0;
    msg.header.inputMode   = static_cast<int>(ControllerUDPDriver::InputMode::VEHICLE_STATE_XYH);

    msg.message.stateXYH.x          = 20.0;
    msg.message.stateXYH.y          = 30.0;
    msg.message.stateXYH.h          = 0.3;
    msg.message.stateXYH.speed      = 30.0;
    msg.message.stateXYH.wheelAngle = 0.1;
    msg.message.stateXYH.deadReckon = 0;

    udpClient->Send(reinterpret_cast<char*>(&msg), sizeof(msg));

    // Make sure last message is applied
    msg.message.stateXYZHPR.y = 40;
    udpClient->Send(reinterpret_cast<char*>(&msg), sizeof(msg));

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
    udpClient->Send(reinterpret_cast<char*>(&msg), sizeof(msg));
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
    for (size_t i = 0; i < 2; i++)
    {
        scenarioengine::Controller::InitArgs args;
        args.name       = "UDPDriverModel Controller";
        args.type       = ControllerUDPDriver::GetTypeNameStatic();
        args.parameters = 0;
        args.gateway    = se->getScenarioGateway();
        args.properties = new OSCProperties();
        OSCProperties::Property property;
        property.name_  = "execMode";
        property.value_ = "synchronous";
        args.properties->property_.push_back(property);
        property.name_  = "port";
        property.value_ = std::to_string(0);
        args.properties->property_.push_back(property);
        property.name_  = "basePort";
        property.value_ = std::to_string(61910);
        args.properties->property_.push_back(property);
        property.name_  = "inputMode";
        property.value_ = "vehicleStateXYZHPR";
        args.properties->property_.push_back(property);
        property.name_  = "timoutMs";
        property.value_ = std::to_string(500);
        args.properties->property_.push_back(property);
        ControllerUDPDriver* controller = reinterpret_cast<ControllerUDPDriver*>(InstantiateControllerUDPDriver(&args));

        for (auto ctrl : se->entities_.object_[i]->controllers_)
        {
            se->entities_.object_[i]->UnassignController(ctrl);
            delete ctrl;
        }
        delete args.properties;
        se->scenarioReader->controller_[i] = controller;
        se->entities_.object_[i]->AssignController(controller);
        controller->LinkObject(se->entities_.object_[i]);
    }

    // assign controllers
    se->step(dt);

    // stimulate driver input
    UDPClient* udpClient = new UDPClient(61910, "127.0.0.1");

    ControllerUDPDriver::DMMessage msg;

    msg.header.frameNumber = 0;
    msg.header.version     = 1;
    msg.header.objectId    = 0;
    msg.header.inputMode   = static_cast<int>(ControllerUDPDriver::InputMode::VEHICLE_STATE_XYZHPR);

    msg.message.stateXYZHPR.h          = 0.3;
    msg.message.stateXYZHPR.x          = 20.0;
    msg.message.stateXYZHPR.y          = 30.0;
    msg.message.stateXYZHPR.deadReckon = 0;

    udpClient->Send(reinterpret_cast<char*>(&msg), sizeof(msg));

    // Put another message on queue of first vehicle
    msg.header.frameNumber++;
    msg.message.stateXYZHPR.y = 40;
    udpClient->Send(reinterpret_cast<char*>(&msg), sizeof(msg));

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
    msg.header.version     = 1;
    msg.header.objectId    = 1;
    msg.header.inputMode   = static_cast<int>(ControllerUDPDriver::InputMode::VEHICLE_STATE_XYZHPR);

    msg.message.stateXYZHPR.h          = 0.3;
    msg.message.stateXYZHPR.x          = 90.0;
    msg.message.stateXYZHPR.y          = -10.0;
    msg.message.stateXYZHPR.deadReckon = 0;

    udpClient2->Send(reinterpret_cast<char*>(&msg), sizeof(msg));

    msg.header.frameNumber    = 2;
    msg.header.objectId       = 0;
    msg.message.stateXYZHPR.x = 150.0;
    udpClient->Send(reinterpret_cast<char*>(&msg), sizeof(msg));

    se->step(dt);
    se->step(dt);
    EXPECT_DOUBLE_EQ(se->entities_.object_[0]->pos_.GetX(), 150.0);
    EXPECT_DOUBLE_EQ(se->entities_.object_[1]->pos_.GetX(), 90.0);
    EXPECT_DOUBLE_EQ(se->entities_.object_[1]->pos_.GetY(), -10.0);

    delete se;
    delete udpClient;
    delete udpClient2;
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
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetZ(), -0.61162, 1e-5);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetP(), 0.0, 1e-5);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetR(), 0.37917, 1e-5);

    // Fast forward
    while (se->getSimulationTime() < (6.0 - SMALL_NUMBER))
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    EXPECT_NEAR(se->entities_.object_[1]->pos_.GetZ(), 0.50319, 1e-5);
    EXPECT_NEAR(fmod(2.0 * M_PI, se->entities_.object_[1]->pos_.GetP()), 0.0, 1e-5);
    EXPECT_NEAR(se->entities_.object_[1]->pos_.GetR(), 5.96641, 1e-5);

    EXPECT_NEAR(se->entities_.object_[2]->pos_.GetZ(), 13.24676, 1e-5);
    EXPECT_NEAR(se->entities_.object_[2]->pos_.GetP(), 0.27808, 1e-5);
    EXPECT_NEAR(fmod(2.0 * M_PI, se->entities_.object_[2]->pos_.GetR()), 0, 1e-5);

    delete se;
}

TEST(ActionDynamicsTest, TestDynamicsTimeDimension)
{
    OSCPrivateAction::TransitionDynamics td;
    double                               p_target = 0.0;
    double                               v_start  = 0.0;
    double                               v_target = 0.0;

    td.dimension_ = OSCPrivateAction::DynamicsDimension::TIME;
    td.shape_     = OSCPrivateAction::DynamicsShape::LINEAR;

    p_target = 10.0;
    v_start  = 0.0;
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
    v_start  = 10.0;
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
    v_start  = 10.0;
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
    td.shape_     = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
    td.Reset();
    p_target = 10.0;
    v_start  = 0.0;
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
    td.shape_     = OSCPrivateAction::DynamicsShape::CUBIC;
    td.Reset();
    p_target = 10.0;
    v_start  = 110.0;
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
    double                               p_target = 0.0;
    double                               v_start  = 0.0;
    double                               v_target = 0.0;

    td.dimension_ = OSCPrivateAction::DynamicsDimension::DISTANCE;
    td.shape_     = OSCPrivateAction::DynamicsShape::LINEAR;

    p_target = 100.0;
    v_start  = 0.0;
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
    v_start  = 10.0;
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
    v_start  = 10.0;
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
    td.shape_     = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
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
    td.shape_     = OSCPrivateAction::DynamicsShape::CUBIC;
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
    td.shape_     = OSCPrivateAction::DynamicsShape::LINEAR;

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
    td.Step(2.0);  // at end of param span
    EXPECT_DOUBLE_EQ(td.Evaluate(), -50.0);
    td.Step(30.0);  // clamp at 50.0s
    EXPECT_DOUBLE_EQ(td.Evaluate(), -50.0);

    td.dimension_ = OSCPrivateAction::DynamicsDimension::RATE;
    td.shape_     = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
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
    td.shape_     = OSCPrivateAction::DynamicsShape::CUBIC;
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

    delete se;
}

TEST(SpeedProfileTest, TestSpeedProfileFirstEntryOffset)
{
    LongSpeedProfileAction        sp_action(nullptr);
    DynamicConstraints            dynamics;  // initalized with default values
    LongSpeedProfileAction::Entry entry;

    Object obj(Object::Type::VEHICLE);
    obj.SetSpeed(10.0);
    double sim_time = 0.0, dt = 0.0;

    sp_action.following_mode_ = FollowingMode::POSITION;
    sp_action.dynamics_       = dynamics;
    sp_action.object_         = &obj;

    ASSERT_EQ(sp_action.entity_ref_, nullptr);
    ASSERT_EQ(sp_action.segment_.size(), 0);

    // Add entries
    entry.speed_ = 4.0;
    entry.time_  = 2.0;
    sp_action.AddEntry(entry);

    sp_action.Start(0.1);

    // Evaluate at a time before first entry time, speed should interpolate towards first entry
    sp_action.Step(1.0);
    EXPECT_NEAR(sp_action.GetSpeed(), 7.30, 1E-5);
    sim_time += dt;
}

TEST(SpeedProfileTest, TestSpeedProfileLinear)
{
    LongSpeedProfileAction        sp_action(nullptr);
    DynamicConstraints            dynamics;  // initalized with default values
    LongSpeedProfileAction::Entry entry;

    Object obj(Object::Type::VEHICLE);
    obj.SetSpeed(10.0);
    double sim_time = 5.0;

    sp_action.following_mode_ = FollowingMode::POSITION;
    sp_action.dynamics_       = dynamics;
    sp_action.object_         = &obj;

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
    LongSpeedProfileAction        sp_action(nullptr);
    LongSpeedProfileAction::Entry entry;
    DynamicConstraints            dynamics;

    dynamics.max_acceleration_      = 4.0;
    dynamics.max_acceleration_rate_ = 1.0;
    dynamics.max_deceleration_      = 5.0;
    dynamics.max_deceleration_rate_ = 2.0;
    dynamics.max_speed_             = 30.0;

    Object obj(Object::Type::VEHICLE);
    obj.SetSpeed(10.0);

    sp_action.following_mode_ = FollowingMode::POSITION;
    sp_action.dynamics_       = dynamics;
    sp_action.object_         = &obj;

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
    LongSpeedProfileAction        sp_action(nullptr);
    LongSpeedProfileAction::Entry entry;

    sp_action.dynamics_.max_acceleration_      = 4.0;
    sp_action.dynamics_.max_acceleration_rate_ = 1.0;
    sp_action.dynamics_.max_deceleration_      = 5.0;
    sp_action.dynamics_.max_deceleration_rate_ = 2.0;
    sp_action.dynamics_.max_speed_             = 30.0;

    Object obj(Object::Type::VEHICLE);
    obj.SetSpeed(1.0);

    sp_action.following_mode_ = FollowingMode::FOLLOW;
    sp_action.object_         = &obj;

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
    LongSpeedProfileAction        sp_action(nullptr);
    LongSpeedProfileAction::Entry entry;

    Object obj(Object::Type::VEHICLE);

    sp_action.object_ = &obj;
    ASSERT_EQ(sp_action.entity_ref_, nullptr);
    ASSERT_EQ(sp_action.entry_.size(), 0);
    sp_action.dynamics_.max_acceleration_      = 5.0;
    sp_action.dynamics_.max_deceleration_      = 10.0;
    sp_action.dynamics_.max_acceleration_rate_ = 5.0;
    sp_action.dynamics_.max_deceleration_rate_ = 5.0;
    sp_action.dynamics_.max_speed_             = 30.0;

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
    LongSpeedProfileAction        sp_action(nullptr);
    LongSpeedProfileAction::Entry entry;

    Object obj(Object::Type::VEHICLE);

    sp_action.object_ = &obj;
    ASSERT_EQ(sp_action.entity_ref_, nullptr);
    ASSERT_EQ(sp_action.entry_.size(), 0);
    sp_action.dynamics_.max_acceleration_      = 4.0;
    sp_action.dynamics_.max_deceleration_      = 10.0;
    sp_action.dynamics_.max_acceleration_rate_ = 5.0;
    sp_action.dynamics_.max_deceleration_rate_ = 4.0;
    sp_action.dynamics_.max_speed_             = 30.0;

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
    LongSpeedProfileAction        sp_action(nullptr);
    DynamicConstraints            dynamics;  // initalized with default values
    LongSpeedProfileAction::Entry entry;

    Object obj(Object::Type::VEHICLE);
    obj.SetSpeed(10.0);
    obj.SetAcc(1.0, 0.0, 0.0);

    sp_action.following_mode_ = FollowingMode::FOLLOW;
    sp_action.dynamics_       = dynamics;
    sp_action.object_         = &obj;

    sp_action.dynamics_.max_acceleration_      = 4.0;
    sp_action.dynamics_.max_deceleration_      = 10.0;
    sp_action.dynamics_.max_acceleration_rate_ = 5.0;
    sp_action.dynamics_.max_deceleration_rate_ = 4.0;
    sp_action.dynamics_.max_speed_             = 30.0;

    ASSERT_EQ(sp_action.entity_ref_, nullptr);
    ASSERT_EQ(sp_action.segment_.size(), 0);

    // Add entries
    entry.speed_ = 4.0;
    entry.time_  = 5.0;
    sp_action.AddEntry(entry);

    sp_action.Start(0.1);

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

TEST(ControllerTest, ALKS_R157_TestR157RegulationMinDist)
{
    double dt = 0.01;

    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/alks_r157_test.xosc");
    ASSERT_NE(se, nullptr);
    ASSERT_EQ(se->entities_.object_.size(), 2);

    // Set controller
    scenarioengine::Controller::InitArgs args;
    args.name       = "ALKS_R157SM_Controller";
    args.type       = ControllerALKS_R157SM::GetTypeNameStatic();
    args.parameters = 0;
    args.gateway    = se->getScenarioGateway();
    args.properties = new OSCProperties();
    OSCProperties::Property property;
    property.name_  = "model";
    property.value_ = "Regulation";
    args.properties->property_.push_back(property);
    ControllerALKS_R157SM* controller = reinterpret_cast<ControllerALKS_R157SM*>(InstantiateControllerALKS_R157SM(&args));
    controller->SetScenarioEngine(se);

    Object* obj = se->entities_.object_[0];

    for (auto ctrl : obj->controllers_)
    {
        obj->UnassignController(ctrl);
        delete ctrl;
    }
    delete args.properties;
    se->scenarioReader->controller_[0] = controller;
    obj->AssignController(controller);
    controller->LinkObject(obj);

    // assign controllers
    se->step(dt);

    obj->SetSpeed(0);
    EXPECT_EQ(controller->model_->MinDist(), 2.0);

    obj->SetSpeed(1.9);
    EXPECT_EQ(controller->model_->MinDist(), 2.0);

    obj->SetSpeed(2.0);
    EXPECT_EQ(controller->model_->MinDist(), 2.0);

    obj->SetSpeed(2.5);
    EXPECT_NEAR(controller->model_->MinDist(), 2.660, 1E-3);

    obj->SetSpeed(10.0 / 3.6);  // break point
    EXPECT_NEAR(controller->model_->MinDist(), 3.056, 1E-3);

    obj->SetSpeed(13.0 / 3.6);
    EXPECT_NEAR(controller->model_->MinDist(), 4.081, 1E-3);

    obj->SetSpeed(20.0 / 3.6);
    EXPECT_NEAR(controller->model_->MinDist(), 6.667, 1E-3);

    obj->SetSpeed(40.0 / 3.6);
    EXPECT_NEAR(controller->model_->MinDist(), 15.556, 1E-3);

    obj->SetSpeed(58.0 / 3.6);
    EXPECT_NEAR(controller->model_->MinDist(), 25.456, 1E-3);

    obj->SetSpeed(60.0 / 3.6);
    EXPECT_NEAR(controller->model_->MinDist(), 26.667, 1E-3);

    obj->SetSpeed(90.0 / 3.6);  // Outside range, but supported anyway
    EXPECT_NEAR(controller->model_->MinDist(), 47.500, 1E-3);

    delete se;
}

static void alksRefDriverParamDeclCallback(void* arg)
{
    bool aeb_available = *(static_cast<bool*>(arg));

    ScenarioReader::parameters.setParameterValue("AEBAvailableInEgo", aeb_available);
}

TEST(ControllerTest, ALKS_R157_TestR157RefDriverBrakeRate)
{
    double dt               = 0.05;
    bool   aeb_available[2] = {true, false};

    // Run first with AEB enabled, then without
    for (int i = 0; i < 2; i++)
    {
        RegisterParameterDeclarationCallback(alksRefDriverParamDeclCallback, &(aeb_available[i]));
        ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/alks_r157_quick_stop_test.xosc");
        ASSERT_NE(se, nullptr);
        ASSERT_EQ(se->entities_.object_.size(), 3);
        Object* obj = se->entities_.object_[0];

        int state = 0;
        while (se->getSimulationTime() < 7.5)
        {
            se->step(dt);
            se->prepareGroundTruth(dt);

            if (state == 0 && se->getSimulationTime() > 2.0)
            {
                if (i == 0)
                {
                    EXPECT_NEAR(obj->pos_.GetX(), 49.250, 1e-3);
                    EXPECT_NEAR(obj->pos_.GetY(), -1.535, 1e-3);
                }
                else
                {
                    EXPECT_NEAR(obj->pos_.GetX(), 49.250, 1e-3);
                    EXPECT_NEAR(obj->pos_.GetY(), -1.535, 1e-3);
                }
                state++;
            }
            else if (state == 1 && se->getSimulationTime() > 3.4)
            {
                if (i == 0)
                {
                    EXPECT_NEAR(obj->pos_.GetX(), 70.322, 1e-3);
                    EXPECT_NEAR(obj->pos_.GetY(), -1.535, 1e-3);
                }
                else
                {
                    EXPECT_NEAR(obj->pos_.GetX(), 70.322, 1e-3);
                    EXPECT_NEAR(obj->pos_.GetY(), -1.535, 1e-3);
                }
                state++;
            }
            else if (state == 2 && se->getSimulationTime() > 5.0)
            {
                if (i == 0)
                {
                    EXPECT_NEAR(obj->pos_.GetX(), 81.982, 1e-3);
                    EXPECT_NEAR(obj->pos_.GetY(), -1.535, 1e-3);
                }
                else
                {
                    EXPECT_NEAR(obj->pos_.GetX(), 87.362, 1e-3);
                    EXPECT_NEAR(obj->pos_.GetY(), -1.535, 1e-3);
                }
                state++;
            }
            else if (state == 3 && se->getSimulationTime() > 8.0)
            {
                if (i == 0)
                {
                    EXPECT_NEAR(obj->pos_.GetX(), 82.571, 1e-3);
                    EXPECT_NEAR(obj->pos_.GetY(), -1.535, 1e-3);
                }
                else
                {
                    EXPECT_NEAR(obj->pos_.GetX(), 89.908, 1e-3);
                    EXPECT_NEAR(obj->pos_.GetY(), -1.535, 1e-3);
                }
                state++;
            }
        }

        delete se;
    }

    RegisterParameterDeclarationCallback(nullptr, nullptr);
}

TEST(OverlapTest, TestOverlapCalculations)
{
    SE_Vector line_v0(2.0, 1.0);
    SE_Vector line_v1(2.0, -1.0);

    SE_Vector point_to_test(5, -0.5);
    double    projected_point[2];
    double    s_norm    = 0.0;
    bool      is_within = false;

    ProjectPointOnLine2D(point_to_test.x(),
                         point_to_test.y(),
                         line_v0.x(),
                         line_v0.y(),
                         line_v1.x(),
                         line_v1.y(),
                         projected_point[0],
                         projected_point[1]);
    is_within = PointInBetweenVectorEndpoints(projected_point[0], projected_point[1], line_v0.x(), line_v0.y(), line_v1.x(), line_v1.y(), s_norm);
    EXPECT_EQ(is_within, true);
    EXPECT_NEAR(s_norm, 0.75, 1e-3);

    point_to_test = {-5, -0.5};
    ProjectPointOnLine2D(point_to_test.x(),
                         point_to_test.y(),
                         line_v0.x(),
                         line_v0.y(),
                         line_v1.x(),
                         line_v1.y(),
                         projected_point[0],
                         projected_point[1]);
    is_within = PointInBetweenVectorEndpoints(projected_point[0], projected_point[1], line_v0.x(), line_v0.y(), line_v1.x(), line_v1.y(), s_norm);
    EXPECT_EQ(is_within, true);
    EXPECT_NEAR(s_norm, 0.75, 1e-3);

    point_to_test = {-5, -1.1};
    ProjectPointOnLine2D(point_to_test.x(),
                         point_to_test.y(),
                         line_v0.x(),
                         line_v0.y(),
                         line_v1.x(),
                         line_v1.y(),
                         projected_point[0],
                         projected_point[1]);
    is_within = PointInBetweenVectorEndpoints(projected_point[0], projected_point[1], line_v0.x(), line_v0.y(), line_v1.x(), line_v1.y(), s_norm);
    EXPECT_EQ(is_within, false);
    EXPECT_NEAR(s_norm, 0.1, 1e-3);

    point_to_test = {-5, 1.1};
    ProjectPointOnLine2D(point_to_test.x(),
                         point_to_test.y(),
                         line_v0.x(),
                         line_v0.y(),
                         line_v1.x(),
                         line_v1.y(),
                         projected_point[0],
                         projected_point[1]);
    is_within = PointInBetweenVectorEndpoints(projected_point[0], projected_point[1], line_v0.x(), line_v0.y(), line_v1.x(), line_v1.y(), s_norm);
    EXPECT_EQ(is_within, false);
    EXPECT_NEAR(s_norm, -0.1, 1e-3);

    line_v0       = {-1.0, 0.0};
    line_v1       = {1.0, 0.0};
    point_to_test = {-0.5, 2.0};
    ProjectPointOnLine2D(point_to_test.x(),
                         point_to_test.y(),
                         line_v0.x(),
                         line_v0.y(),
                         line_v1.x(),
                         line_v1.y(),
                         projected_point[0],
                         projected_point[1]);
    is_within = PointInBetweenVectorEndpoints(projected_point[0], projected_point[1], line_v0.x(), line_v0.y(), line_v1.x(), line_v1.y(), s_norm);
    EXPECT_EQ(is_within, true);
    EXPECT_NEAR(s_norm, 0.25, 1e-3);

    point_to_test = {-20.5, 2.0};
    ProjectPointOnLine2D(point_to_test.x(),
                         point_to_test.y(),
                         line_v0.x(),
                         line_v0.y(),
                         line_v1.x(),
                         line_v1.y(),
                         projected_point[0],
                         projected_point[1]);
    is_within = PointInBetweenVectorEndpoints(projected_point[0], projected_point[1], line_v0.x(), line_v0.y(), line_v1.x(), line_v1.y(), s_norm);
    EXPECT_EQ(is_within, false);
    EXPECT_NEAR(s_norm, -19.5, 1e-3);
}

class StraightRoadTest : public testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        ASSERT_EQ(roadmanager::Position::LoadOpenDrive("../../../resources/xodr/straight_500m.xodr"), true);
        static OpenDrive* odr = Position::GetOpenDrive();
        ASSERT_NE(odr, nullptr);
        EXPECT_EQ(odr->GetNumOfRoads(), 1);
    }

    static void TearDownTestSuite()
    {
    }
};

TEST_F(StraightRoadTest, TestRoadPosition)
{
    OSCOrientation o(Position::OrientationType::ORIENTATION_RELATIVE, 0.1, 0.0, 0.0);

    OSCPositionRoad road_pos1(1, 50, 1.5, o);
    EXPECT_NEAR(reinterpret_cast<OSCPosition&>(road_pos1).GetRMPos()->GetH(), 0.1, 1e-3);

    OSCPositionRoad road_pos2(1, 50, -1.5, o);
    EXPECT_NEAR(reinterpret_cast<OSCPosition&>(road_pos2).GetRMPos()->GetH(), 0.1, 1e-3);
}

TEST(DistributionTest, TestDeterministicDistribution)
{
    OSCParameterDistribution& dist = OSCParameterDistribution::Inst();

    EXPECT_EQ(dist.Load("../../../resources/xosc/cut-in_parameter_set.xosc"), 0);

    dist.SetIndex(0);
    EXPECT_EQ(dist.GetNumParameters(), 4);
    EXPECT_EQ(dist.GetNumPermutations(), 12);

    EXPECT_EQ(dist.GetParamName(0), "HostVehicle");
    EXPECT_EQ(dist.GetParamName(1), "TargetVehicle");
    EXPECT_EQ(dist.GetParamName(2), "EgoSpeed");
    EXPECT_EQ(dist.GetParamName(3), "TargetSpeedFactor");
    EXPECT_EQ(dist.GetParamName(4), "");

    // check some samples
    EXPECT_EQ(dist.SetIndex(4), 0);
    EXPECT_EQ(dist.GetParamName(0), "HostVehicle");
    EXPECT_EQ(dist.GetParamValue(0), "car_blue");
    EXPECT_EQ(dist.GetParamName(1), "TargetVehicle");
    EXPECT_EQ(dist.GetParamValue(1), "car_yellow");
    EXPECT_EQ(dist.SetIndex(12), -1);
    EXPECT_EQ(dist.SetIndex(13), -1);
    EXPECT_EQ(dist.SetIndex(7), 0);
    EXPECT_EQ(dist.GetNumParameters(), 3);
    EXPECT_EQ(dist.GetParamName(0), "TargetVehicle");
    EXPECT_EQ(dist.GetParamValue(0), "van_red");
    EXPECT_EQ(dist.SetIndex(5), 0);
    EXPECT_NEAR(std::atof(dist.GetParamValue(2).c_str()), 110.0, 1e-3);
    EXPECT_NEAR(std::atof(dist.GetParamValue(3).c_str()), 1.5, 1e-3);
    EXPECT_EQ(dist.SetIndex(3), 0);
    EXPECT_NEAR(std::atof(dist.GetParamValue(2).c_str()), 110.0, 1e-3);
    EXPECT_NEAR(std::atof(dist.GetParamValue(3).c_str()), 1.1, 1e-3);
    EXPECT_EQ(dist.SetIndex(1), 0);
    EXPECT_NEAR(std::atof(dist.GetParamValue(2).c_str()), 70.0, 1e-3);
    EXPECT_NEAR(std::atof(dist.GetParamValue(3).c_str()), 1.3, 1e-3);
    dist.Reset();
}

TEST_F(StraightRoadTest, TestObjectOverlap)
{
    Object ego(Object::Type::VEHICLE);
    Object target(Object::Type::VEHICLE);

    ego.boundingbox_ = {{1.0, 0.0, 0.0}, {2.0, 5.0, 1.5}};
    ego.pos_.SetInertiaPos(10, 1.5, 0.0);
    target.boundingbox_ = {{0.5, 0.0, 0.0}, {1.0, 1.0, 1.5}};
    target.pos_.SetInertiaPos(10, 1.5, 0.0);
    EXPECT_EQ(ego.OverlappingFront(&target, 0.1), Object::OverlapType::INSIDE);

    target.boundingbox_ = {{0.5, 0.0, 0.0}, {6.0, 2.0, 1.5}};
    EXPECT_EQ(ego.OverlappingFront(&target, 0.1), Object::OverlapType::FULL);

    ego.pos_.SetInertiaPos(10, 0.0, 0.0);
    target.pos_.SetInertiaPos(10, 3.0, 0.0);
    EXPECT_EQ(ego.OverlappingFront(&target, 0.1), Object::OverlapType::PART);

    ego.boundingbox_ = {{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
    ego.pos_.SetInertiaPos(0.0, 0.0, 0.0);
    target.boundingbox_ = {{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}};
    target.pos_.SetInertiaPos(10.0, 1.01, 0.0);
    EXPECT_EQ(ego.OverlappingFront(&target, 0.0), Object::OverlapType::NONE);

    EXPECT_EQ(ego.OverlappingFront(&target, 0.02), Object::OverlapType::PART);

    // Rotate ego 90 deg
    ego.pos_.SetHeading(M_PI_2, false);
    target.pos_.SetInertiaPos(0.0, 10.0, 0.0);
    EXPECT_EQ(ego.OverlappingFront(&target, 0.01), Object::OverlapType::INSIDE_AND_FULL);
}

TEST(SpeedTest, TestAbsoluteSpeed)
{
    double          dt = 0.05;
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/lateral_maneuvers_test.xosc");
    ASSERT_NE(se, nullptr);

    double time = 0.0;

    while (time < 7.5)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
        time = se->getSimulationTime();

        if (time > 1.1 + SMALL_NUMBER && time < 3.05 + SMALL_NUMBER)
        {
            // Lane change action
            EXPECT_NEAR(se->entities_.object_[0]->pos_.GetVelY(), 1.535, 1e-3);
        }
        else if (time > 4.15 + SMALL_NUMBER && time < 7.10 + SMALL_NUMBER)
        {
            // Lane change action
            EXPECT_NEAR(se->entities_.object_[0]->pos_.GetVelY(), -1.0116, 1e-4);
        }
        else if (time > 7.2)
        {
            ASSERT_NEAR(se->entities_.object_[0]->pos_.GetVelY(), 0.0, 1e-4);
        }
    }

    delete se;
}

TEST(SpeedTest, TestChangeSpeedOverDistance)
{
    LongSpeedAction action(nullptr);
    Object          obj(Object::Type::VEHICLE);
    action.object_ = &obj;

    std::shared_ptr<LongSpeedAction::TargetAbsolute> target = std::make_shared<LongSpeedAction::TargetAbsolute>();
    action.target_                                          = target;

    action.transition_.dimension_ = OSCPrivateAction::DynamicsDimension::DISTANCE;
    action.transition_.shape_     = OSCPrivateAction::DynamicsShape::LINEAR;

    double v0[6]   = {5.0, -5.0, 0.0, -5.0, 5.0, 1.5};
    double v1[6]   = {10.0, -10.0, 0.0, 5.0, -5.0, -0.5};
    double dist[6] = {20.0, 20.0, 20.0, 20.0, 20.0, 1.25};
    double time[6] = {2.66667, 2.66667, 0.0, 8.0, 8.0, 2.0};

    for (unsigned int i = 0; i < static_cast<unsigned int>(sizeof(v0) / sizeof(double)); i++)
    {
        obj.SetSpeed(v0[i]);
        target->value_ = v1[i];
        action.transition_.SetParamTargetVal(dist[i]);  // distance

        action.Start(0.0);
        EXPECT_NEAR(action.transition_.GetParamTargetVal(), time[i], 1E-5);
    }
}

TEST(ControllerTest, TestLoomingControllerSimple)
{
    double          dt = 0.05;
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/loomingTest.xosc");
    ASSERT_NE(se, nullptr);

    while (se->getSimulationTime() < 5.0 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetS(), 41.4060182298, 1E-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetT(), -1.0844662327, 1E-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetH(), 0.2065681342, 1e-5);

    while (se->getSimulationTime() < 10.0 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetS(), 82.8457730916, 1E-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetT(), -1.0642454223, 1E-5);

    while (se->getSimulationTime() < 15.0 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetS(), 4.2868112508, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetT(), -1.0630431865, 1e-5);

    while (se->getSimulationTime() < 20.0 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetS(), 45.7279451776, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetT(), -1.0638741808, 1e-5);

    while (se->getSimulationTime() < 25.0 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetS(), 87.1641289481, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetT(), -1.2282167087, 1e-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetH(), 1.0125829829, 1e-5);

    delete se;
}

TEST(ControllerTest, TestLoomingSimpleFarTan)
{
    double          dt = 0.05;
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/loomingTest.xosc");
    ASSERT_NE(se, nullptr);

    // Get handle to first controller, which we know is a looming controller
    ASSERT_EQ(se->scenarioReader->controller_[0]->GetType(), scenarioengine::Controller::Type::CONTROLLER_TYPE_LOOMING);

    ControllerLooming* ctrl = reinterpret_cast<ControllerLooming*>(se->scenarioReader->controller_[0]);
    ASSERT_NE(ctrl, nullptr);

    while (se->getSimulationTime() < 5.0 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
        EXPECT_EQ(ctrl->getHasFarTan(), true);
    }
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetS(), 41.4060182298, 1E-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetT(), -1.0844662326, 1E-5);

    delete se;
}

static void clearanceFreeSpaceParamDeclCallback(void*)
{
    static int counter  = 0;
    bool       value[2] = {true, false};

    if (counter < 2)
    {
        ScenarioReader::parameters.setParameterValue("FreeSpace", value[counter]);
    }

    counter++;
}

TEST(RelativeClearanceTest, TestRelativeClearanceFreeSpace)
{
    double dt = 0.05;

    RegisterParameterDeclarationCallback(clearanceFreeSpaceParamDeclCallback, 0);
    for (int i = 0; i < 2; i++)
    {
        ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/relative_clearance_freeSpace.xosc");
        ASSERT_NE(se, nullptr);

        while (se->getSimulationTime() < 5.0 - SMALL_NUMBER)
        {
            se->step(dt);
            se->prepareGroundTruth(dt);
        }

        ASSERT_NEAR(se->entities_.object_[2]->pos_.GetT(), -4.500000, 1E-3);
        ASSERT_EQ(se->entities_.object_[2]->GetName(), "TargetRef");

        while (se->getSimulationTime() < 10.5 - SMALL_NUMBER)
        {
            se->step(dt);
            se->prepareGroundTruth(dt);
        }

        if (i == 1)
        {
            ASSERT_NEAR(se->entities_.object_[2]->pos_.GetT(), -3.000, 1E-3);
            ASSERT_EQ(se->entities_.object_[2]->GetName(), "TargetRef");
        }
        if (i == 2)
        {
            ASSERT_NEAR(se->entities_.object_[2]->pos_.GetT(), -3.7222222, 1E-3);
            ASSERT_EQ(se->entities_.object_[2]->GetName(), "TargetRef");
        }
        delete se;
    }
    RegisterParameterDeclarationCallback(nullptr, 0);
}

TEST(TwoPlusOneRoadTest, TestTwoPlusOneRoad)
{
    double          dt = 0.05;
    ScenarioEngine* se = new ScenarioEngine("../../../resources/xosc/two_plus_one_road.xosc");
    struct
    {
        double time;
        double s;
        double t;
        double h;
        int    lane_id;
    } exp_values[5] = {{4.0, 115.0, -1.75, 0.0, -1},
                       {5.25, 134.19, -1.789, 0.054, -2},
                       {7.0, 168.89, -2.154, 0.05, -1},
                       {9.0, 218.22, -1.75, 0.0, -1},
                       {11.25, 274.39, -4.118, 6.20, -2}};

    ASSERT_NE(se, nullptr);

    for (int i = 0; i < 5; i++)
    {
        while (se->getSimulationTime() < exp_values[i].time + SMALL_NUMBER)
        {
            se->step(dt);
            se->prepareGroundTruth(dt);
        }
        EXPECT_NEAR(se->entities_.object_[1]->pos_.GetS(), exp_values[i].s, 1E-2);
        EXPECT_NEAR(se->entities_.object_[1]->pos_.GetT(), exp_values[i].t, 1E-2);
        EXPECT_NEAR(se->entities_.object_[1]->pos_.GetH(), exp_values[i].h, 1E-2);
        EXPECT_EQ(se->entities_.object_[1]->pos_.GetLaneId(), exp_values[i].lane_id);
    }

    delete se;
}

static void clearanceParamDeclCallback(void*)
{
    static int counter  = 0;
    bool       value[2] = {false, true};

    if (counter < 2)
    {
        ScenarioReader::parameters.setParameterValue("OppositeLanes", value[counter]);
    }

    counter++;
}

TEST(RelativeClearanceTest, TestRelativeClearanceOppositeLane)
{
    double dt = 0.05;

    RegisterParameterDeclarationCallback(clearanceParamDeclCallback, 0);

    for (int i = 0; i < 2; i++)
    {
        ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/relative_clearance_oppositLane.xosc");
        ASSERT_NE(se, nullptr);

        while (se->getSimulationTime() < 1.6 - SMALL_NUMBER)
        {
            se->step(dt);
            se->prepareGroundTruth(dt);
        }
        if (i == 0)
        {
            ASSERT_NEAR(se->entities_.object_[2]->pos_.GetT(), 0.150, 1E-3);
            ASSERT_EQ(se->entities_.object_[2]->GetName(), "TargetRef");
        }

        while (se->getSimulationTime() < 5.75 - SMALL_NUMBER)
        {
            se->step(dt);
            se->prepareGroundTruth(dt);
        }

        if (i == 1)
        {
            ASSERT_NEAR(se->entities_.object_[2]->pos_.GetT(), 0.298, 1E-3);
            ASSERT_EQ(se->entities_.object_[2]->GetName(), "TargetRef");
        }

        delete se;
    }

    RegisterParameterDeclarationCallback(nullptr, 0);
}

TEST(ControllerTest, TestLoomingControllerAdvanced)
{
    double          dt = 0.05;
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/loomingAdvancedTest.xosc");
    ASSERT_NE(se, nullptr);

    // Get handle to first controller, which we know is a looming controller
    ASSERT_EQ(se->scenarioReader->controller_[0]->GetType(), scenarioengine::Controller::Type::CONTROLLER_TYPE_LOOMING);

    ControllerLooming* ctrl = reinterpret_cast<ControllerLooming*>(se->scenarioReader->controller_[0]);
    ASSERT_NE(ctrl, nullptr);

    while (se->getSimulationTime() < 2.5 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_EQ(ctrl->getHasFarTan(), false);

    while (se->getSimulationTime() < 6.0 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_EQ(ctrl->getHasFarTan(), true);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetS(), 83.321507875, 1E-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetT(), -1.452957222, 1E-5);
    ASSERT_NEAR(se->entities_.object_[0]->pos_.GetH(), 5.7631188563, 1e-5);

    while (se->getSimulationTime() < 31.0 - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }
    EXPECT_EQ(ctrl->getHasFarTan(), false);

    delete se;
}

static void EvaluateRelativeSpeed(Object& trig_obj, Object& obj, TrigByTimeToCollision& t, double heading)
{
    // object in front of triggering object
    double obj_pos[2]      = {0.0, 0.0};
    double trig_obj_pos[2] = {0.0, 0.0};
    RotateVec2D(0.0, 0.0, heading, trig_obj_pos[0], trig_obj_pos[1]);
    trig_obj.pos_.SetInertiaPos(trig_obj_pos[0], trig_obj_pos[1], heading, false);
    RotateVec2D(100.0, 0.0, heading, obj_pos[0], obj_pos[1]);
    obj.pos_.SetInertiaPos(obj_pos[0], obj_pos[1], heading, false);

    trig_obj.SetSpeed(10.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    obj.SetSpeed(0.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, 10.0, 1e-3);

    obj.SetSpeed(-20.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, 3.33333, 1e-3);

    obj.SetSpeed(10.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    trig_obj.SetSpeed(50.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), true);
    EXPECT_NEAR(t.ttc_, 2.5, 1e-3);

    obj.SetSpeed(20.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    trig_obj.SetSpeed(10.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, -1.0, 1e-3);

    obj.SetSpeed(20.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    trig_obj.SetSpeed(-10.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, -1.0, 1e-3);

    // triggering object in front of object
    RotateVec2D(100.0, 0.0, heading, trig_obj_pos[0], trig_obj_pos[1]);
    trig_obj.pos_.SetInertiaPos(trig_obj_pos[0], trig_obj_pos[1], heading, false);
    RotateVec2D(0.0, 0.0, heading, obj_pos[0], obj_pos[1]);
    obj.pos_.SetInertiaPos(obj_pos[0], obj_pos[1], heading, false);

    trig_obj.SetSpeed(10.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    obj.SetSpeed(0.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, -1.0, 1e-3);

    obj.SetSpeed(-20.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, -1.0, 1e-3);

    obj.SetSpeed(10.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    trig_obj.SetSpeed(50.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, -1, 1e-3);

    obj.SetSpeed(20.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    trig_obj.SetSpeed(10.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, 10.0, 1e-3);

    obj.SetSpeed(20.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    trig_obj.SetSpeed(-10.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, 3.33333, 1e-3);

    // object in front of object and opposite direction
    RotateVec2D(0.0, 0.0, heading, trig_obj_pos[0], trig_obj_pos[1]);
    trig_obj.pos_.SetInertiaPos(trig_obj_pos[0], trig_obj_pos[1], heading, false);
    RotateVec2D(100.0, 0.0, heading, obj_pos[0], obj_pos[1]);
    obj.pos_.SetInertiaPos(obj_pos[0], obj_pos[1], heading + M_PI, false);

    trig_obj.SetSpeed(10.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    obj.SetSpeed(0.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, 10.0, 1e-3);

    obj.SetSpeed(-20.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, -1, 1e-3);

    obj.SetSpeed(10.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    trig_obj.SetSpeed(50.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), true);
    EXPECT_NEAR(t.ttc_, 1.66667, 1e-3);

    obj.SetSpeed(20.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    trig_obj.SetSpeed(10.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, 3.33333, 1e-3);

    obj.SetSpeed(20.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    trig_obj.SetSpeed(-10.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, 10.0, 1e-3);

    // Trig obj in front of object and opposite direction
    RotateVec2D(100.0, 0.0, heading, trig_obj_pos[0], trig_obj_pos[1]);
    trig_obj.pos_.SetInertiaPos(trig_obj_pos[0], trig_obj_pos[1], heading + M_PI, false);
    RotateVec2D(0.0, 0.0, heading, obj_pos[0], obj_pos[1]);
    obj.pos_.SetInertiaPos(obj_pos[0], obj_pos[1], heading, false);

    trig_obj.SetSpeed(10.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    obj.SetSpeed(0.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, 10.0, 1e-3);

    obj.SetSpeed(-20.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, -1.0, 1e-3);

    obj.SetSpeed(10.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    trig_obj.SetSpeed(50.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), true);
    EXPECT_NEAR(t.ttc_, 1.66667, 1e-3);

    obj.SetSpeed(20.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    trig_obj.SetSpeed(10.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, 3.33333, 1e-3);

    obj.SetSpeed(20.0);
    obj.SetVel(obj.GetSpeed() * cos(obj.pos_.GetH()), obj.GetSpeed() * sin(obj.pos_.GetH()), 0.0);
    trig_obj.SetSpeed(-10.0);
    trig_obj.SetVel(trig_obj.GetSpeed() * cos(trig_obj.pos_.GetH()), trig_obj.GetSpeed() * sin(trig_obj.pos_.GetH()), 0.0);
    EXPECT_EQ(t.CheckCondition(0.0), false);
    EXPECT_NEAR(t.ttc_, 10.0, 1e-3);
}

TEST(ConditionTest, TestTTC)
{
    Object trig_obj(Object::Type::VEHICLE);
    Object obj(Object::Type::VEHICLE);

    TrigByTimeToCollision t;
    t.object_                 = &obj;
    t.triggering_entity_rule_ = TrigByTimeToCollision::TriggeringEntitiesRule::ANY;
    t.triggering_entities_.entity_.push_back({&trig_obj});
    t.value_       = 3.0;
    t.freespace_   = false;
    t.cs_          = roadmanager::CoordinateSystem::CS_ENTITY;
    t.relDistType_ = roadmanager::RelativeDistanceType::REL_DIST_LONGITUDINAL;
    t.rule_        = Rule::LESS_OR_EQUAL;

    trig_obj.SetActive(true);
    obj.SetActive(true);

    EvaluateRelativeSpeed(trig_obj, obj, t, 0.0 * M_PI_4);
    EvaluateRelativeSpeed(trig_obj, obj, t, 1.0 * M_PI_4);
    EvaluateRelativeSpeed(trig_obj, obj, t, 2.0 * M_PI_4);
    EvaluateRelativeSpeed(trig_obj, obj, t, 3.0 * M_PI_4);
    EvaluateRelativeSpeed(trig_obj, obj, t, 4.0 * M_PI_4);
    EvaluateRelativeSpeed(trig_obj, obj, t, 5.0 * M_PI_4);
    EvaluateRelativeSpeed(trig_obj, obj, t, 6.0 * M_PI_4);
    EvaluateRelativeSpeed(trig_obj, obj, t, 7.0 * M_PI_4);
}

static void TTCAndLateralDistParamDeclCallback(void*)
{
    static int counter  = 0;
    double     value[2] = {0.2, 5.0};

    if (counter < 2)
    {
        ScenarioReader::parameters.setParameterValue("LateralDist", value[counter]);
    }

    counter++;
}

TEST(ConditionTest, TestTTCAndLateralDist)
{
    double dt = 0.05;

    RegisterParameterDeclarationCallback(TTCAndLateralDistParamDeclCallback, 0);
    for (int i = 0; i < 2; i++)
    {
        ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/ttc_condition.xosc");
        ASSERT_NE(se, nullptr);
        ASSERT_EQ(se->entities_.object_[0]->GetName(), "Ego");
        ASSERT_EQ(se->entities_.object_[1]->GetName(), "Target");

        while (se->getSimulationTime() < 2.0 - SMALL_NUMBER)
        {
            se->step(dt);
            se->prepareGroundTruth(dt);
        }

        if (i == 0)
        {
            EXPECT_NEAR(se->entities_.object_[0]->pos_.GetVelX(), 10.0, 1E-3);
        }
        else
        {
            EXPECT_NEAR(se->entities_.object_[0]->pos_.GetVelX(), 1.0, 1E-3);
        }

        delete se;
    }
    RegisterParameterDeclarationCallback(nullptr, 0);
}

TEST(ActionTest, TestRelativeLaneChangeAction)
{
    double dt = 0.1;

    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/relative_lane_change.xosc");
    ASSERT_NE(se, nullptr);
    ASSERT_EQ(se->entities_.object_[0]->GetName(), "Ego");
    ASSERT_EQ(se->entities_.object_[1]->GetName(), "Target1");
    ASSERT_EQ(se->entities_.object_[2]->GetName(), "Target2");
    ASSERT_EQ(se->entities_.object_[3]->GetName(), "Target3");

    while (se->getSimulationTime() < 4.5 + dt - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 109.483, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -4.425, 1E-3);
    EXPECT_NEAR(se->entities_.object_[1]->pos_.GetX(), 140.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[1]->pos_.GetY(), -8.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[2]->pos_.GetX(), 290.547, 1E-3);
    EXPECT_NEAR(se->entities_.object_[2]->pos_.GetY(), 11.7, 1E-3);
    EXPECT_NEAR(se->entities_.object_[3]->pos_.GetX(), 260.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[3]->pos_.GetY(), 8.0, 1E-3);

    while (se->getSimulationTime() < 8.5 + dt - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 188.965, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -11.5, 1E-3);
    EXPECT_NEAR(se->entities_.object_[1]->pos_.GetX(), 220.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[1]->pos_.GetY(), -8.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[2]->pos_.GetX(), 211.095, 1E-3);
    EXPECT_NEAR(se->entities_.object_[2]->pos_.GetY(), 4.425, 1E-3);
    EXPECT_NEAR(se->entities_.object_[3]->pos_.GetX(), 180.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[3]->pos_.GetY(), 8.0, 1E-3);

    delete se;
}

TEST(ActionTest, TestRelativeLaneOffsetAction)
{
    double dt = 0.1;

    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/relative_lane_offset.xosc");
    ASSERT_NE(se, nullptr);
    ASSERT_EQ(se->entities_.object_[0]->GetName(), "Ego");
    ASSERT_EQ(se->entities_.object_[1]->GetName(), "Target1");
    ASSERT_EQ(se->entities_.object_[2]->GetName(), "Target2");
    ASSERT_EQ(se->entities_.object_[3]->GetName(), "Target3");

    while (se->getSimulationTime() < 4.5 + dt - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 110.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -7.1, 1E-3);
    EXPECT_NEAR(se->entities_.object_[1]->pos_.GetX(), 140.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[1]->pos_.GetY(), -8.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[2]->pos_.GetX(), 290.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[2]->pos_.GetY(), 8.9, 1E-3);
    EXPECT_NEAR(se->entities_.object_[3]->pos_.GetX(), 260.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[3]->pos_.GetY(), 8.0, 1E-3);

    while (se->getSimulationTime() < 8.5 + dt - SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(dt);
    }

    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 190.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -3.5, 1E-3);
    EXPECT_NEAR(se->entities_.object_[1]->pos_.GetX(), 220.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[1]->pos_.GetY(), -8.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[2]->pos_.GetX(), 210.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[2]->pos_.GetY(), 4.1, 1E-3);
    EXPECT_NEAR(se->entities_.object_[3]->pos_.GetX(), 180.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[3]->pos_.GetY(), 8.0, 1E-3);

    delete se;
}

TEST(ActionTest, TestRelativeLanePosition)
{
    const int    n         = 8;
    const double pos[n][5] = {{1.5, 75.833, -5.250, 78.730, -2.332},
                              {4.2, 158.333, -5.250, 158.369, -8.168},
                              {7.0, 243.889, -5.250, 245.441, -2.652},
                              {9.6, 323.333, -5.250, 323.369, -8.168},
                              {12.5, 451.667, 5.250, 450.115, 2.652},
                              {15.2, 369.167, 5.250, 367.615, 7.848},
                              {17.8, 289.722, 5.250, 289.695, 2.690},
                              {20.5, 207.222, 5.250, 207.195, 7.810}};

    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/relative_lane_pos_trajectories.xosc");
    ASSERT_NE(se, nullptr);
    ASSERT_EQ(se->entities_.object_[0]->GetName(), "Ego");
    ASSERT_EQ(se->entities_.object_[1]->GetName(), "Target");

    se->step(0.0);
    se->prepareGroundTruth(0.0);

    double dt = 0.1;
    for (int i = 0; i < n; i++)
    {
        while (se->getSimulationTime() < pos[i][0] - SMALL_NUMBER)
        {
            se->step(dt);
            se->prepareGroundTruth(dt);
        }
        EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), pos[i][1], 1E-3);
        EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), pos[i][2], 1E-3);
        EXPECT_NEAR(se->entities_.object_[1]->pos_.GetX(), pos[i][3], 1E-3);
        EXPECT_NEAR(se->entities_.object_[1]->pos_.GetY(), pos[i][4], 1E-3);
    }

    delete se;
}

TEST(ActionTest, TestRelativeLaneOffsetPosition)
{
    const int    n         = 8;
    const double pos[n][5] = {{1.5, 75.833, -5.250, 78.730, -2.332},
                              {4.2, 158.333, -5.250, 158.369, -8.168},
                              {7.0, 243.889, -5.250, 245.441, -2.652},
                              {9.6, 323.333, -5.250, 323.369, -8.168},
                              {12.5, 451.667, 5.250, 450.115, 2.652},
                              {15.2, 369.167, 5.250, 367.615, 7.848},
                              {17.8, 289.722, 5.250, 289.695, 2.690},
                              {20.5, 207.222, 5.250, 207.195, 7.810}};

    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/relative_lane_pos_offset_trajectories.xosc");
    ASSERT_NE(se, nullptr);
    ASSERT_EQ(se->entities_.object_[0]->GetName(), "Ego");
    ASSERT_EQ(se->entities_.object_[1]->GetName(), "Target");

    se->step(0.0);
    se->prepareGroundTruth(0.0);

    double dt = 0.1;
    for (int i = 0; i < n; i++)
    {
        while (se->getSimulationTime() < pos[i][0] - SMALL_NUMBER)
        {
            se->step(dt);
            se->prepareGroundTruth(dt);
        }
        EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), pos[i][1], 1E-3);
        EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), pos[i][2], 1E-3);
        EXPECT_NEAR(se->entities_.object_[1]->pos_.GetX(), pos[i][3], 1E-3);
        EXPECT_NEAR(se->entities_.object_[1]->pos_.GetY(), pos[i][4], 1E-3);
    }

    delete se;
}

TEST(PositionTest, TestPositionMode)
{
    double dt = 0.1;

    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/positioning_slope_up_leaning_right.xosc");
    ASSERT_NE(se, nullptr);
    EXPECT_EQ(se->entities_.object_[0]->GetName(), "Ego");

    se->step(dt);
    se->prepareGroundTruth(dt);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 10.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -3.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetZ(), -0.268, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetH(), 0.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetP(), 6.184, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetR(), 0.4, 1E-3);

    se->entities_.object_[0]->pos_.SetInertiaPosMode(35.0,
                                                     -3.0,
                                                     0.0,
                                                     1.0,
                                                     0.0,
                                                     0.0,
                                                     roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL |
                                                         roadmanager::Position::PosMode::R_REL | roadmanager::Position::PosMode::P_REL);

    se->step(dt);
    se->prepareGroundTruth(dt);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 35.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -3.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetZ(), 2.232, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetH(), 0.993, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetP(), 5.894, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetR(), 0.136, 1E-3);

    se->entities_.object_[0]->pos_.SetInertiaPosMode(30.0,
                                                     -2.0,
                                                     1.0,
                                                     -0.5,
                                                     0.3,
                                                     0.5,
                                                     roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS |
                                                         roadmanager::Position::PosMode::R_REL | roadmanager::Position::PosMode::P_ABS);

    se->step(dt);
    se->prepareGroundTruth(dt);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 30.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -2.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetZ(), 3.154, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetH(), 5.932, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetP(), 0.467, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetR(), 0.893, 1E-3);

    se->entities_.object_[0]->pos_.SetInertiaPosMode(30.0,
                                                     -2.0,
                                                     0.0,
                                                     -0.5,
                                                     0.3,
                                                     0.0,
                                                     roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS |
                                                         roadmanager::Position::PosMode::R_REL | roadmanager::Position::PosMode::P_REL);

    se->step(dt);
    se->prepareGroundTruth(dt);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 30.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -2.0, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetZ(), 2.154, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetH(), 5.947, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetP(), 0.374, 1E-3);
    EXPECT_NEAR(se->entities_.object_[0]->pos_.GetR(), 0.430, 1E-3);

    delete se;
}

TEST(PositionTest, TestPositionTypes)
{
    double dt = 0.1;

    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/position_types.xosc");
    ASSERT_NE(se, nullptr);

    se->step(0.0);
    se->prepareGroundTruth(0.0);
    scenarioengine::Entities* entities = &se->entities_;

    ASSERT_NE(entities, nullptr);

    EXPECT_EQ(entities->object_.size(), 8);
    EXPECT_EQ(entities->object_[0]->GetName(), "Car0");
    EXPECT_EQ(entities->object_[1]->GetName(), "Car1");
    EXPECT_EQ(entities->object_[2]->GetName(), "Car2");
    EXPECT_EQ(entities->object_[3]->GetName(), "Car3");

    // Check lane position in lane along s-axis
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 14.329, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), 200.519, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetZ(), 10.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[0]->pos_.GetH(), 3.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[0]->pos_.GetP(), 5.991), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[0]->pos_.GetR(), 0.0), 0.0, 1E-3);

    // Check lane position in opposite lane
    EXPECT_NEAR(entities->object_[1]->pos_.GetX(), 9.006, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetY(), 198.052, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetZ(), 11.495, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[1]->pos_.GetH(), 6.192), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[1]->pos_.GetP(), 0.289), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[1]->pos_.GetR(), 6.283), 0.0, 1E-3);

    // Check relative lane position in lane along s-axis
    EXPECT_NEAR(entities->object_[2]->pos_.GetX(), entities->object_[0]->pos_.GetX(), 1E-3);
    EXPECT_NEAR(entities->object_[2]->pos_.GetY(), entities->object_[0]->pos_.GetY(), 1E-3);
    EXPECT_NEAR(entities->object_[2]->pos_.GetZ(), entities->object_[0]->pos_.GetZ(), 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[2]->pos_.GetH(), entities->object_[0]->pos_.GetH()), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[2]->pos_.GetP(), entities->object_[0]->pos_.GetP()), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[2]->pos_.GetR(), entities->object_[0]->pos_.GetR()), 0.0, 1E-3);

    // Check relative lane position in opposite lane
    EXPECT_NEAR(entities->object_[3]->pos_.GetX(), entities->object_[1]->pos_.GetX(), 1E-3);
    EXPECT_NEAR(entities->object_[3]->pos_.GetY(), entities->object_[1]->pos_.GetY(), 1E-3);
    EXPECT_NEAR(entities->object_[3]->pos_.GetZ(), entities->object_[1]->pos_.GetZ(), 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[3]->pos_.GetH(), entities->object_[1]->pos_.GetH()), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[3]->pos_.GetP(), entities->object_[1]->pos_.GetP()), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[3]->pos_.GetR(), entities->object_[1]->pos_.GetR()), 0.0, 1E-3);

    // Check corresponding variants with RoadPosition instead of LanePosition
    for (unsigned int i = 0; i < 4; i++)
    {
        EXPECT_NEAR(entities->object_[4 + i]->pos_.GetX(), entities->object_[i]->pos_.GetX(), 1E-3);
        EXPECT_NEAR(entities->object_[4 + i]->pos_.GetY(), entities->object_[i]->pos_.GetY(), 1E-3);
        EXPECT_NEAR(entities->object_[4 + i]->pos_.GetZ(), entities->object_[i]->pos_.GetZ(), 1E-3);
        EXPECT_NEAR(GetAngleDifference(entities->object_[4 + i]->pos_.GetH(), entities->object_[i]->pos_.GetH()), 0.0, 1E-3);
        EXPECT_NEAR(GetAngleDifference(entities->object_[4 + i]->pos_.GetP(), entities->object_[i]->pos_.GetP()), 0.0, 1E-3);
        EXPECT_NEAR(GetAngleDifference(entities->object_[4 + i]->pos_.GetR(), entities->object_[i]->pos_.GetR()), 0.0, 1E-3);
    }

    se->step(dt);
    se->prepareGroundTruth(dt);

    delete se;
}

TEST(ClothoidSplineTest, TestTrajectoryShape)
{
    double dt = 0.05;

    ScenarioEngine* se = new ScenarioEngine("../../../resources/xosc/lane-change_clothoid_spline_based_trajectory.xosc");
    ASSERT_NE(se, nullptr);
    se->step(0.0);
    se->prepareGroundTruth(0.0);

    scenarioengine::Entities* entities = &se->entities_;
    ASSERT_NE(entities, nullptr);
    ASSERT_EQ(entities->object_.size(), 1);

    while (se->getSimulationTime() < 26.4)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }

    // Check car position at given time at end phase of the scenario
    // Correct position indicates all trajectories have been evaluated correctly
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 242.101, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), 1.085, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[0]->pos_.GetH(), 6.262), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[0]->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[0]->pos_.GetR(), 0.0), 0.0, 1E-3);

    delete se;
}

TEST(RelativePositionRouting, TestRelativePositionWithRoutes)
{
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/relative_pos_over_intersection.xosc");
    ASSERT_NE(se, nullptr);
    se->step(0.0);
    se->prepareGroundTruth(0.0);
    scenarioengine::Entities* entities = &se->entities_;
    ASSERT_NE(entities, nullptr);
    ASSERT_EQ(entities->object_.size(), 6);

    // Check car position at given time at end phase of the scenario
    // Correct position indicates all trajectories have been evaluated correctly
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 40.210, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), -58.518, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[0]->pos_.GetH(), 1.793), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[0]->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[0]->pos_.GetR(), 0.0), 0.0, 1E-3);

    EXPECT_NEAR(entities->object_[1]->pos_.GetX(), 19.690, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetY(), 39.310, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[1]->pos_.GetH(), 1.752), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[1]->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[1]->pos_.GetR(), 0.0), 0.0, 1E-3);

    EXPECT_NEAR(entities->object_[2]->pos_.GetX(), -16.886, 1E-3);
    EXPECT_NEAR(entities->object_[2]->pos_.GetY(), -7.189, 1E-3);
    EXPECT_NEAR(entities->object_[2]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[2]->pos_.GetH(), 3.287), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[2]->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[2]->pos_.GetR(), 0.0), 0.0, 1E-3);

    EXPECT_NEAR(entities->object_[3]->pos_.GetX(), 44.132, 1E-3);
    EXPECT_NEAR(entities->object_[3]->pos_.GetY(), -0.885, 1E-3);
    EXPECT_NEAR(entities->object_[3]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[3]->pos_.GetH(), 0.193), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[3]->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[3]->pos_.GetR(), 0.0), 0.0, 1E-3);

    EXPECT_NEAR(entities->object_[4]->pos_.GetX(), 16.064, 1E-3);
    EXPECT_NEAR(entities->object_[4]->pos_.GetY(), 58.981, 1E-3);
    EXPECT_NEAR(entities->object_[4]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[4]->pos_.GetH(), 1.754), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[4]->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[4]->pos_.GetR(), 0.0), 0.0, 1E-3);

    EXPECT_NEAR(entities->object_[5]->pos_.GetX(), 46.198, 1E-3);
    EXPECT_NEAR(entities->object_[5]->pos_.GetY(), -88.003, 1E-3);
    EXPECT_NEAR(entities->object_[5]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[5]->pos_.GetH(), 1.742), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[5]->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[5]->pos_.GetR(), 0.0), 0.0, 1E-3);

    while (se->getSimulationTime() < 1.0 + SMALL_NUMBER)
    {
        se->step(0.5);
    }

    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 40.210, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), -58.518, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[0]->pos_.GetH(), 1.793), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[0]->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[0]->pos_.GetR(), 0.0), 0.0, 1E-3);

    EXPECT_NEAR(entities->object_[1]->pos_.GetX(), 16.248, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetY(), 38.678, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[1]->pos_.GetH(), 1.752), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[1]->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[1]->pos_.GetR(), 0.0), 0.0, 1E-3);

    EXPECT_NEAR(entities->object_[2]->pos_.GetX(), -16.378, 1E-3);
    EXPECT_NEAR(entities->object_[2]->pos_.GetY(), -10.652, 1E-3);
    EXPECT_NEAR(entities->object_[2]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[2]->pos_.GetH(), 0.146), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[2]->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[2]->pos_.GetR(), 0.0), 0.0, 1E-3);

    EXPECT_NEAR(entities->object_[3]->pos_.GetX(), 43.461, 1E-3);
    EXPECT_NEAR(entities->object_[3]->pos_.GetY(), 2.550, 1E-3);
    EXPECT_NEAR(entities->object_[3]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[3]->pos_.GetH(), 0.193), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[3]->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[3]->pos_.GetR(), 0.0), 0.0, 1E-3);

    EXPECT_NEAR(entities->object_[4]->pos_.GetX(), 16.064, 1E-3);
    EXPECT_NEAR(entities->object_[4]->pos_.GetY(), 58.981, 1E-3);
    EXPECT_NEAR(entities->object_[4]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[4]->pos_.GetH(), 1.754), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[4]->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[4]->pos_.GetR(), 0.0), 0.0, 1E-3);

    EXPECT_NEAR(entities->object_[5]->pos_.GetX(), 46.0261, 1E-3);
    EXPECT_NEAR(entities->object_[5]->pos_.GetY(), -87.017, 1E-3);
    EXPECT_NEAR(entities->object_[5]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[5]->pos_.GetH(), 1.744), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[5]->pos_.GetP(), 0.0), 0.0, 1E-3);
    EXPECT_NEAR(GetAngleDifference(entities->object_[5]->pos_.GetR(), 0.0), 0.0, 1E-3);

    delete se;
}

TEST(StoryboardTest, TestStoryboardElementNaming)
{
    StoryBoard sb;

    // Create a storyboard hierarchy as:
    // story1
    //    act1
    //       mg1
    //       mg1 (duplicate maneuvergroup name)
    //    act1   (duplicate act name)
    //       mg1 (duplicate maneuvergroup name)
    // story2
    //    act3
    //       mg1 (duplicate maneuvergroup name)
    //       mg4

    Story* story1 = new Story(&sb);
    Story* story2 = new Story(&sb);
    story1->SetName("story1");
    story2->SetName("story2");
    sb.story_.push_back(story1);
    sb.story_.push_back(story2);

    Act* act1 = new Act(story1);
    Act* act2 = new Act(story1);
    Act* act3 = new Act(story2);
    act1->SetName("act1");
    act2->SetName("act1");  // duplicate name
    act3->SetName("act3");
    story1->act_.push_back(act1);
    story1->act_.push_back(act2);
    story2->act_.push_back(act3);

    ManeuverGroup* mg1 = new ManeuverGroup(act1);
    ManeuverGroup* mg2 = new ManeuverGroup(act2);
    ManeuverGroup* mg3 = new ManeuverGroup(act3);
    ManeuverGroup* mg4 = new ManeuverGroup(act3);
    mg1->SetName("mg1");
    mg2->SetName("mg1");  // duplicate name
    mg3->SetName("mg1");  // duplicate name
    mg4->SetName("mg4");
    act1->maneuverGroup_.push_back(mg1);
    act2->maneuverGroup_.push_back(mg2);
    act3->maneuverGroup_.push_back(mg3);
    act3->maneuverGroup_.push_back(mg4);

    EXPECT_EQ(mg1->GetName(), "mg1");
    EXPECT_EQ(mg2->GetName(), "mg1");
    EXPECT_EQ(mg3->GetName(), "mg1");
    EXPECT_EQ(mg4->GetName(), "mg4");
    EXPECT_EQ(act1->GetName(), "act1");
    EXPECT_EQ(act2->GetName(), "act1");
    EXPECT_EQ(act3->GetName(), "act3");

    std::vector<StoryBoardElement*> elements = sb.FindChildByTypeAndName(StoryBoardElement::ElementType::MANEUVER_GROUP, "mg1");
    EXPECT_EQ(elements.size(), 3);

    elements = sb.FindChildByTypeAndName(StoryBoardElement::ElementType::MANEUVER_GROUP, "act1::mg1");
    EXPECT_EQ(elements.size(), 2);

    elements = sb.FindChildByTypeAndName(StoryBoardElement::ElementType::MANEUVER_GROUP, "story1::act1::mg1");
    EXPECT_EQ(elements.size(), 2);

    elements = sb.FindChildByTypeAndName(StoryBoardElement::ElementType::MANEUVER_GROUP, "story2::act3::mg1");
    EXPECT_EQ(elements.size(), 1);

    elements = sb.FindChildByTypeAndName(StoryBoardElement::ElementType::MANEUVER_GROUP, "act1");
    EXPECT_EQ(elements.size(), 0);

    elements = sb.FindChildByTypeAndName(StoryBoardElement::ElementType::ACT, "act1");
    EXPECT_EQ(elements.size(), 2);

    elements = sb.FindChildByTypeAndName(StoryBoardElement::ElementType::MANEUVER_GROUP, "");
    EXPECT_EQ(elements.size(), 0);
}

TEST(Friction, TestFrictionPerWheel)
{
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/friction_and_lane_change_edge_case.xosc");
    ASSERT_NE(se, nullptr);
    se->step(0.0);
    se->prepareGroundTruth(0.0);

    scenarioengine::Entities* entities = &se->entities_;
    ASSERT_NE(entities, nullptr);
    ASSERT_EQ(entities->object_.size(), 1);

    ScenarioGateway* gw = se->getScenarioGateway();

    // Check friction per wheel at some key time stamps

    // time = 0.0
    ObjectStateStruct* state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->info.wheel_data[0].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].friction_coefficient, 1.0, 1E-3);

    while (se->getSimulationTime() < 1.8 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->info.wheel_data[0].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].friction_coefficient, 1.0, 1E-3);

    while (se->getSimulationTime() < 1.9 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->info.wheel_data[0].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].friction_coefficient, 0.8, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].friction_coefficient, 1.0, 1E-3);

    while (se->getSimulationTime() < 3.9 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->info.wheel_data[0].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].friction_coefficient, 1.0, 1E-3);

    while (se->getSimulationTime() < 4.0 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->info.wheel_data[0].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].friction_coefficient, 0.8, 1E-3);

    while (se->getSimulationTime() < 12.1 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->info.wheel_data[0].friction_coefficient, 0.4, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].friction_coefficient, 0.4, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].friction_coefficient, 1.0, 1E-3);

    while (se->getSimulationTime() < 13.7 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->info.wheel_data[0].friction_coefficient, 0.4, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].friction_coefficient, 0.4, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].friction_coefficient, 0.4, 1E-3);

    while (se->getSimulationTime() < 20.0 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->info.wheel_data[0].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].friction_coefficient, 1.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].friction_coefficient, 1.0, 1E-3);

    delete se;
}

TEST(WheelData, TestWheelData)
{
    ScenarioEngine* se = new ScenarioEngine("../../../resources/xosc/lane_change_crest.xosc");
    ASSERT_NE(se, nullptr);
    se->step(0.0);
    se->prepareGroundTruth(0.0);

    scenarioengine::Entities* entities = &se->entities_;
    ASSERT_NE(entities, nullptr);
    ASSERT_EQ(entities->object_.size(), 3);

    ScenarioGateway* gw = se->getScenarioGateway();

    // Check friction per wheel at some key time stamps

    // time = 0.0
    ObjectStateStruct* state = &gw->objectState_[0]->state_;
    EXPECT_EQ(state->info.wheel_data[0].axle, 0);
    EXPECT_EQ(state->info.wheel_data[1].axle, 0);
    EXPECT_EQ(state->info.wheel_data[2].axle, 1);
    EXPECT_EQ(state->info.wheel_data[3].axle, 1);

    EXPECT_NEAR(state->info.wheel_data[0].h, 0.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].h, 0.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].h, 0.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].h, 0.0, 1E-3);

    EXPECT_EQ(state->info.wheel_data[0].index, 0);
    EXPECT_EQ(state->info.wheel_data[1].index, 1);
    EXPECT_EQ(state->info.wheel_data[2].index, 0);
    EXPECT_EQ(state->info.wheel_data[3].index, 1);

    EXPECT_NEAR(state->info.wheel_data[0].p, 0.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].p, 0.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].p, 0.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].p, 0.0, 1E-3);

    EXPECT_NEAR(state->info.wheel_data[0].x, 2.98, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].x, 2.98, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].x, 0.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].x, 0.0, 1E-3);

    EXPECT_NEAR(state->info.wheel_data[0].y, -0.840, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].y, 0.840, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].y, -0.840, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].y, 0.840, 1E-3);

    EXPECT_NEAR(state->info.wheel_data[0].z, 0.4, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].z, 0.4, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].z, 0.4, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].z, 0.4, 1E-3);

    while (se->getSimulationTime() < 5.8 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.1);
    }

    // check overtaking car, some wheel heading expected
    state = &gw->objectState_[2]->state_;

    EXPECT_EQ(state->info.wheel_data[0].axle, 0);
    EXPECT_EQ(state->info.wheel_data[1].axle, 0);
    EXPECT_EQ(state->info.wheel_data[2].axle, 1);
    EXPECT_EQ(state->info.wheel_data[3].axle, 1);

    EXPECT_NEAR(state->info.wheel_data[0].h, 0.054, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].h, 0.054, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].h, 0.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].h, 0.0, 1E-3);

    EXPECT_EQ(state->info.wheel_data[0].index, 0);
    EXPECT_EQ(state->info.wheel_data[1].index, 1);
    EXPECT_EQ(state->info.wheel_data[2].index, 0);
    EXPECT_EQ(state->info.wheel_data[3].index, 1);

    EXPECT_NEAR(state->info.wheel_data[0].p, 1.989, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].p, 1.989, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].p, 1.989, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].p, 1.989, 1E-3);

    EXPECT_NEAR(state->info.wheel_data[0].x, 2.98, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].x, 2.98, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].x, 0.0, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].x, 0.0, 1E-3);

    EXPECT_NEAR(state->info.wheel_data[0].y, -0.840, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].y, 0.840, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].y, -0.840, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].y, 0.840, 1E-3);

    // ensure z=0 even when the car is on a crest
    EXPECT_NEAR(state->info.wheel_data[0].z, 0.4, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[1].z, 0.4, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[2].z, 0.4, 1E-3);
    EXPECT_NEAR(state->info.wheel_data[3].z, 0.4, 1E-3);

    delete se;
}

TEST(LaneChange, TestLaneChangeEdgeCase)
{
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/friction_and_lane_change_edge_case.xosc");
    ASSERT_NE(se, nullptr);
    se->step(0.0);
    se->prepareGroundTruth(0.0);

    scenarioengine::Entities* entities = &se->entities_;
    ASSERT_NE(entities, nullptr);
    ASSERT_EQ(entities->object_.size(), 1);

    ScenarioGateway* gw = se->getScenarioGateway();

    // Check expected position and orientation at some specific time stamps
    while (se->getSimulationTime() < 21.0 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    ObjectStateStruct* state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->pos.GetX(), 121.059, 1E-3);
    EXPECT_NEAR(state->pos.GetY(), 5.245, 1E-3);
    EXPECT_NEAR(state->pos.GetZ(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetH(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetP(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetR(), 0.000, 1E-3);

    while (se->getSimulationTime() < 21.6 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->pos.GetX(), 121.604, 1E-3);
    EXPECT_NEAR(state->pos.GetY(), 4.992, 1E-3);
    EXPECT_NEAR(state->pos.GetZ(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetH(), 5.609, 1E-3);
    EXPECT_NEAR(state->pos.GetP(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetR(), 0.000, 1E-3);

    while (se->getSimulationTime() < 22.3 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->pos.GetX(), 121.802, 1E-3);
    EXPECT_NEAR(state->pos.GetY(), 3.987, 1E-3);
    EXPECT_NEAR(state->pos.GetZ(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetH(), 4.712, 1E-3);
    EXPECT_NEAR(state->pos.GetP(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetR(), 0.000, 1E-3);

    while (se->getSimulationTime() < 22.3 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->pos.GetX(), 121.802, 1E-3);
    EXPECT_NEAR(state->pos.GetY(), 3.987, 1E-3);
    EXPECT_NEAR(state->pos.GetZ(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetH(), 4.712, 1E-3);
    EXPECT_NEAR(state->pos.GetP(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetR(), 0.000, 1E-3);

    while (se->getSimulationTime() < 25.7 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->pos.GetX(), 123.581, 1E-3);
    EXPECT_NEAR(state->pos.GetY(), 1.751, 1E-3);
    EXPECT_NEAR(state->pos.GetZ(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetH(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetP(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetR(), 0.000, 1E-3);

    while (se->getSimulationTime() < 26.7 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->pos.GetX(), 124.400, 1E-3);
    EXPECT_NEAR(state->pos.GetY(), 2.263, 1E-3);
    EXPECT_NEAR(state->pos.GetZ(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetH(), 1.103, 1E-3);
    EXPECT_NEAR(state->pos.GetP(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetR(), 0.000, 1E-3);

    while (se->getSimulationTime() < 28.4 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->pos.GetX(), 124.424, 1E-3);
    EXPECT_NEAR(state->pos.GetY(), 4.313, 1E-3);
    EXPECT_NEAR(state->pos.GetZ(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetH(), 1.571, 1E-3);
    EXPECT_NEAR(state->pos.GetP(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetR(), 0.000, 1E-3);

    while (se->getSimulationTime() < 29.5 + SMALL_NUMBER)
    {
        se->step(0.1);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->pos.GetX(), 125.203, 1E-3);
    EXPECT_NEAR(state->pos.GetY(), 5.250, 1E-3);
    EXPECT_NEAR(state->pos.GetZ(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetH(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetP(), 0.000, 1E-3);
    EXPECT_NEAR(state->pos.GetR(), 0.000, 1E-3);

    delete se;
}

TEST(Trajectory, TestOrientationInterpolation)
{
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/bike_tilt_smooth.xosc");
    const double    dt = 0.05;
    ASSERT_NE(se, nullptr);
    se->step(0.0);
    se->prepareGroundTruth(0.0);

    scenarioengine::Entities* entities = &se->entities_;
    ASSERT_NE(entities, nullptr);
    ASSERT_EQ(entities->object_.size(), 1);

    ScenarioGateway* gw = se->getScenarioGateway();

    // Check expected position and orientation at some specific time stamps
    while (se->getSimulationTime() < 2.95 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    ObjectStateStruct* state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->pos.GetX(), 120.83, 1E-2);
    EXPECT_NEAR(state->pos.GetY(), -2.54, 1E-2);
    EXPECT_NEAR(state->pos.GetZ(), 0.00, 1E-2);
    EXPECT_NEAR(state->pos.GetH(), 0.00, 1E-2);
    EXPECT_NEAR(state->pos.GetP(), 0.00, 1E-2);
    EXPECT_NEAR(state->pos.GetR(), 0.00, 1E-2);
    EXPECT_NEAR(state->info.speed, 6.94, 1E-2);

    while (se->getSimulationTime() < 3.85 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->pos.GetX(), 127.08, 1E-2);
    EXPECT_NEAR(state->pos.GetY(), -2.54, 1E-2);
    EXPECT_NEAR(state->pos.GetZ(), 0.00, 1E-2);
    EXPECT_NEAR(state->pos.GetH(), 0.00, 1E-2);
    EXPECT_NEAR(state->pos.GetP(), 0.00, 1E-2);
    EXPECT_NEAR(state->pos.GetR(), 1.31, 1E-2);
    EXPECT_NEAR(state->info.speed, 6.94, 1E-2);

    while (se->getSimulationTime() < 4.75 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->pos.GetX(), 130.61, 1E-2);
    EXPECT_NEAR(state->pos.GetY(), -2.54, 1E-2);
    EXPECT_NEAR(state->pos.GetZ(), 0.00, 1E-2);
    EXPECT_NEAR(state->pos.GetH(), 0.00, 1E-2);
    EXPECT_NEAR(state->pos.GetP(), 0.00, 1E-2);
    EXPECT_NEAR(state->pos.GetR(), 1.57, 1E-2);
    EXPECT_NEAR(state->info.speed, 0.54, 1E-2);

    while (se->getSimulationTime() < 4.95 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    state = &gw->objectState_[0]->state_;
    EXPECT_NEAR(state->pos.GetX(), 130.62, 1E-2);
    EXPECT_NEAR(state->pos.GetY(), -2.54, 1E-2);
    EXPECT_NEAR(state->pos.GetZ(), 0.00, 1E-2);
    EXPECT_NEAR(state->pos.GetH(), 0.00, 1E-2);
    EXPECT_NEAR(state->pos.GetP(), 0.00, 1E-2);
    EXPECT_NEAR(state->pos.GetR(), 1.57, 1E-2);
    EXPECT_NEAR(state->info.speed, 0.0, 1E-2);

    delete se;
}

TEST(PositionTest, TestRelativePositionPhases)
{
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/relative_speed_phases.xosc");
    const double    dt = 0.05;
    ASSERT_NE(se, nullptr);
    se->step(0.0);
    se->prepareGroundTruth(0.0);

    scenarioengine::Entities* entities = &se->entities_;
    ASSERT_NE(entities, nullptr);
    ASSERT_EQ(entities->object_.size(), 2);

    // Check expected position and orientation at some specific time stamps
    while (se->getSimulationTime() < 7.15 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }

    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 129.500, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), -1.535, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetH(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetP(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetR(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->GetSpeed(), 20.0, 1E-3);

    EXPECT_NEAR(entities->object_[1]->pos_.GetX(), 53.688, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetY(), -1.535, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetH(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetP(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetR(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[1]->GetSpeed(), 18.613, 1E-3);

    while (se->getSimulationTime() < 7.2 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }

    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 130.512, 1E-3);
    EXPECT_NEAR(entities->object_[0]->GetSpeed(), 20.250, 1E-3);

    EXPECT_NEAR(entities->object_[1]->pos_.GetX(), 54.628, 1E-3);
    EXPECT_NEAR(entities->object_[1]->GetSpeed(), 18.813, 1E-3);

    while (se->getSimulationTime() < 7.2 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }

    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 130.512, 1E-3);
    EXPECT_NEAR(entities->object_[0]->GetSpeed(), 20.250, 1E-3);

    EXPECT_NEAR(entities->object_[1]->pos_.GetX(), 54.628, 1E-3);
    EXPECT_NEAR(entities->object_[1]->GetSpeed(), 18.813, 1E-3);

    while (se->getSimulationTime() < 9.95 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }

    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 203.750, 1E-3);
    EXPECT_NEAR(entities->object_[0]->GetSpeed(), 30.0, 1E-3);

    EXPECT_NEAR(entities->object_[1]->pos_.GetX(), 121.741, 1E-3);
    EXPECT_NEAR(entities->object_[1]->GetSpeed(), 29.538, 1E-3);

    while (se->getSimulationTime() < 10.95 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }

    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 233.750, 1E-3);
    EXPECT_NEAR(entities->object_[0]->GetSpeed(), 30.0, 1E-3);

    EXPECT_NEAR(entities->object_[1]->pos_.GetX(), 151.666, 1E-3);
    EXPECT_NEAR(entities->object_[1]->GetSpeed(), 30.0, 1E-3);

    while (se->getSimulationTime() < 11.45 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }

    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 249.438, 1E-3);
    EXPECT_NEAR(entities->object_[0]->GetSpeed(), 32.500, 1E-3);

    EXPECT_NEAR(entities->object_[1]->pos_.GetX(), 167.216, 1E-3);
    EXPECT_NEAR(entities->object_[1]->GetSpeed(), 32.0, 1E-3);

    while (se->getSimulationTime() < 12.20 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }

    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 275.125, 1E-3);
    EXPECT_NEAR(entities->object_[0]->GetSpeed(), 35.0, 1E-3);

    EXPECT_NEAR(entities->object_[1]->pos_.GetX(), 192.416, 1E-3);
    EXPECT_NEAR(entities->object_[1]->GetSpeed(), 35.0, 1E-3);

    while (se->getSimulationTime() < 18.40 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }

    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 431.750, 1E-3);
    EXPECT_NEAR(entities->object_[0]->GetSpeed(), 0.500, 1E-3);

    EXPECT_NEAR(entities->object_[1]->pos_.GetX(), 367.154, 1E-3);
    EXPECT_NEAR(entities->object_[1]->GetSpeed(), 10.850, 1E-3);

    while (se->getSimulationTime() < 19.95 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }

    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 431.750, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), -1.535, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetH(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetP(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetR(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->GetSpeed(), 0.0, 1E-3);

    EXPECT_NEAR(entities->object_[1]->pos_.GetX(), 375.291, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetY(), -1.535, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetH(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetP(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[1]->pos_.GetR(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[1]->GetSpeed(), 0.0, 1E-3);

    delete se;
}

TEST(ActionTest, TestInstantLaneChange)
{
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/instant_lane_change.xosc");
    const double    dt = 0.05;
    ASSERT_NE(se, nullptr);
    se->step(0.0);
    se->prepareGroundTruth(0.0);

    scenarioengine::Entities* entities = &se->entities_;
    ASSERT_NE(entities, nullptr);
    ASSERT_EQ(entities->object_.size(), 1);

    // Check expected position and orientation at some specific time stamps
    while (se->getSimulationTime() < 0.95 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 50.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), -1.535, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetH(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetP(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetR(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->GetSpeed(), 5.0, 1E-3);

    while (se->getSimulationTime() < 1.0 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 50.25, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), 1.535, 1E-3);

    while (se->getSimulationTime() < 1.95 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 55.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), 1.535, 1E-3);

    while (se->getSimulationTime() < 2.0 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 55.25, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), -1.535, 1E-3);

    while (se->getSimulationTime() < 2.95 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 60.00, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), -1.535, 1E-3);

    while (se->getSimulationTime() < 3.0 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 60.25, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), 1.535, 1E-3);

    delete se;
}

TEST(RouteingTest, TestPositionOffRoute)
{
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/route_detour.xosc", true);
    const double    dt = 0.05;
    ASSERT_NE(se, nullptr);
    se->step(0.0);
    se->prepareGroundTruth(0.0);

    scenarioengine::Entities* entities = &se->entities_;
    ASSERT_NE(entities, nullptr);
    ASSERT_EQ(entities->object_.size(), 1);

    // Check expected position and orientation at some specific time stamps
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 221.5, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), 45.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetH(), 1.5708, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetP(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetR(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->GetSpeed(), 40.0, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetTrackId(), 2);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.route_->currentPos_.GetS(), 25.0, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->OnRoute(), true);

    while (se->getSimulationTime() < 1.0 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 216.982, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), 86.012, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.GetTrackId(), 200);
    EXPECT_EQ(entities->object_[0]->pos_.GetLaneId(), -1);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetTrackId(), 201);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.route_->currentPos_.GetS(), 16.012, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->OnRoute(), false);

    while (se->getSimulationTime() < 3.15 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 132.565, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), 91.5, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.GetTrackId(), 3);
    EXPECT_EQ(entities->object_[0]->pos_.GetLaneId(), -1);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetTrackId(), 201);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.route_->currentPos_.GetS(), 21.5, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->OnRoute(), false);

    while (se->getSimulationTime() < 3.20 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 130.565, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), 91.5, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.GetTrackId(), 3);
    EXPECT_EQ(entities->object_[0]->pos_.GetLaneId(), -1);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetTrackId(), 4);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.route_->currentPos_.GetS(), 150.851, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->OnRoute(), false);

    while (se->getSimulationTime() < 5.75 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 28.565, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), 91.5, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.GetTrackId(), 3);
    EXPECT_EQ(entities->object_[0]->pos_.GetLaneId(), -1);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetTrackId(), 4);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.route_->currentPos_.GetS(), 252.851, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->OnRoute(), false);

    while (se->getSimulationTime() < 5.8 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 26.565, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), 91.5, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.GetTrackId(), 3);
    EXPECT_EQ(entities->object_[0]->pos_.GetLaneId(), -1);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetTrackId(), 302);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.route_->currentPos_.GetS(), 18.5, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->OnRoute(), false);

    while (se->getSimulationTime() < 8.0 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), -58.592, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), 83.938, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.GetTrackId(), 301);
    EXPECT_EQ(entities->object_[0]->pos_.GetLaneId(), -1);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetTrackId(), 302);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.route_->currentPos_.GetS(), 26.062, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->OnRoute(), false);

    while (se->getSimulationTime() < 8.4 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), -61.500, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), 68.564, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.GetTrackId(), 6);
    EXPECT_EQ(entities->object_[0]->pos_.GetLaneId(), -1);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetTrackId(), 6);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.route_->currentPos_.GetS(), 1.435, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->OnRoute(), true);

    while (se->getSimulationTime() < 11.25 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), -61.500, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), -45.435, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.GetTrackId(), 5);
    EXPECT_EQ(entities->object_[0]->pos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.GetS(), 25.435, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetTrackId(), 5);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.route_->currentPos_.GetS(), 25.435, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->OnRoute(), true);

    while (se->getSimulationTime() < 11.50 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), -61.500, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), -55.435, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.GetTrackId(), 5);
    EXPECT_EQ(entities->object_[0]->pos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.GetS(), 35.435, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetTrackId(), 5);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.route_->currentPos_.GetS(), 30.0, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->OnRoute(), false);

    while (se->getSimulationTime() < 11.60 + SMALL_NUMBER)
    {
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), -61.500, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), -59.435, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.GetTrackId(), 5);
    EXPECT_EQ(entities->object_[0]->pos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.GetS(), 39.435, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetTrackId(), 5);
    EXPECT_EQ(entities->object_[0]->pos_.route_->currentPos_.GetLaneId(), -1);
    EXPECT_NEAR(entities->object_[0]->pos_.route_->currentPos_.GetS(), 30.0, 1E-3);
    EXPECT_EQ(entities->object_[0]->pos_.route_->OnRoute(), false);

    delete se;
}

TEST(PositioningTest, TestElevationMapping)
{
    ScenarioEngine* se = new ScenarioEngine("../../../EnvironmentSimulator/Unittest/xosc/test_elevation_mapping.xosc", false);
    const double    dt = 0.1;
    ASSERT_NE(se, nullptr);
    se->step(0.0);
    se->prepareGroundTruth(0.0);

    scenarioengine::Entities* entities = &se->entities_;
    ASSERT_NE(entities, nullptr);
    ASSERT_EQ(entities->object_.size(), 1);

    // Check expected position and orientation at some specific time stamps
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 8.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), -1.75, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetZ(), 3.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetH(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetP(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetR(), 0.0, 1E-3);

    ControllerInteractive* ctrl = static_cast<ControllerInteractive*>(entities->object_[0]->GetController("interactiveDriver"));
    ASSERT_NE(ctrl, nullptr);

    // drive car forward, so that it reaches end of top road, dropping to the below road
    while (se->getSimulationTime() < 1.0 + SMALL_NUMBER)
    {
        ctrl->ReportKeyEvent(static_cast<int>(KeyType::KEY_Up), true);
        se->step(dt);
        se->prepareGroundTruth(0.0);
    }

    // Verify that car has moved to lower road
    EXPECT_NEAR(entities->object_[0]->pos_.GetX(), 11.3, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetY(), -1.75, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetZ(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetH(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetP(), 0.0, 1E-3);
    EXPECT_NEAR(entities->object_[0]->pos_.GetR(), 0.0, 1E-3);

    delete se;
}

TEST(StringIds, TestRoadStringIdsEdgeCases)
{
    pugi::xml_document  doc;
    pugi::xml_node      parent = doc.append_child("parent");
    pugi::xml_node      child1 = parent.append_child("road");
    pugi::xml_node      child2 = parent.append_child("road");
    pugi::xml_node      child3 = parent.append_child("road");
    pugi::xml_attribute attr1  = child1.append_attribute("id");
    pugi::xml_attribute attr2  = child2.append_attribute("id");
    pugi::xml_attribute attr3  = child3.append_attribute("id");

    std::vector<std::pair<id_t, std::string>> ids;
    bool                                      exception_found = false;

    // check overflow
    attr1.set_value("4294967294");
    attr2.set_value("4294967295");
    attr3.set_value("5");
    try
    {
        roadmanager::Position::GetOpenDrive()->EstablishUniqueIds(parent, "road", ids);
    }
    catch (const std::exception&)
    {
        exception_found = true;
    }
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(ids[0].first, 4294967294);
    EXPECT_EQ(exception_found, true);

    // Another round with unique and valid numbers
    ids.clear();
    exception_found = false;
    attr1.set_value("1000");
    attr2.set_value("4294967294");
    attr3.set_value("22");
    try
    {
        roadmanager::Position::GetOpenDrive()->EstablishUniqueIds(parent, "road", ids);
    }
    catch (const std::exception&)
    {
        exception_found = true;
    }
    EXPECT_EQ(ids.size(), 3);
    EXPECT_EQ(exception_found, false);
    EXPECT_EQ(ids[0].first, 1000);
    EXPECT_EQ(ids[1].first, 4294967294);
    EXPECT_EQ(ids[2].first, 22);

    // Check latest id wins
    ids.clear();
    exception_found = false;
    attr1.set_value("4294967291");
    attr2.set_value("4294967291");
    attr3.set_value("44");
    try
    {
        roadmanager::Position::GetOpenDrive()->EstablishUniqueIds(parent, "road", ids);
    }
    catch (const std::exception&)
    {
        exception_found = true;
    }
    EXPECT_EQ(ids.size(), 3);
    EXPECT_EQ(exception_found, false);
    EXPECT_EQ(ids[0].first, 4294967292);
    EXPECT_EQ(ids[1].first, 4294967291);
    EXPECT_EQ(ids[2].first, 44);

    // Another round with unique but invalid numbers
    ids.clear();
    exception_found = false;
    attr1.set_value("5");
    attr2.set_value("12j12");
    attr3.set_value("4");
    try
    {
        roadmanager::Position::GetOpenDrive()->EstablishUniqueIds(parent, "road", ids);
    }
    catch (const std::exception&)
    {
        exception_found = true;
    }
    EXPECT_EQ(ids.size(), 3);
    EXPECT_EQ(exception_found, false);
    EXPECT_EQ(ids[0].first, 5);
    EXPECT_EQ(ids[1].first, 6);
    EXPECT_EQ(ids[2].first, 4);

    // Check handling of reserved ID
    ids.clear();
    exception_found = false;
    attr1.set_value("5");
    attr2.set_value("4294967295");
    attr3.set_value("3");
    try
    {
        roadmanager::Position::GetOpenDrive()->EstablishUniqueIds(parent, "road", ids);
    }
    catch (const std::exception&)
    {
        exception_found = true;
    }
    EXPECT_EQ(ids.size(), 3);
    EXPECT_EQ(exception_found, false);
    EXPECT_EQ(ids[0].first, 5);
    EXPECT_EQ(ids[1].first, 6);
    EXPECT_EQ(ids[2].first, 3);
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

#if 0  // set to 1 and modify filter to run one single test
    testing::GTEST_FLAG(filter) = "*ALKS_R157_TestR157RegulationMinDist*";
    // Or make use of launch argument, e.g. --gtest_filter=*ALKS_R157_TestR157RegulationMinDist*
#endif
    ParseAndSetLoggerOptions(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
