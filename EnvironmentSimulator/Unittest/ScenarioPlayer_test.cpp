#include <iostream>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <vector>
#include <stdexcept>

#include "playerbase.hpp"

using namespace roadmanager;
using namespace scenarioengine;

#ifdef _USE_OSG

TEST(CustomCameraTest, TestCustomCameraVariants)
{
    const char* args[] =
        {"esmini", "--osc", "../../../resources/xosc/cut-in_cr.xosc", "--window", "60", "60", "800", "600", "--headless", "--disable_stdout"};
    int             argc   = sizeof(args) / sizeof(char*);
    ScenarioPlayer* player = new ScenarioPlayer(argc, const_cast<char**>(args));

    ASSERT_NE(player, nullptr);

    int retval = player->Init();
    ASSERT_EQ(retval, 0);

    player->AddCustomCamera(-4.0, 1.0, 1.5, 0.0, 0.0, false);
    player->AddCustomCamera(100.0, 50.0, 10.0, 0.5, 0.1, true);
    player->AddCustomCamera(100.0, 50.0, 10.0, true);
    player->AddCustomFixedTopCamera(100.0, 5.0, 10000.0, 0.5);

    osg::Vec3 pos, rot;
    player->viewer_->GetCameraPosAndRot(pos, rot);
    EXPECT_NEAR(pos[0], 125.153, 1E-3);
    EXPECT_NEAR(pos[1], 12.595, 1E-3);
    EXPECT_NEAR(pos[2], 2.937, 1E-3);
    EXPECT_NEAR(rot[0], 0.280, 1E-3);
    EXPECT_NEAR(rot[1], 0.227, 1E-3);
    EXPECT_NEAR(rot[2], 0.000, 1E-3);

    player->viewer_->SetCameraMode(osgGA::RubberbandManipulator::CAMERA_MODE::RB_MODE_CUSTOM + 1);
    player->Frame(0.0);
    player->viewer_->GetCameraPosAndRot(pos, rot);
    EXPECT_NEAR(pos[0], 100.000, 1E-3);
    EXPECT_NEAR(pos[1], 50.000, 1E-3);
    EXPECT_NEAR(pos[2], 10.000, 1E-3);
    EXPECT_NEAR(rot[0], 0.500, 1E-3);
    EXPECT_NEAR(rot[1], 0.100, 1E-3);
    EXPECT_NEAR(rot[2], 0.000, 1E-3);

    player->viewer_->SetCameraMode(osgGA::RubberbandManipulator::CAMERA_MODE::RB_MODE_CUSTOM + 2);
    player->Frame(0.0);
    player->viewer_->GetCameraPosAndRot(pos, rot);
    EXPECT_NEAR(pos[0], 100.000, 1E-3);
    EXPECT_NEAR(pos[1], 50.000, 1E-3);
    EXPECT_NEAR(pos[2], 10.000, 1E-3);
    EXPECT_NEAR(rot[0], 5.720, 1E-3);
    EXPECT_NEAR(rot[1], 0.217, 1E-3);
    EXPECT_NEAR(rot[2], 0.000, 1E-3);

    player->viewer_->SetCameraMode(osgGA::RubberbandManipulator::CAMERA_MODE::RB_MODE_CUSTOM + 3);
    player->Frame(0.0);
    player->viewer_->GetCameraPosAndRot(pos, rot);
    EXPECT_NEAR(pos[0], 100.000, 1E-3);
    EXPECT_NEAR(pos[1], 5.00, 1E-3);
    EXPECT_NEAR(pos[2], 10000.00, 1E-3);
    EXPECT_NEAR(rot[0], 0.500, 1E-3);
    EXPECT_NEAR(rot[1], 1.571, 1E-3);
    EXPECT_NEAR(rot[2], 0.000, 1E-3);

    for (int i = 0; i < 100; i++)
    {
        player->Frame(0.1);
    }
    player->viewer_->SetCameraMode(osgGA::RubberbandManipulator::CAMERA_MODE::RB_MODE_CUSTOM + 0);
    player->Frame(0.0);
    player->viewer_->GetCameraPosAndRot(pos, rot);
    EXPECT_NEAR(pos[0], 217.634, 1E-3);
    EXPECT_NEAR(pos[1], 128.856, 1E-3);
    EXPECT_NEAR(pos[2], 1.601, 1E-3);
    EXPECT_NEAR(rot[0], 1.487, 1E-3);
    EXPECT_NEAR(rot[1], 6.228, 1E-3);
    EXPECT_NEAR(rot[2], 0.000, 1E-3);

    player->viewer_->SetCameraMode(osgGA::RubberbandManipulator::CAMERA_MODE::RB_MODE_CUSTOM + 1);
    player->Frame(0.0);
    player->viewer_->GetCameraPosAndRot(pos, rot);
    EXPECT_NEAR(pos[0], 100.000, 1E-3);
    EXPECT_NEAR(pos[1], 50.000, 1E-3);
    EXPECT_NEAR(pos[2], 10.000, 1E-3);
    EXPECT_NEAR(rot[0], 0.500, 1E-3);
    EXPECT_NEAR(rot[1], 0.100, 1E-3);
    EXPECT_NEAR(rot[2], 0.000, 1E-3);

    player->viewer_->SetCameraMode(osgGA::RubberbandManipulator::CAMERA_MODE::RB_MODE_CUSTOM + 2);
    player->Frame(0.0);
    player->viewer_->GetCameraPosAndRot(pos, rot);
    EXPECT_NEAR(pos[0], 100.000, 1E-3);
    EXPECT_NEAR(pos[1], 50.000, 1E-3);
    EXPECT_NEAR(pos[2], 10.000, 1E-3);
    EXPECT_NEAR(rot[0], 0.609, 1E-3);
    EXPECT_NEAR(rot[1], 0.065, 1E-3);
    EXPECT_NEAR(rot[2], 0.000, 1E-3);

    player->viewer_->SetCameraMode(osgGA::RubberbandManipulator::CAMERA_MODE::RB_MODE_CUSTOM + 3);
    player->Frame(0.0);
    player->viewer_->GetCameraPosAndRot(pos, rot);
    EXPECT_NEAR(pos[0], 100.000, 1E-3);
    EXPECT_NEAR(pos[1], 5.000, 1E-3);
    EXPECT_NEAR(pos[2], 10000.000, 1E-3);
    EXPECT_NEAR(rot[0], 0.500, 1E-3);
    EXPECT_NEAR(rot[1], 1.571, 1E-3);
    EXPECT_NEAR(rot[2], 0.000, 1E-3);

    delete player;
}

#endif  // _USE_OSG

TEST(AlignmentTest, TestPositionAlignmentVariants)
{
    const char*     args[] = {"esmini", "--osc", "../../../resources/xosc/lane_change_crest.xosc", "--headless", "--disable_stdout"};
    int             argc   = sizeof(args) / sizeof(char*);
    ScenarioPlayer* player = new ScenarioPlayer(argc, const_cast<char**>(args));

    ASSERT_NE(player, nullptr);
    int retval = player->Init();
    ASSERT_EQ(retval, 0);

    EXPECT_EQ(player->scenarioEngine->entities_.object_[0]->pos_.GetMode(roadmanager::Position::PosModeType::UPDATE) &
                  roadmanager::Position::PosMode::Z_MASK,
              roadmanager::Position::PosMode::Z_REL);
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 0.0, 1E-3);

    player->scenarioGateway->updateObjectWorldPos(0, 0.0, 164.65, -4.63, 10.0, 6.14, 0.0, 0.0);
    player->Frame(0.0);
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 10.0, 1E-3);
    player->scenarioGateway->updateObjectSpeed(0, 0.0, 15.0);
    while (player->scenarioEngine->getSimulationTime() < 4.0 - SMALL_NUMBER)
    {
        player->Frame(0.1);
    }
    // no change expected since align mode activated
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 11.822, 1E-3);

    // Ignore road, no alignment
    EXPECT_EQ(player->scenarioEngine->entities_.object_[0]->pos_.GetMode(roadmanager::Position::PosModeType::UPDATE) &
                  roadmanager::Position::PosMode::Z_MASK,
              roadmanager::Position::PosMode::Z_REL);
    // use default z mode = relative
    player->scenarioGateway->updateObjectWorldPos(0, 0.0, 221.381, -22.974, 1.0, 5.575, 0.0, 0.0);
    player->scenarioGateway->updateObjectSpeed(0, 0.0, 15.0);
    player->Frame(0.1);
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetX(), 221.381, 1E-3);
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetY(), -22.974, 1E-3);
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 2.822, 1E-3);
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ() - player->scenarioEngine->entities_.object_[0]->pos_.GetZRoad(), 1.0, 1E-3);
    player->Frame(1.0);
    // z should not have changed, same offset to road
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetX(), 233.808, 1E-3);
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetY(), -31.333, 1E-3);
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 4.741, 1E-3);
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ() - player->scenarioEngine->entities_.object_[0]->pos_.GetZRoad(), 1.0, 1E-3);

    // Update with z mode = absolute, ignore road
    player->scenarioGateway->updateObjectWorldPosMode(0, 0.0, 221.381, -22.974, 5.0, 5.575, 0.0, 0.0, Position::PosMode::Z_ABS);
    player->scenarioGateway->updateObjectSpeed(0, 0.0, 15.0);
    player->Frame(0.1);
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 5.0, 1E-3);
    player->Frame(1.0);
    // z should now have changed, due to underlying PositionMode = relative
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 6.919, 1E-3);

    // Ensure absolute update mode preserves z level
    player->scenarioGateway->setObjectPositionMode(0,
                                                   static_cast<int>(roadmanager::Position::PosModeType::UPDATE),
                                                   static_cast<int>(roadmanager::Position::PosMode::Z_ABS));
    player->Frame(0.0);
    EXPECT_EQ(player->scenarioEngine->entities_.object_[0]->pos_.GetMode(roadmanager::Position::PosModeType::UPDATE) &
                  roadmanager::Position::PosMode::Z_MASK,
              roadmanager::Position::PosMode::Z_ABS);
    player->scenarioGateway->updateObjectWorldPosMode(0, 0.0, 221.381, -22.974, 3.0, 5.575, 0.0, 0.0, roadmanager::Position::PosMode::Z_ABS);
    player->scenarioGateway->updateObjectSpeed(0, 0.0, 15.0);
    player->Frame(0.1);
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 3.0, 1E-3);
    player->Frame(0.1);
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 3.0, 1E-3);

    // Align to road surface
    player->scenarioGateway->updateObjectWorldPosMode(0, 0.0, 221.381, -22.974, 0.0, 0.0, 0.0, 0.0, roadmanager::Position::PosMode::Z_REL);
    player->scenarioGateway->setObjectPositionMode(0,

                                                   static_cast<int>(roadmanager::Position::PosModeType::UPDATE),
                                                   roadmanager::Position::PosMode::Z_REL);
    player->scenarioGateway->updateObjectSpeed(0, 0.0, 15.0);

    for (int i = 0; i < 2; i++)  // step twice to move
    {
        player->Frame(0.1);
    }
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 2.006, 1E-3);

    delete player;
}

TEST(SensorTest, TestSensorFunctionsReturnValuesAndCounters)
{
    const char*     args[] = {"esmini", "--osc", "../../../resources/xosc/cut-in.xosc", "--headless", "--disable_stdout"};
    int             argc   = sizeof(args) / sizeof(char*);
    ScenarioPlayer* player = new ScenarioPlayer(argc, const_cast<char**>(args));

    ASSERT_NE(player, nullptr);
    int retval = player->Init();
    ASSERT_EQ(retval, 0);

    ASSERT_EQ(player->scenarioEngine->entities_.object_.size(), 2);

    EXPECT_EQ(player->AddObjectSensor(player->scenarioEngine->entities_.object_[0], 2.0, 0.0, 1.0, 0.0, 1.0, 80.0, 0.7, 10), 0);
    EXPECT_EQ(player->AddObjectSensor(player->scenarioEngine->entities_.object_[0], 1.0, 1.0, 1.0, 0.0, 1.0, 40.0, 0.7, 10), 1);
    EXPECT_EQ(player->AddObjectSensor(player->scenarioEngine->entities_.object_[1], 1.0, 1.0, 1.0, 0.0, 1.0, 40.0, 0.7, 10), 2);
    EXPECT_EQ(player->GetNumberOfSensorsAttachedToObject(player->scenarioEngine->entities_.object_[0]), 2);
    EXPECT_EQ(player->GetNumberOfSensorsAttachedToObject(player->scenarioEngine->entities_.object_[1]), 1);
    EXPECT_EQ(player->GetNumberOfObjectSensors(), 3);

    delete player;
}

TEST(AlignmentTest, TestPosMode)
{
    const char* args[] = {"esmini", "--headless", "--osc", "../../../EnvironmentSimulator/Unittest/xosc/curve_slope_simple.xosc", "--disable_stdout"};
    int         argc   = sizeof(args) / sizeof(char*);
    ScenarioPlayer* player = new ScenarioPlayer(argc, const_cast<char**>(args));

    ASSERT_NE(player, nullptr);
    int retval = player->Init();
    ASSERT_EQ(retval, 0);

    roadmanager::Position& pos  = player->scenarioEngine->entities_.object_[0]->pos_;
    roadmanager::Road*     road = player->GetODRManager()->GetRoadByIdx(0);

    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::UPDATE), Position::GetModeDefault(Position::PosModeType::UPDATE));

    // Test some operations
    pos.SetLanePos(road->GetId(), -1, 140.0, 0.0);
    player->scenarioEngine->entities_.object_[0]->SetSpeed(0.0);
    EXPECT_NEAR(pos.GetH(), 1.4, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.0, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.486, 1e-3);
    player->Frame(0.1);

    pos.SetMode(Position::PosModeType::UPDATE, Position::PosMode::R_REL);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::UPDATE),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);
    pos.SetRollRelative(0.1);
    pos.SetLanePos(road->GetId(), -1, 150.0, 0.0);
    player->scenarioEngine->entities_.object_[0]->SetSpeed(0.0);

    EXPECT_NEAR(pos.GetH(), 1.5, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.0, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.6, 1e-3);
    EXPECT_NEAR(pos.GetRRoad(), 0.6 - 0.1, 1e-3);
    player->Frame(0.1);

    pos.SetLanePos(road->GetId(), -1, 140.0, 0.0);
    player->scenarioEngine->entities_.object_[0]->SetSpeed(0.0);
    EXPECT_NEAR(pos.GetH(), 1.4, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.0, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.486 + 0.1, 1e-3);
    player->Frame(0.1);

    pos.SetMode(Position::PosModeType::UPDATE, Position::PosMode::R_ABS);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::UPDATE),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_ABS);
    pos.SetRoll(0.1);
    pos.SetLanePos(road->GetId(), -1, 150.0, 0.0);
    player->scenarioEngine->entities_.object_[0]->SetSpeed(0.0);
    EXPECT_NEAR(GetAngleDifference(pos.GetH(), 1.5), 0.0, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetP(), 0.0), 0.0, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetR(), 0.1), 0.0, 1e-3);
    player->Frame(0.1);

    pos.SetLanePos(road->GetId(), -1, 140.0, 0.0);
    player->scenarioEngine->entities_.object_[0]->SetSpeed(0.0);
    EXPECT_NEAR(GetAngleDifference(pos.GetH(), 1.4), 0.0, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetP(), 0.0), 0.0, 1e-3);
    EXPECT_NEAR(GetAngleDifference(pos.GetR(), 0.1), 0.0, 1e-3);
    player->Frame(0.1);

    pos.SetLanePos(road->GetId(), -1, 300.0, 0.0);
    player->scenarioEngine->entities_.object_[0]->SetSpeed(0.0);
    EXPECT_NEAR(pos.GetH(), 3.0, 1e-3);
    EXPECT_NEAR(pos.GetP(), 5.991, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.1, 1e-3);
    player->Frame(0.1);

    pos.SetMode(Position::PosModeType::UPDATE, Position::PosMode::R_REL);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::UPDATE),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);
    pos.SetRollRelative(0.0);
    pos.SetLanePos(road->GetId(), -1, 300.0, 0.0);
    player->scenarioEngine->entities_.object_[0]->SetSpeed(0.0);
    EXPECT_NEAR(pos.GetH(), 3.0, 1e-3);
    EXPECT_NEAR(pos.GetP(), 5.991, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.0, 1e-3);

    pos.SetInertiaPos(0.0, 200.0, 0.5);
    player->scenarioEngine->entities_.object_[0]->SetSpeed(0.0);
    EXPECT_NEAR(pos.GetH(), 0.5156, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.2356, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.1315, 1e-3);

    pos.SetInertiaPos(-100.0, 83.0, 0.5);
    player->scenarioEngine->entities_.object_[0]->SetSpeed(0.0);
    EXPECT_NEAR(pos.GetH(), 0.5, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.0, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.0, 1e-3);

    pos.SetModeDefault(Position::PosModeType::SET);
    pos.SetInertiaPos(100.0, 85.0, -10.0, 0.5, 0.0, 0.3);
    player->scenarioEngine->entities_.object_[0]->SetSpeed(0.0);
    EXPECT_NEAR(pos.GetH(), 0.5615, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.3853, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.6127, 1e-3);

    pos.SetMode(Position::PosModeType::SET, Position::PosMode::R_ABS);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_ABS);
    pos.SetInertiaPos(100.0, 85.0, -10.0, 0.5, 0.0, 0.3);
    player->scenarioEngine->entities_.object_[0]->SetSpeed(0.0);
    EXPECT_NEAR(pos.GetH(), 0.5, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.0, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.3, 1e-3);

    // Test some settings
    pos.SetMode(Position::PosModeType::UPDATE, Position::PosMode::H_REL | Position::PosMode::Z_ABS);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::UPDATE),
              roadmanager::Position::PosMode::Z_ABS | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);

    pos.SetMode(Position::PosModeType::SET, Position::PosMode::H_REL | Position::PosMode::Z_ABS | Position::PosMode::P_ABS);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_ABS | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_ABS |
                  roadmanager::Position::PosMode::R_ABS);
    pos.SetMode(Position::PosModeType::SET, Position::PosMode::Z_MASK & Position::PosMode::Z_DEFAULT);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET),
              roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_ABS |
                  roadmanager::Position::PosMode::R_ABS);
    EXPECT_EQ(pos.GetMode(Position::PosModeType::UPDATE),
              roadmanager::Position::PosMode::Z_ABS | roadmanager::Position::PosMode::H_REL | roadmanager::Position::PosMode::P_REL |
                  roadmanager::Position::PosMode::R_REL);

    delete player;
}

TEST(Controllers, TestSeparateControllersOnLatLong)
{
    const char*     args[] = {"esmini",
                              "--osc",
                              "../../../EnvironmentSimulator/Unittest/xosc/acc_with_interactive_steering.xosc",
                              "--window",
                              "60",
                              "60",
                              "800",
                              "600",
                              "--headless",
                              "--disable_stdout"};
    int             argc   = sizeof(args) / sizeof(char*);
    double          dt     = 0.1f;
    ScenarioPlayer* player = new ScenarioPlayer(argc, const_cast<char**>(args));

    ASSERT_NE(player, nullptr);
    int retval = player->Init();
    ASSERT_EQ(retval, 0);

    ScenarioEngine* se = player->scenarioEngine;
    ASSERT_EQ(se->entities_.object_.size(), 2);
    EXPECT_EQ(se->entities_.object_[0]->controllers_.size(), 2);

    scenarioengine::Controller* ctrl = se->entities_.object_[0]->controllers_[0];

    // Check expected position and orientation at some specific time stamps
    while (!player->IsQuitRequested())
    {
        // steer left
        if (fabs(se->getSimulationTime() - 5.0) < 0.1 * dt)
        {
            ctrl->ReportKeyEvent(static_cast<int>(KeyType::KEY_Left), true);
        }
        if (fabs(se->getSimulationTime() - 5.4) < 0.1 * dt)
        {
            ctrl->ReportKeyEvent(static_cast<int>(KeyType::KEY_Left), false);
        }

        // steer right
        if (fabs(se->getSimulationTime() - 6.0) < 0.1 * dt)
        {
            ctrl->ReportKeyEvent(static_cast<int>(KeyType::KEY_Right), true);
        }
        if (fabs(se->getSimulationTime() - 6.6) < 0.1 * dt)
        {
            ctrl->ReportKeyEvent(static_cast<int>(KeyType::KEY_Right), false);
            EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 106.756, 1e-3);
            EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), 1.339, 1e-3);
        }

        // steer left
        if (fabs(se->getSimulationTime() - 8.0) < 0.1 * dt)
        {
            ctrl->ReportKeyEvent(static_cast<int>(KeyType::KEY_Left), true);
        }
        if (fabs(se->getSimulationTime() - 8.2) < 0.1 * dt)
        {
            ctrl->ReportKeyEvent(static_cast<int>(KeyType::KEY_Left), false);
            EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 125.565, 1e-3);
            EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.060, 1e-3);
        }

        if (fabs(se->getSimulationTime() - 31.3) < 0.1 * dt)
        {
            EXPECT_NEAR(se->entities_.object_[0]->pos_.GetX(), 353.365, 1e-3);
            EXPECT_NEAR(se->entities_.object_[0]->pos_.GetY(), -1.539, 1e-3);
        }

        player->Frame(dt);
    }

    delete player;
}

#ifdef _USE_OSI

TEST(OSI, TestOrientation)
{
    const char* args[] =
        {"esmini", "--osc", "../../../EnvironmentSimulator/Unittest/xosc/curve_slope_simple.xosc", "--headless", "--osi_file", "--disable_stdout"};
    int             argc   = sizeof(args) / sizeof(char*);
    double          dt     = 0.1f;
    ScenarioPlayer* player = new ScenarioPlayer(argc, const_cast<char**>(args));

    ASSERT_NE(player, nullptr);
    int retval = player->Init();
    ASSERT_EQ(retval, 0);

    ScenarioEngine* se = player->scenarioEngine;
    ASSERT_EQ(se->entities_.object_.size(), 1);
    Object* obj = se->entities_.object_[0];

    const osi3::GroundTruth* osi_gt_ptr = reinterpret_cast<const osi3::GroundTruth*>(player->osiReporter->GetOSIGroundTruthRaw());
    ASSERT_NE(osi_gt_ptr, nullptr);

    // OpenSCENARIO position
    EXPECT_NEAR(obj->pos_.GetX(), 100.0576, 1e-3);
    EXPECT_NEAR(obj->pos_.GetY(), 82.7423, 1e-3);
    EXPECT_NEAR(obj->pos_.GetZ(), -0.8108, 1e-3);
    EXPECT_NEAR(obj->pos_.GetH(), 1.4000, 1e-3);
    EXPECT_NEAR(obj->pos_.GetP(), 0.0, 1e-3);
    EXPECT_NEAR(obj->pos_.GetR(), 0.48599, 1e-3);

    // OSI position
    EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().position().x(), 100.6408, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().position().y(), 84.0624, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().position().z(), -0.1477, 1e-3);
    EXPECT_NEAR(GetAngleDifference(osi_gt_ptr->moving_object(0).base().orientation().yaw(), 1.4000), 0.0, 1e-3);
    EXPECT_NEAR(GetAngleDifference(osi_gt_ptr->moving_object(0).base().orientation().pitch(), 0.0), 0.0, 1e-3);
    EXPECT_NEAR(GetAngleDifference(osi_gt_ptr->moving_object(0).base().orientation().roll(), 0.48599), 0.0, 1e-3);

    // move forward to the uphill part
    obj->MoveAlongS(170.0);
    player->Frame(dt);

    // OpenSCENARIO position
    EXPECT_NEAR(obj->pos_.GetX(), 6.8274, 1e-3);
    EXPECT_NEAR(obj->pos_.GetY(), 201.3051, 1e-3);
    EXPECT_NEAR(obj->pos_.GetZ(), 12.2125, 1e-3);
    EXPECT_NEAR(obj->pos_.GetH(), 3.0742, 1e-3);
    EXPECT_NEAR(obj->pos_.GetP(), 5.9978, 1e-3);
    EXPECT_NEAR(obj->pos_.GetR(), 0.0, 1e-3);

    // OSI position
    EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().position().x(), 5.6977, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().position().y(), 201.3813, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().position().z(), 13.3263, 1e-3);
    EXPECT_NEAR(GetAngleDifference(osi_gt_ptr->moving_object(0).base().orientation().yaw(), 3.0742), 0.0, 1e-3);
    EXPECT_NEAR(GetAngleDifference(osi_gt_ptr->moving_object(0).base().orientation().pitch(), 5.9978), 0.0, 1e-3);
    EXPECT_NEAR(GetAngleDifference(osi_gt_ptr->moving_object(0).base().orientation().roll(), 0.0), 0.0, 1e-3);

    delete player;
}

TEST(OSI, TestDirectJunctions)
{
    const char* args[] =
        {"esmini", "--osc", "../../../EnvironmentSimulator/Unittest/xosc/direct_junction.xosc", "--headless", "--osi_file", "--disable_stdout"};
    int             argc   = sizeof(args) / sizeof(char*);
    ScenarioPlayer* player = new ScenarioPlayer(argc, const_cast<char**>(args));

    ASSERT_NE(player, nullptr);
    int retval = player->Init();
    ASSERT_EQ(retval, 0);

    ScenarioEngine* se = player->scenarioEngine;
    ASSERT_EQ(se->entities_.object_.size(), 1);

    const osi3::GroundTruth* osi_gt_ptr = reinterpret_cast<const osi3::GroundTruth*>(player->osiReporter->GetOSIGroundTruthRaw());
    ASSERT_NE(osi_gt_ptr, nullptr);

    // direct junction connections - both ways
    EXPECT_EQ(osi_gt_ptr->lane(1).id().value(), 2);
    EXPECT_EQ(osi_gt_ptr->lane(1).classification().lane_pairing_size(), 1);
    EXPECT_EQ(osi_gt_ptr->lane(1).classification().lane_pairing(0).has_antecessor_lane_id(), false);
    EXPECT_EQ(osi_gt_ptr->lane(1).classification().lane_pairing(0).has_successor_lane_id(), true);
    EXPECT_EQ(osi_gt_ptr->lane(1).classification().lane_pairing(0).successor_lane_id().value(), 8);

    EXPECT_EQ(osi_gt_ptr->lane(2).id().value(), 3);
    EXPECT_EQ(osi_gt_ptr->lane(2).classification().lane_pairing_size(), 1);
    EXPECT_EQ(osi_gt_ptr->lane(2).classification().lane_pairing(0).has_antecessor_lane_id(), false);
    EXPECT_EQ(osi_gt_ptr->lane(2).classification().lane_pairing(0).has_successor_lane_id(), true);
    EXPECT_EQ(osi_gt_ptr->lane(2).classification().lane_pairing(0).successor_lane_id().value(), 9);

    EXPECT_EQ(osi_gt_ptr->lane(3).id().value(), 4);
    EXPECT_EQ(osi_gt_ptr->lane(3).classification().lane_pairing_size(), 1);
    EXPECT_EQ(osi_gt_ptr->lane(3).classification().lane_pairing(0).has_antecessor_lane_id(), false);
    EXPECT_EQ(osi_gt_ptr->lane(3).classification().lane_pairing(0).has_successor_lane_id(), true);
    EXPECT_EQ(osi_gt_ptr->lane(3).classification().lane_pairing(0).successor_lane_id().value(), 11);

    EXPECT_EQ(osi_gt_ptr->lane(6).id().value(), 8);
    EXPECT_EQ(osi_gt_ptr->lane(6).classification().lane_pairing_size(), 1);
    EXPECT_EQ(osi_gt_ptr->lane(6).classification().lane_pairing(0).has_antecessor_lane_id(), true);
    EXPECT_EQ(osi_gt_ptr->lane(6).classification().lane_pairing(0).antecessor_lane_id().value(), 2);
    EXPECT_EQ(osi_gt_ptr->lane(6).classification().lane_pairing(0).has_successor_lane_id(), true);
    EXPECT_EQ(osi_gt_ptr->lane(6).classification().lane_pairing(0).successor_lane_id().value(), 15);

    EXPECT_EQ(osi_gt_ptr->lane(7).id().value(), 9);
    EXPECT_EQ(osi_gt_ptr->lane(7).classification().lane_pairing_size(), 1);
    EXPECT_EQ(osi_gt_ptr->lane(7).classification().lane_pairing(0).has_antecessor_lane_id(), true);
    EXPECT_EQ(osi_gt_ptr->lane(7).classification().lane_pairing(0).antecessor_lane_id().value(), 3);
    EXPECT_EQ(osi_gt_ptr->lane(7).classification().lane_pairing(0).has_successor_lane_id(), true);
    EXPECT_EQ(osi_gt_ptr->lane(7).classification().lane_pairing(0).successor_lane_id().value(), 16);

    EXPECT_EQ(osi_gt_ptr->lane(8).id().value(), 11);
    EXPECT_EQ(osi_gt_ptr->lane(8).classification().lane_pairing_size(), 1);
    EXPECT_EQ(osi_gt_ptr->lane(8).classification().lane_pairing(0).has_antecessor_lane_id(), true);
    EXPECT_EQ(osi_gt_ptr->lane(8).classification().lane_pairing(0).antecessor_lane_id().value(), 4);
    EXPECT_EQ(osi_gt_ptr->lane(8).classification().lane_pairing(0).has_successor_lane_id(), false);

    delete player;
}

TEST(OSI, TestGeoOffset)
{
    const char* args[] =
        {"esmini", "--osc", "../../../EnvironmentSimulator/Unittest/xosc/geo_pos.xosc", "--headless", "--osi_file", "--disable_stdout"};
    int             argc   = sizeof(args) / sizeof(char*);
    ScenarioPlayer* player = new ScenarioPlayer(argc, const_cast<char**>(args));

    ASSERT_NE(player, nullptr);
    int retval = player->Init();
    ASSERT_EQ(retval, 0);

    ScenarioEngine* se = player->scenarioEngine;
    ASSERT_EQ(se->entities_.object_.size(), 2);

    const osi3::GroundTruth* osi_gt_ptr = reinterpret_cast<const osi3::GroundTruth*>(player->osiReporter->GetOSIGroundTruthRaw());
    ASSERT_NE(osi_gt_ptr, nullptr);

    // verify that proj_string includes the offset values, and that the missing z value is omitted
    EXPECT_STREQ(osi_gt_ptr->proj_string().c_str(), "offset x=-1000 y=-500 z=0.0 hdg=0.959931");

    // verify correct location of some lane OSI points after offset transformation
    EXPECT_EQ(osi_gt_ptr->lane(1).id().value(), 2);
    EXPECT_EQ(osi_gt_ptr->lane(0).classification().centerline().size(), 3);
    EXPECT_NEAR(osi_gt_ptr->lane(0).classification().centerline(0).x(), -139.5333, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->lane(0).classification().centerline(0).y(), -23.0803, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->lane(0).classification().centerline(1).x(), -90.2929, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->lane(0).classification().centerline(1).y(), -14.3979, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->lane(0).classification().centerline(2).x(), -41.0525, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->lane(0).classification().centerline(2).y(), -5.7155, 1e-3);

    // check world position specified for car_offset
    EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().position().x(), -128.3065, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().position().y(), -21.1007, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->moving_object(0).base().orientation().yaw(), 0.1745, 1e-3);

    delete player;
}

TEST(OSI, TestGeoOffsetIgnoreODROffset)
{
    const char*     args[] = {"esmini",
                              "--osc",
                              "../../../EnvironmentSimulator/Unittest/xosc/geo_pos.xosc",
                              "--headless",
                              "--osi_file",
                              "--disable_stdout",
                              "--ignore_odr_offset"};
    int             argc   = sizeof(args) / sizeof(char*);
    ScenarioPlayer* player = new ScenarioPlayer(argc, const_cast<char**>(args));

    ASSERT_NE(player, nullptr);
    int retval = player->Init();
    ASSERT_EQ(retval, 0);

    ScenarioEngine* se = player->scenarioEngine;
    ASSERT_EQ(se->entities_.object_.size(), 2);

    const osi3::GroundTruth* osi_gt_ptr = reinterpret_cast<const osi3::GroundTruth*>(player->osiReporter->GetOSIGroundTruthRaw());
    ASSERT_NE(osi_gt_ptr, nullptr);

    // verify that proj_string includes the offset values, and that the missing z value is omitted
    EXPECT_STREQ(osi_gt_ptr->proj_string().c_str(), "offset x=-1000 y=-500 z=0.0 hdg=0.959931");

    // verify correct location of some lane OSI points after offset transformation
    EXPECT_EQ(osi_gt_ptr->lane(1).id().value(), 2);
    EXPECT_EQ(osi_gt_ptr->lane(0).classification().centerline().size(), 3);
    EXPECT_NEAR(osi_gt_ptr->lane(0).classification().centerline(0).x(), -139.5333, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->lane(0).classification().centerline(0).y(), -23.0803, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->lane(0).classification().centerline(1).x(), -90.2929, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->lane(0).classification().centerline(1).y(), -14.3979, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->lane(0).classification().centerline(2).x(), -41.0525, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->lane(0).classification().centerline(2).y(), -5.7155, 1e-3);

    // check world position specified for car_ignore_offset
    EXPECT_NEAR(osi_gt_ptr->moving_object(1).base().position().x(), -128.3065, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->moving_object(1).base().position().y(), -21.1007, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->moving_object(1).base().orientation().yaw(), 0.1745, 1e-3);

    delete player;
}

TEST(OSI, TestStationaryObjects)
{
    const char* args[] =
        {"esmini", "--osc", "../../../EnvironmentSimulator/Unittest/xosc/stationary_objects.xosc", "--headless", "--osi_file", "--disable_stdout"};
    int             argc   = sizeof(args) / sizeof(char*);
    ScenarioPlayer* player = new ScenarioPlayer(argc, const_cast<char**>(args));

    ASSERT_NE(player, nullptr);
    int retval = player->Init();
    ASSERT_EQ(retval, 0);

    ScenarioEngine* se = player->scenarioEngine;
    ASSERT_EQ(se->entities_.object_.size(), 1);

    const osi3::GroundTruth* osi_gt_ptr = reinterpret_cast<const osi3::GroundTruth*>(player->osiReporter->GetOSIGroundTruthRaw());
    ASSERT_NE(osi_gt_ptr, nullptr);

    // verify correct location of OpenDRIVE stationary object with polygon
    EXPECT_EQ(osi_gt_ptr->stationary_object(0).id().value(), 0);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().dimension().length(), 0.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().dimension().width(), 0.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().dimension().height(), 2.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().position().x(), 20.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().position().y(), -7.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().position().z(), 8.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().orientation().yaw(), 0.2, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().orientation().pitch(), 0.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().orientation().roll(), 0.0, 1e-3);

    EXPECT_EQ(osi_gt_ptr->stationary_object(0).base().base_polygon_size(), 4);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().base_polygon().at(0).x(), -2.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().base_polygon().at(0).y(), 2.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().base_polygon().at(1).x(), -2.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().base_polygon().at(1).y(), -3.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().base_polygon().at(2).x(), 8.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().base_polygon().at(2).y(), 0.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().base_polygon().at(3).x(), 5.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(0).base().base_polygon().at(3).y(), 2.0, 1e-3);

    // verify correct location of OpenDRIVE second stationary object
    EXPECT_EQ(osi_gt_ptr->stationary_object(1).id().value(), 1);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(1).base().dimension().length(), 4.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(1).base().dimension().width(), 4.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(1).base().dimension().height(), 10.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(1).base().position().x(), 40.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(1).base().position().y(), -7.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(1).base().position().z(), 4.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(1).base().orientation().yaw(), 0.7800, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(1).base().orientation().pitch(), 0.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(1).base().orientation().roll(), 0.0, 1e-3);

    // verify correct location of OpenDRIVE sign pole object
    EXPECT_EQ(osi_gt_ptr->stationary_object(2).id().value(), 2);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(2).base().dimension().length(), 0.06, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(2).base().dimension().width(), 0.06, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(2).base().dimension().height(), 2.35, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(2).base().position().x(), 5.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(2).base().position().y(), 0.5, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(2).base().position().z(), 0.975, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(2).base().orientation().yaw(), 0.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(2).base().orientation().pitch(), 0.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(2).base().orientation().roll(), 0.0, 1e-3);

    // verify correct location of OSC box object
    EXPECT_EQ(osi_gt_ptr->stationary_object(3).id().value(), 3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(3).base().dimension().length(), 2.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(3).base().dimension().width(), 1.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(3).base().dimension().height(), 2.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(3).base().position().x(), 10.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(3).base().position().y(), -5.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(3).base().position().z(), 1.5, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(3).base().orientation().yaw(), 0.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(3).base().orientation().pitch(), 0.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->stationary_object(3).base().orientation().roll(), 0.0, 1e-3);

    // verify correct location of OpenDRIVE traffic sign
    EXPECT_EQ(osi_gt_ptr->traffic_sign(0).id().value(), 1);
    EXPECT_NEAR(osi_gt_ptr->traffic_sign(0).main_sign().base().dimension().length(), 0.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->traffic_sign(0).main_sign().base().dimension().width(), 0.6, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->traffic_sign(0).main_sign().base().dimension().height(), 0.6, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->traffic_sign(0).main_sign().base().position().x(), 5.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->traffic_sign(0).main_sign().base().position().y(), 0.5, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->traffic_sign(0).main_sign().base().position().z(), 2.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->traffic_sign(0).main_sign().base().orientation().yaw(), 3.1415, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->traffic_sign(0).main_sign().base().orientation().pitch(), 0.0, 1e-3);
    EXPECT_NEAR(osi_gt_ptr->traffic_sign(0).main_sign().base().orientation().roll(), 0.0, 1e-3);

    delete player;
}

#endif  // _USE_OSI

int main(int argc, char** argv)
{
    // testing::GTEST_FLAG(filter) = "*TestCustomCameraVariants*";
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
