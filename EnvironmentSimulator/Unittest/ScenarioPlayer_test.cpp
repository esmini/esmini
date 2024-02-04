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
    player->scenarioEngine->entities_.object_[0]->pos_.SetMode(roadmanager::Position::PosModeType::UPDATE, roadmanager::Position::PosMode::Z_ABS);
    EXPECT_EQ(player->scenarioEngine->entities_.object_[0]->pos_.GetMode(roadmanager::Position::PosModeType::UPDATE) &
                  roadmanager::Position::PosMode::Z_MASK,
              roadmanager::Position::PosMode::Z_ABS);
    player->Frame(0.0);
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

    EXPECT_EQ(pos.GetMode(Position::PosModeType::SET), Position::GetModeDefault(Position::PosModeType::SET));
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
    EXPECT_NEAR(pos.GetH(), 1.5, 1e-3);
    EXPECT_NEAR(pos.GetP(), 0.0, 1e-3);
    EXPECT_NEAR(pos.GetR(), 0.1, 1e-3);
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

    // testing::GTEST_FLAG(filter) = "*TestCustomCameraVariants*";

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
