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

    EXPECT_EQ(player->scenarioEngine->entities_.object_[0]->pos_.GetAlignModeZ(), roadmanager::Position::ALIGN_MODE::ALIGN_SOFT);
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
    player->scenarioGateway->setObjectAlignModeZ(0, roadmanager::Position::ALIGN_MODE::ALIGN_NONE);
    player->scenarioGateway->updateObjectWorldPos(0, 0.0, 221.381, -22.974, 5.0, 5.575, 0.0, 0.0);
    player->scenarioGateway->updateObjectSpeed(0, 0.0, 15.0);
    player->Frame(0.1);
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 5.0, 1E-3);

    // Keep same relative height from road
    player->scenarioGateway->setObjectAlignModeZ(0, roadmanager::Position::ALIGN_MODE::ALIGN_SOFT);
    player->scenarioGateway->updateObjectWorldPos(0, 0.0, 221.381, -22.974, 3.0, 5.575, 0.0, 0.0);
    player->scenarioGateway->updateObjectSpeed(0, 0.0, 15.0);
    for (int i = 0; i < 2; i++)  // step twice to move
    {
        player->Frame(0.1);
    }
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 3.184, 1E-3);

    // Align to road surface
    player->scenarioGateway->setObjectAlignModeZ(0, roadmanager::Position::ALIGN_MODE::ALIGN_HARD);
    player->scenarioGateway->updateObjectWorldPos(0, 0.0, 221.381, -22.974, 3.0, 5.575, 0.0, 0.0);
    player->scenarioGateway->updateObjectSpeed(0, 0.0, 15.0);
    for (int i = 0; i < 2; i++)  // step twice to move
    {
        player->Frame(0.1);
    }
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 2.006, 1E-3);

    delete player;
}

int main(int argc, char** argv)
{
    // testing::GTEST_FLAG(filter) = "*TestCustomCameraVariants*";

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
