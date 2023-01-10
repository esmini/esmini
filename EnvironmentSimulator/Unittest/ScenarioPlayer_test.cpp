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
    {
        "esmini", "--osc", "../../../resources/xosc/cut-in_cr.xosc", "--window", "60", "60", "800", "600", "--headless", "--disable_stdout"
    };
    int argc = sizeof(args) / sizeof(char*);
    ScenarioPlayer* player = new ScenarioPlayer(argc, const_cast<char**>(args));

    ASSERT_NE(player, nullptr);

    int retval = player->Init();
    ASSERT_EQ(retval, 0);

    player->AddCustomCamera(-4.0, 1.0, 1.5, 0.0, 0.0);
    player->AddCustomFixedCamera(100.0, 50.0, 10.0, 0.5, 0.1);
    player->AddCustomSemiFixedCamera(100.0, 50.0, 10.0);
    player->AddCustomFixedTopCamera(100.0, 5.0, 10000.0, 0.5);

    osg::Vec3 pos, rot;
    player->viewer_->GetCameraPosAndRot(pos, rot);
    EXPECT_NEAR(pos[0], 123.952, 1E-3);
    EXPECT_NEAR(pos[1], 11.902, 1E-3);
    EXPECT_NEAR(pos[2], 3.020, 1E-3);
    EXPECT_NEAR(rot[0], 0.280, 1E-3);
    EXPECT_NEAR(rot[1], 0.230, 1E-3);
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
    EXPECT_NEAR(rot[0], 5.698, 1E-3);
    EXPECT_NEAR(rot[1], 0.220, 1E-3);
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
    EXPECT_NEAR(pos[0], 217.520, 1E-3);
    EXPECT_NEAR(pos[1], 127.505, 1E-3);
    EXPECT_NEAR(pos[2], 0.775, 1E-3);
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
    EXPECT_NEAR(rot[0], 0.601, 1E-3);
    EXPECT_NEAR(rot[1], 0.066, 1E-3);
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

#endif // _USE_OSG

TEST(AlignmentTest, TestPositionAlignmentVariants)
{
    const char* args[] =
    {
        "esmini", "--osc", "../../../resources/xosc/cut-in.xosc", "--headless", "--disable_stdout"
    };
    int argc = sizeof(args) / sizeof(char*);
    ScenarioPlayer* player = new ScenarioPlayer(argc, (char**)args);

    ASSERT_NE(player, nullptr);
    int retval = player->Init();
    ASSERT_EQ(retval, 0);

    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), -0.0406, 1E-3);

    player->scenarioGateway->updateObjectWorldPos(0, 0.0, 8.0, 67.0, 10.0, 1.57, 0.0, 0.0);
    player->Frame(0.05);
    // no change expected since align mode activated
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), -0.06901, 1E-3);

    player->scenarioGateway->setObjectAlignModeZ(0, roadmanager::Position::ALIGN_MODE::ALIGN_NONE);
    player->scenarioGateway->updateObjectWorldPos(0, 0.0, 8.0, 67.0, 10.0, 1.57, 0.0, 0.0);
    player->Frame(0.05);
    // change expected since align mode deactivated
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 10.0, 1E-3);

    player->scenarioGateway->setObjectAlignModeZ(0, roadmanager::Position::ALIGN_MODE::ALIGN_SOFT);
    player->scenarioGateway->updateObjectWorldPos(0, 0.0, 12.0, 400.0, 10.0, 1.57, 0.0, 0.0);
    player->Frame(0.05);
    // Keep same relative height from road
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), 9.34987, 1E-3);

    player->scenarioGateway->setObjectAlignModeZ(0, roadmanager::Position::ALIGN_MODE::ALIGN_HARD);
    player->scenarioGateway->updateObjectWorldPos(0, 0.0, 12.0, 400.0, 10.0, 1.57, 0.0, 0.0);
    player->Frame(0.05);
    // Align to road surface
    EXPECT_NEAR(player->scenarioEngine->entities_.object_[0]->pos_.GetZ(), -0.719140, 1E-3);

    delete player;
}

int main(int argc, char** argv)
{
    //testing::GTEST_FLAG(filter) = "*TestCustomCameraVariants*";

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
