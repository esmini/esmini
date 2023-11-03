#include <gtest/gtest.h>
#include <cstring>
#include <iostream>
#include <chrono>
#include <dirent.h>

#include "ScenarioEngine.hpp"
#include "esminiLib.hpp"
#include "CommonMini.hpp"
#include "DatLogger.hpp"
#include "Dat2csv.hpp"
#include "Replay.hpp"

using namespace datLogger;
using namespace scenarioengine;

TEST(TestReplayer, WithOneObject)
{
    std::string fileName    = "sim.dat";
    std::string odrFileName = "e6mini.xodr";
    std::string model_Name  = "e6mini.osgb";
    int         version_    = 2;

    DatLogger* logger = new DatLogger;

    logger->init(fileName, version_, odrFileName, model_Name);

    double x     = 1.0;
    double y     = 2.0;
    double z     = 3.0;
    double h     = 4.0;
    double r     = 5.0;
    double p     = 6.0;
    double speed = 1.0;

    double current_time = 0.033;
    int    no_of_obj    = 1;
    int    total_time   = 6;

    for (int i = 0; i < total_time; i++)
    {
        if (i == 4 || i == 5)
        {
            h = 6.0;
        }
        logger->simTimeTemp = current_time;
        for (int j = 0; j < no_of_obj; j++)
        {
            if (i == 2 && j == 2)
            {
                break;  // delete one object.
            }
            int object_id = j;
            logger->AddObject(object_id);
            logger->WriteObjPos(object_id, x, y, z, h, p, r);
            logger->WriteObjSpeed(object_id, speed);
            logger->ObjIdPkgAdded = false;
        }
        if (i != 3)
        {
            speed += 1.0;
        }
        current_time += 1.089;
        logger->deleteObject();
        logger->TimePkgAdded = false;
    }

    delete logger;

    std::unique_ptr<scenarioengine::Replay> replayer_ = std::make_unique<scenarioengine::Replay>(fileName);

    ASSERT_EQ(replayer_->pkgs_.size(), 21);  // header not stored, 1 scenario end pkg added
    ASSERT_EQ(replayer_->scenarioState.sim_time, replayer_->GetTimeFromCnt(1));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 2);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(2));
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, 1.1220000000000001);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 2.0);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(4));
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->GetTimeFromCnt(4));
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 4.0);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(5));
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->GetTimeFromCnt(5));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 4.0);
}

TEST(TestReplayer, WithTwoObject)
{
    std::string fileName    = "sim.dat";
    std::string odrFileName = "e6mini.xodr";
    std::string model_Name  = "e6mini.osgb";
    int         version_    = 2;

    DatLogger* logger = new DatLogger;

    logger->init(fileName, version_, odrFileName, model_Name);

    double x     = 1.0;
    double y     = 2.0;
    double z     = 3.0;
    double h     = 4.0;
    double r     = 5.0;
    double p     = 6.0;
    double speed = 1.0;

    double current_time = 0.033;

    int no_of_obj  = 2;
    int total_time = 6;

    for (int i = 0; i < total_time; i++)
    {
        if (i == 4 || i == 5)
        {
            h = 6.0;
        }
        logger->simTimeTemp = current_time;
        for (int j = 0; j < no_of_obj; j++)
        {
            if (i == 2 && j == 2)
            {
                break;  // delete one object.
            }
            int object_id = j;
            logger->AddObject(object_id);
            logger->WriteObjPos(object_id, x, y, z, h, p, r);
            logger->WriteObjSpeed(object_id, speed);
            logger->ObjIdPkgAdded = false;
        }
        if (i != 3)
        {
            speed += 1.0;
        }
        current_time += 1.089;
        logger->deleteObject();
        logger->TimePkgAdded = false;
    }

    delete logger;

    std::unique_ptr<scenarioengine::Replay> replayer_ = std::make_unique<scenarioengine::Replay>(fileName);

    ASSERT_EQ(replayer_->pkgs_.size(), 35);  // header not stored, 1 scenario end pkg added
    ASSERT_EQ(replayer_->scenarioState.sim_time, replayer_->GetTimeFromCnt(1));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 2);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 2);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(2));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetTimeFromCnt(2), 1.1220000000000001);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 2.0);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[1].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[1].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[1].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[1].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[1].id), 2.0);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(4));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 2);
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->GetTimeFromCnt(4));
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 4.0);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[1].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[1].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[1].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[1].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[1].id), 4.0);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(5));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 2);
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->GetTimeFromCnt(5));
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 4.0);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[1].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[1].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[1].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[1].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[1].id), 4.0);
}

TEST(TestReplayer, WithTwoObjectAndAddAndDelete)
{
    std::string fileName    = "sim.dat";
    std::string odrFileName = "e6mini.xodr";
    std::string model_Name  = "e6mini.osgb";
    int         version_    = 2;

    DatLogger* logger = new DatLogger;

    logger->init(fileName, version_, odrFileName, model_Name);

    double x     = 1.0;
    double y     = 2.0;
    double z     = 3.0;
    double h     = 4.0;
    double r     = 5.0;
    double p     = 6.0;
    double speed = 1.0;

    double current_time = 0.033;

    int no_of_obj  = 3;
    int total_time = 6;

    for (int i = 0; i < total_time; i++)
    {
        if (i == 4 || i == 5)
        {
            h = 6.0;
        }
        logger->simTimeTemp = current_time;
        for (int j = 0; j < no_of_obj; j++)
        {
            if (i == 2 && j == 2)
            {
                break;  // delete one object.
            }
            int object_id = j;
            logger->AddObject(object_id);
            logger->WriteObjPos(object_id, x, y, z, h, p, r);
            logger->WriteObjSpeed(object_id, speed);
            logger->ObjIdPkgAdded = false;
        }
        if (i != 3)
        {
            speed += 1.0;
        }
        current_time += 1.089;
        logger->deleteObject();
        logger->TimePkgAdded = false;
    }

    delete logger;

    std::unique_ptr<scenarioengine::Replay> replayer_ = std::make_unique<scenarioengine::Replay>(fileName);
    ASSERT_EQ(replayer_->pkgs_.size(), 51);  // header not stored.

    ASSERT_EQ(replayer_->scenarioState.sim_time, replayer_->GetTimeFromCnt(1));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 3);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetTimeFromCnt(2), 1.122);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(2));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 3);
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->GetTimeFromCnt(2));
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 2.0);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[1].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[1].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[1].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[1].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[1].id), 2.0);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[2].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[2].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[2].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[2].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[2].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[2].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[2].id), 2.0);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(3));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 3);  // obj deleted
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->GetTimeFromCnt(3));
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].id, 0);
    ASSERT_EQ(replayer_->scenarioState.obj_states[1].id, 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[2].id, 2);
    ASSERT_EQ(replayer_->scenarioState.obj_states[2].active, false);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 3.0);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[1].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[1].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[1].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[1].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[1].id), 3.0);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(4));
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->GetTimeFromCnt(4));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 3);  // obj added
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].id, 0);
    ASSERT_EQ(replayer_->scenarioState.obj_states[1].id, 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[2].id, 2);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 4.0);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[1].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[1].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[1].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[1].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[1].id), 4.0);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[2].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[2].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[2].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[2].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[2].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[2].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[2].id), 4.0);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(5));
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->GetTimeFromCnt(5));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 4.0);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[1].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[1].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[1].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[1].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[1].id), 4.0);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[2].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[2].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[2].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[2].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[2].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[2].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[2].id), 4.0);

    std::unique_ptr<Dat2csv> dat_to_csv;
    dat_to_csv = std::make_unique<Dat2csv>("sim.dat");

    dat_to_csv->SetLogMode(Dat2csv::log_mode::ORIGINAL);
    dat_to_csv->SetIncludeRefs(true);
    dat_to_csv->CreateCSV();

    // Also check a few entries in the csv log file, focus on scenario controlled entity "Target"
    std::vector<std::vector<std::string>> csv;
    ASSERT_EQ(SE_ReadCSVFile("sim.csv", csv, 0), 0);
    EXPECT_NEAR(std::stod(csv[2][0]), 0.033, 1E-3);
    ASSERT_EQ(std::stod(csv[2][1]), 0);
    ASSERT_EQ(std::stod(csv[2][3]), 1.0);

    EXPECT_NEAR(std::stod(csv[3][0]), 0.033, 1E-3);
    ASSERT_EQ(std::stod(csv[3][1]), 1);
    ASSERT_EQ(std::stod(csv[2][3]), 1.0);

    EXPECT_NEAR(std::stod(csv[4][0]), 0.033, 1E-3);
    ASSERT_EQ(std::stod(csv[4][1]), 2);
    ASSERT_EQ(std::stod(csv[4][3]), 1.0);

    EXPECT_NEAR(std::stod(csv[8][0]), 2.211, 1E-3);
    ASSERT_EQ(std::stod(csv[8][1]), 0);

    EXPECT_NEAR(std::stod(csv[9][0]), 2.211, 1E-3);
    ASSERT_EQ(std::stod(csv[9][1]), 1);  // one object deleted

    EXPECT_NEAR(std::stod(csv[10][0]), 3.300, 1E-3);
    ASSERT_EQ(std::stod(csv[10][1]), 0);
    ASSERT_EQ(std::stod(csv[10][3]), 1.0);

    EXPECT_NEAR(std::stod(csv[11][0]), 3.300, 1E-3);
    ASSERT_EQ(std::stod(csv[11][1]), 1);
    ASSERT_EQ(std::stod(csv[11][3]), 1.0);

    EXPECT_NEAR(std::stod(csv[12][0]), 3.300, 1E-3);
    ASSERT_EQ(std::stod(csv[12][1]), 2);  // added back
    ASSERT_EQ(std::stod(csv[12][3]), 1.0);
}

TEST(TestReplayer, RepeatedObjectState)
{
    std::string fileName    = "sim.dat";
    std::string odrFileName = "e6mini.xodr";
    std::string model_Name  = "e6mini.osgb";
    int         version_    = 2;

    DatLogger* logger = new DatLogger;

    logger->init(fileName, version_, odrFileName, model_Name);

    double x     = 1.0;
    double y     = 2.0;
    double z     = 3.0;
    double h     = 4.0;
    double r     = 5.0;
    double p     = 6.0;
    double speed = 1.0;

    double current_time = 0.033;
    int    no_of_obj    = 1;
    int    total_time   = 6;

    for (int i = 0; i < total_time; i++)
    {
        logger->simTimeTemp = current_time;
        for (int j = 0; j < no_of_obj; j++)
        {
            int object_id = j;
            logger->AddObject(object_id);
            logger->WriteObjPos(object_id, x, y, z, h, p, r);
            logger->WriteObjSpeed(object_id, speed);
            logger->ObjIdPkgAdded = false;
        }
        current_time += 1.089;
        logger->deleteObject();
        logger->TimePkgAdded = false;
    }

    delete logger;

    std::unique_ptr<scenarioengine::Replay> replayer_ = std::make_unique<scenarioengine::Replay>(fileName);
    ASSERT_EQ(replayer_->pkgs_.size(), 7);  // header not stored, last time added

    ASSERT_EQ(replayer_->scenarioState.sim_time, replayer_->GetTimeFromCnt(1));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetTimeFromCnt(1), 0.033000000000000002);

    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 1.0);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(2));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    EXPECT_NEAR(replayer_->GetTimeFromCnt(2), 5.4779999999999998, 1E-3);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 1);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), 2);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 4);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 5);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 1.0);
}

TEST(TestReplayer, SimpleScenario)
{
    const char* args[] =
        {"--osc", "../../../EnvironmentSimulator/Unittest/xosc/simple_scenario.xosc", "--record", "new_sim.dat", "--fixed_timestep", "0.5"};

    SE_AddPath("../../../resources/models");
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);

    while (SE_GetQuitFlag() == 0)
    {
        SE_StepDT(0.05f);
    }

    SE_Close();

    scenarioengine::Replay* replayer_ = new scenarioengine::Replay("new_sim.dat");
    ASSERT_EQ(replayer_->pkgs_.size(), 3171);

    replayer_->MoveToTime(2.0);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 2.00, 1E-3);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 2.48064172555961);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelAngle(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelRot(replayer_->scenarioState.obj_states[0].id), 5.026893138885498);
    std::string name;
    replayer_->GetName(replayer_->scenarioState.obj_states[0].id, name);
    EXPECT_EQ(name, "Car");

    replayer_->MoveToTime(8.0);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 8.0, 1E-3);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 13.888889000000001);
    ASSERT_EQ(replayer_->GetCtrlType(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelRot(replayer_->scenarioState.obj_states[0].id), 5.949242115020752);
    ASSERT_EQ(replayer_->GetScaleMode(replayer_->scenarioState.obj_states[0].id), 0);

    replayer_->MoveToTime(15.0);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 15.0, 1E-3);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 13.888889000000001);
    ASSERT_EQ(replayer_->GetCtrlType(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelRot(replayer_->scenarioState.obj_states[0].id), 0.98368740081787109);
    ASSERT_EQ(replayer_->GetScaleMode(replayer_->scenarioState.obj_states[0].id), 0);

    // going back in time
    replayer_->MoveToTime(13.0);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 13.0, 1E-3);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 13.888889000000001);
    ASSERT_EQ(replayer_->GetCtrlType(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelRot(replayer_->scenarioState.obj_states[0].id), 3.3000152111053467);
    ASSERT_EQ(replayer_->GetScaleMode(replayer_->scenarioState.obj_states[0].id), 0);

    replayer_->MoveToTime(7.0);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 7.0, 1E-3);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 13.862463334428035);
    ASSERT_EQ(replayer_->GetCtrlType(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelRot(replayer_->scenarioState.obj_states[0].id), 3.9691176414489746);
    ASSERT_EQ(replayer_->GetScaleMode(replayer_->scenarioState.obj_states[0].id), 0);

    replayer_->MoveToTime(4.0);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 4.0, 1E-3);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 8.150334905687135);
    ASSERT_EQ(replayer_->GetCtrlType(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelRot(replayer_->scenarioState.obj_states[0].id), 3.7510917186737061);
    ASSERT_EQ(replayer_->GetScaleMode(replayer_->scenarioState.obj_states[0].id), 0);
}

TEST(TestReplayer, SpeedChangeScenario)
{
    const char* args[] =
        {"--osc", "../../../EnvironmentSimulator/Unittest/xosc/speed_change.xosc", "--record", "new_sim.dat", "--fixed_timestep", "0.5"};

    SE_AddPath("../../../resources/models");
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);

    while (SE_GetQuitFlag() == 0)
    {
        SE_StepDT(0.05f);
    }

    SE_Close();

    scenarioengine::Replay* replayer_ = new scenarioengine::Replay("new_sim.dat");
    ASSERT_EQ(replayer_->pkgs_.size(), 3212);

    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    replayer_->MoveToTime(18);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 18.0, 1E-3);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 113.66666721531664);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), -1.5349999999999999);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 0.0);
    ASSERT_EQ(replayer_->GetCtrlType(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_EQ(replayer_->GetObjCategory(replayer_->scenarioState.obj_states[0].id), 0);

    replayer_->MoveToTime(19.5);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 19.5, 1E-3);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 113.66666721531664);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), -1.5349999999999999);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 0.0);
    ASSERT_EQ(replayer_->GetCtrlType(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_EQ(replayer_->GetObjCategory(replayer_->scenarioState.obj_states[0].id), 0);

    replayer_->MoveToTime(21);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 21.0, 1E-3);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 113.90702692004841);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), -1.5349999999999999);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 0.69052735824166378);
    ASSERT_EQ(replayer_->GetCtrlType(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_EQ(replayer_->GetObjCategory(replayer_->scenarioState.obj_states[0].id), 0);
}

TEST(TestReplayer, TwoSimpleScenarioMerge)
{
    const char* args[] =
        {"--osc", "../../../EnvironmentSimulator/Unittest/xosc/simple_scenario.xosc", "--record", "simple_scenario_0.dat", "--fixed_timestep", "0.5"};

    SE_AddPath("../../../resources/models");
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);

    while (SE_GetQuitFlag() == 0)
    {
        SE_StepDT(0.05f);
    }

    SE_Close();

    const char* args1[] =
        {"--osc", "../../../EnvironmentSimulator/Unittest/xosc/speed_change.xosc", "--record", "simple_scenario_1.dat", "--fixed_timestep", "0.5"};

    SE_AddPath("../../../resources/models");
    ASSERT_EQ(SE_InitWithArgs(sizeof(args1) / sizeof(char*), args1), 0);

    while (SE_GetQuitFlag() == 0)
    {
        SE_StepDT(0.05f);
    }
    SE_Close();

    std::unique_ptr<scenarioengine::Replay> replayer_ = std::make_unique<scenarioengine::Replay>(".", "simple_scenario_", "");
    ASSERT_EQ(replayer_->pkgs_.size(), 5836);

    replayer_->MoveToTime(2.0);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 2);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 2.00, 1E-3);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].id, 0);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 2.48064172555961);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelAngle(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelRot(replayer_->scenarioState.obj_states[0].id), 5.026893138885498);
    std::string name;
    replayer_->GetName(replayer_->scenarioState.obj_states[0].id, name);
    EXPECT_EQ(name, "Car");
    ASSERT_EQ(replayer_->scenarioState.obj_states[1].id, 10);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[1].id), 1.9845132778764569);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelAngle(replayer_->scenarioState.obj_states[1].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelRot(replayer_->scenarioState.obj_states[1].id), 4.0215144157409668);
    replayer_->GetName(replayer_->scenarioState.obj_states[1].id, name);
    EXPECT_EQ(name, "Car");

    replayer_->MoveToTime(8.0);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 2);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 8.0, 1E-3);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].id, 0);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 13.888889000000001);
    ASSERT_EQ(replayer_->GetCtrlType(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelRot(replayer_->scenarioState.obj_states[0].id), 5.949242115020752);
    ASSERT_EQ(replayer_->GetScaleMode(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_EQ(replayer_->scenarioState.obj_states[1].id, 10);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[1].id), 11.111110999999999);
    ASSERT_EQ(replayer_->GetCtrlType(replayer_->scenarioState.obj_states[1].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelRot(replayer_->scenarioState.obj_states[1].id), 2.24611496925354);
    ASSERT_EQ(replayer_->GetScaleMode(replayer_->scenarioState.obj_states[1].id), 1);

    replayer_->MoveToTime(15.0);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 2);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 15.0, 1E-3);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].id, 0);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 13.888889000000001);
    ASSERT_EQ(replayer_->GetCtrlType(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelRot(replayer_->scenarioState.obj_states[0].id), 0.98368740081787109);
    ASSERT_EQ(replayer_->GetScaleMode(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_EQ(replayer_->scenarioState.obj_states[1].id, 10);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[1].id), 3.472222150060575);
    ASSERT_EQ(replayer_->GetCtrlType(replayer_->scenarioState.obj_states[1].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetWheelRot(replayer_->scenarioState.obj_states[1].id), 0.25884237885475159);
    ASSERT_EQ(replayer_->GetScaleMode(replayer_->scenarioState.obj_states[1].id), 1);
}

TEST(TestReplayer, ShowAndNotShowRestart)
{
    const char* args[] = {"--osc",
                          "../../../EnvironmentSimulator/Unittest/xosc/timing_scenario_with_restarts.xosc",
                          "--record",
                          "new_sim.dat",
                          "--fixed_timestep",
                          "0.1"};
    SE_AddPath("../../../resources/xosc");
    SE_AddPath("../../../resources/models");
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);

    while (SE_GetQuitFlag() == 0)
    {
        SE_StepDT(0.01f);
    }

    SE_Close();

    scenarioengine::Replay* replayer_ = new scenarioengine::Replay("new_sim.dat");
    ASSERT_EQ(replayer_->pkgs_.size(), 10429);

    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 10);
    ASSERT_DOUBLE_EQ(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), -1.5);
    ASSERT_DOUBLE_EQ(replayer_->GetZ(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetH(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetR(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetP(replayer_->scenarioState.obj_states[0].id), 0);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 10.0);
    ASSERT_EQ(replayer_->GetCtrlType(replayer_->scenarioState.obj_states[0].id), 1);
    ASSERT_EQ(replayer_->GetObjCategory(replayer_->scenarioState.obj_states[0].id), 0);

    std::string name;
    replayer_->GetName(replayer_->scenarioState.obj_states[0].id, name);
    EXPECT_EQ(name, "Ego");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 10.0, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), -1.5, 1E-3);

    replayer_->GetName(replayer_->scenarioState.obj_states[1].id, name);
    EXPECT_EQ(name, "Target");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 10.0, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), -4.5, 1E-3);

    replayer_->GetName(replayer_->scenarioState.obj_states[2].id, name);
    EXPECT_EQ(name, "Ego_ghost");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[2].id), 10.0, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[2].id), -1.5, 1E-3);

    // with show restart
    replayer_->SetShowRestart(true);
    replayer_->GetRestartTimes();

    ASSERT_EQ(replayer_->restartTimes.size(), 2);
    ASSERT_EQ(replayer_->restartTimes[0].restart_index_, 2161);
    ASSERT_EQ(replayer_->restartTimes[0].next_index_, 2528);
    ASSERT_DOUBLE_EQ(replayer_->restartTimes[0].restart_time_, 2.009999955072999);
    ASSERT_DOUBLE_EQ(replayer_->restartTimes[0].next_time_, 2.0199999548494829);
    ASSERT_EQ(replayer_->restartTimes[1].restart_index_, 8065);
    ASSERT_EQ(replayer_->restartTimes[1].next_index_, 8432);
    ASSERT_DOUBLE_EQ(replayer_->restartTimes[1].restart_time_, 8.00999982096255);
    ASSERT_DOUBLE_EQ(replayer_->restartTimes[1].next_time_, 8.019999820739022);

    replayer_->MoveToTime(replayer_->restartTimes[0].restart_time_);  // frame first restart triggered
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 3);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->restartTimes[0].restart_time_);
    replayer_->GetName(replayer_->scenarioState.obj_states[0].id, name);
    EXPECT_EQ(name, "Ego");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 10.0, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), -1.5, 1E-3);
    replayer_->GetName(replayer_->scenarioState.obj_states[1].id, name);
    EXPECT_EQ(name, "Target");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 50.199, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), -4.5, 1E-3);
    replayer_->GetName(replayer_->scenarioState.obj_states[2].id, name);
    EXPECT_EQ(name, "Ego_ghost");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[2].id), 60.099, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[2].id), -1.5, 1E-3);

    replayer_->MoveToTime(replayer_->restartTimes[0].next_time_);  // shall go first restart started frame
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, -0.93000004515051837);
    replayer_->GetName(replayer_->scenarioState.obj_states[0].id, name);
    EXPECT_EQ(name, "Ego");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 10.0, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), -1.5, 1E-3);
    replayer_->GetName(replayer_->scenarioState.obj_states[1].id, name);
    EXPECT_EQ(name, "Target");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 50.399, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), -4.5, 1E-3);
    replayer_->GetName(replayer_->scenarioState.obj_states[2].id, name);
    EXPECT_EQ(name, "Ego_ghost");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[2].id), 10.000, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[2].id), -1.5, 1E-3);

    // check reverse from first restart start time

    replayer_->MoveToTime(replayer_->scenarioState.sim_time - 0.05);  // going back from first restart started frame
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 3);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->restartTimes[0].restart_time_);  // shall go to first restart triggered time
    replayer_->GetName(replayer_->scenarioState.obj_states[0].id, name);
    EXPECT_EQ(name, "Ego");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 10.0, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), -1.5, 1E-3);
    replayer_->GetName(replayer_->scenarioState.obj_states[1].id, name);
    EXPECT_EQ(name, "Target");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 50.199, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), -4.5, 1E-3);
    replayer_->GetName(replayer_->scenarioState.obj_states[2].id, name);
    EXPECT_EQ(name, "Ego_ghost");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[2].id), 60.099, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[2].id), -1.5, 1E-3);

    replayer_->MoveToTime(replayer_->scenarioState.sim_time + 0.05);  // go forward to test second restart
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 3);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, -0.93000004515051837);

    // check second restart
    replayer_->MoveToTime(replayer_->restartTimes[1].restart_time_);  // frame second restart triggered
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 3);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->restartTimes[1].restart_time_);

    replayer_->MoveToTime(replayer_->restartTimes[1].next_time_);  // shall go second restart started frame
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, 5.0699998207390324);

    // check reverse from first restart start time
    replayer_->MoveToTime(replayer_->scenarioState.sim_time - 0.05);                                // shall go second restart frame
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->restartTimes[1].restart_time_);  // shall go to second restart triggered time

    // with no show restart
    replayer_->InitiateStates();
    replayer_->SetShowRestart(false);

    replayer_->MoveToTime(replayer_->restartTimes[0].restart_time_);  // first restart triggered frame
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 3);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    replayer_->GetName(replayer_->scenarioState.obj_states[0].id, name);
    EXPECT_EQ(name, "Ego");
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->restartTimes[0].restart_time_);
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 10.0, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), -1.5, 1E-3);
    replayer_->GetName(replayer_->scenarioState.obj_states[1].id, name);
    EXPECT_EQ(name, "Target");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 50.199, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), -4.5, 1E-3);
    replayer_->GetName(replayer_->scenarioState.obj_states[2].id, name);
    EXPECT_EQ(name, "Ego_ghost");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[2].id), 60.099, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[2].id), -1.5, 1E-3);

    replayer_->MoveToTime(replayer_->restartTimes[0].next_time_);  // shall go next frame from restart triggered frame
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->restartTimes[0].next_time_);
    replayer_->GetName(replayer_->scenarioState.obj_states[0].id, name);
    EXPECT_EQ(name, "Ego");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 10.0, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), -1.5, 1E-3);
    replayer_->GetName(replayer_->scenarioState.obj_states[1].id, name);
    EXPECT_EQ(name, "Target");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 50.399, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), -4.5, 1E-3);
    replayer_->GetName(replayer_->scenarioState.obj_states[2].id, name);
    EXPECT_EQ(name, "Ego_ghost");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[2].id), 23.724, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[2].id), -1.5, 1E-3);

    // check going back from first restart started time
    replayer_->MoveToTime(replayer_->scenarioState.sim_time - 0.05);  // going back first restart frame
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 3);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    replayer_->GetName(replayer_->scenarioState.obj_states[0].id, name);
    EXPECT_EQ(name, "Ego");
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, 1.9699999548494829);  // shall go requested time
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[0].id), 10.0, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[0].id), -1.5, 1E-3);
    replayer_->GetName(replayer_->scenarioState.obj_states[1].id, name);
    EXPECT_EQ(name, "Target");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[1].id), 50.399999096989632, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[1].id), -4.5, 1E-3);
    replayer_->GetName(replayer_->scenarioState.obj_states[2].id, name);
    EXPECT_EQ(name, "Ego_ghost");
    EXPECT_NEAR(replayer_->GetX(replayer_->scenarioState.obj_states[2].id), 23.166686680659474, 1E-3);
    EXPECT_NEAR(replayer_->GetY(replayer_->scenarioState.obj_states[2].id), -1.5, 1E-3);

    // check second restart
    replayer_->MoveToTime(replayer_->restartTimes[1].restart_time_);  // second restart frame
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 3);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->restartTimes[1].restart_time_);

    replayer_->MoveToTime(replayer_->restartTimes[1].next_time_);  // shall go next frame
    ASSERT_DOUBLE_EQ(replayer_->scenarioState.sim_time, replayer_->restartTimes[1].next_time_);
}

TEST(TestReplayer, StopAtEachTimeFrame)
{
    const char* args[] =
        {"--osc", "../../../EnvironmentSimulator/Unittest/xosc/simple_scenario.xosc", "--record", "new_sim.dat", "--fixed_timestep", "0.5"};
    SE_AddPath("../../../resources/xosc");
    SE_AddPath("../../../resources/models");
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);

    while (SE_GetQuitFlag() == 0)
    {
        SE_StepDT(0.5f);
    }

    SE_Close();

    scenarioengine::Replay* replayer_ = new scenarioengine::Replay("new_sim.dat");
    ASSERT_EQ(replayer_->pkgs_.size(), 342);

    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 18);
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    replayer_->MoveToTime(0.0);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 0.0, 1E-3);

    replayer_->MoveToTime(0.5);
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 0.5, 1E-3);

    replayer_->MoveToTime(3, true);  // go through each frame
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 1, 1E-3);

    replayer_->MoveToTime(1.5, true);  // go through each frame
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 1.5, 1E-3);

    replayer_->MoveToTime(3, true);  // go through each frame
    EXPECT_NEAR(replayer_->scenarioState.sim_time, 2.0, 1E-3);
}

TEST(TestDat2Csv, TimeModes)
{
    const char* args[] = {"--osc", "../../../EnvironmentSimulator/Unittest/xosc/test_time_mode.xosc", "--record", "sim.dat"};
    SE_AddPath("../../../resources/xosc");
    SE_AddPath("../../../../resources/models");
    ASSERT_EQ(SE_InitWithArgs(sizeof(args) / sizeof(char*), args), 0);

    while (SE_GetQuitFlag() == 0)
    {
        SE_StepDT(0.01f);
    }

    SE_Close();

    std::unique_ptr<Dat2csv> dat_to_csv;
    dat_to_csv = std::make_unique<Dat2csv>("sim.dat");

    dat_to_csv->SetLogMode(Dat2csv::log_mode::ORIGINAL);
    dat_to_csv->SetIncludeRefs(true);
    dat_to_csv->CreateCSV();

    // Also check a few entries in the csv log file, focus on scenario controlled entity "Target"
    std::vector<std::vector<std::string>> csv_original;
    ASSERT_EQ(SE_ReadCSVFile("sim.csv", csv_original, 0), 0);
    EXPECT_NEAR(std::stod(csv_original[2][0]), 0.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_original[2][3]), 10.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_original[3][0]), 4.020, 1E-3);
    EXPECT_NEAR(std::stod(csv_original[3][3]), 25.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_original[4][0]), 8.340, 1E-3);
    EXPECT_NEAR(std::stod(csv_original[4][3]), 50.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_original[5][0]), 10.010, 1E-3);
    EXPECT_NEAR(std::stod(csv_original[5][3]), 50.0, 1E-3);

    std::unique_ptr<Dat2csv> dat_to_csv1;
    dat_to_csv1 = std::make_unique<Dat2csv>("sim.dat");

    dat_to_csv1->SetLogMode(Dat2csv::log_mode::MIN_STEP);
    dat_to_csv1->SetIncludeRefs(true);
    dat_to_csv1->CreateCSV();

    std::vector<std::vector<std::string>> csv_min_step;
    ASSERT_EQ(SE_ReadCSVFile("sim.csv", csv_min_step, 0), 0);
    EXPECT_NEAR(std::stod(csv_min_step[2][0]), 0.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step[2][3]), 10.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step[3][0]), 1.670, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step[3][3]), 10.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step[4][0]), 3.340, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step[4][3]), 10.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step[5][0]), 5.010, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step[5][3]), 25.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step[6][0]), 6.680, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step[6][3]), 25.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step[7][0]), 8.350, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step[7][3]), 50.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step[8][0]), 10.010, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step[8][3]), 50.0, 1E-3);

    std::unique_ptr<Dat2csv> dat_to_csv2;
    dat_to_csv2 = std::make_unique<Dat2csv>("sim.dat");

    dat_to_csv2->SetLogMode(Dat2csv::log_mode::MIN_STEP_MIXED);
    dat_to_csv2->SetIncludeRefs(true);
    dat_to_csv2->CreateCSV();

    std::vector<std::vector<std::string>> csv_min_step_mixed;
    ASSERT_EQ(SE_ReadCSVFile("sim.csv", csv_min_step_mixed, 0), 0);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[2][0]), 0.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[2][3]), 10.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[3][0]), 1.670, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[3][3]), 10.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[4][0]), 3.340, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[4][3]), 10.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[5][0]), 4.020, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[5][3]), 25.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[6][0]), 5.010, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[6][3]), 25.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[7][0]), 6.680, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[7][3]), 25.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[8][0]), 8.340, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[8][3]), 50.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[9][0]), 8.350, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[9][3]), 50.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[10][0]), 10.010, 1E-3);
    EXPECT_NEAR(std::stod(csv_min_step_mixed[10][3]), 50.0, 1E-3);

    std::unique_ptr<Dat2csv> dat_to_csv3;
    dat_to_csv3 = std::make_unique<Dat2csv>("sim.dat");

    dat_to_csv3->SetLogMode(Dat2csv::log_mode::CUSTOM_TIME_STEP);
    dat_to_csv3->SetIncludeRefs(true);
    dat_to_csv3->SetStepTime(1);
    dat_to_csv3->CreateCSV();

    std::vector<std::vector<std::string>> csv_time_step;
    ASSERT_EQ(SE_ReadCSVFile("sim.csv", csv_time_step, 0), 0);
    EXPECT_NEAR(std::stod(csv_time_step[2][0]), 0.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step[2][3]), 10.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step[3][0]), 1.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step[3][3]), 10.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step[4][0]), 2.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step[4][3]), 10.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step[5][0]), 3.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step[5][3]), 10.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step[6][0]), 4.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step[6][3]), 10.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step[7][0]), 5.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step[7][3]), 25.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step[8][0]), 6.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step[8][3]), 25.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step[9][0]), 7.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step[9][3]), 25.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step[10][0]), 8.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step[10][3]), 25.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step[11][0]), 9.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step[11][3]), 50.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step[12][0]), 10.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step[12][3]), 50.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step[13][0]), 10.010, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step[13][3]), 50.0, 1E-3);

    std::unique_ptr<Dat2csv> dat_to_csv4;
    dat_to_csv4 = std::make_unique<Dat2csv>("sim.dat");

    dat_to_csv4->SetLogMode(Dat2csv::log_mode::CUSTOM_TIME_STEP_MIXED);
    dat_to_csv4->SetStepTime(1);
    dat_to_csv4->SetIncludeRefs(true);
    dat_to_csv4->CreateCSV();

    std::vector<std::vector<std::string>> csv_time_step_mixed;
    ASSERT_EQ(SE_ReadCSVFile("sim.csv", csv_time_step_mixed, 0), 0);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[2][0]), 0.0, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[2][3]), 10.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step_mixed[3][0]), 1.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[3][3]), 10.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step_mixed[4][0]), 2.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[4][3]), 10.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step_mixed[5][0]), 3.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[5][3]), 10.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step_mixed[6][0]), 4.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[6][3]), 10.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step_mixed[7][0]), 4.020, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[7][3]), 25.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step_mixed[8][0]), 5.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[8][3]), 25.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step_mixed[9][0]), 6.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[9][3]), 25.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step_mixed[10][0]), 7.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[10][3]), 25.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step_mixed[11][0]), 8.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[11][3]), 25.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step_mixed[12][0]), 8.340, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[12][3]), 50.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step_mixed[13][0]), 9.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[13][3]), 50.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step_mixed[14][0]), 10.000, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[14][3]), 50.0, 1E-3);

    EXPECT_NEAR(std::stod(csv_time_step_mixed[15][0]), 10.010, 1E-3);
    EXPECT_NEAR(std::stod(csv_time_step_mixed[15][3]), 50.0, 1E-3);
}

TEST(TestLightStateInReplayer, SimpleTest)
{
    std::string fileName    = "sim.dat";
    std::string odrFileName = "e6mini.xodr";
    std::string model_Name  = "e6mini.osgb";
    int         version_    = 2;

    DatLogger* logger = new DatLogger;

    logger->init(fileName, version_, odrFileName, model_Name);

    double x     = 1.0;
    double y     = 2.0;
    double z     = 3.0;
    double h     = 4.0;
    double r     = 5.0;
    double p     = 6.0;
    double speed = 1.0;

    double diffuseRgb[3]  = {0.4, 0.0, 0.0};
    double emissionRgb[3] = {0.0, 0.0, 0.0};

    datLogger::LightState lightState_;

    double rgb_[4];
    rgb_[0] = diffuseRgb[0] + emissionRgb[0];
    rgb_[1] = diffuseRgb[1] + emissionRgb[1];
    rgb_[2] = diffuseRgb[2] + emissionRgb[2];
    rgb_[3] = emissionRgb[0] / rgb_[0];

    // convert
    datLogger::LightRGB rgb_value;
    rgb_value.red       = static_cast<unsigned char>(MIN(MAX(rgb_[0], 0.0), 255.0) * 255.0);
    rgb_value.green     = static_cast<unsigned char>(MIN(MAX(rgb_[1], 0.0), 255.0) * 255.0);
    rgb_value.blue      = static_cast<unsigned char>(MIN(MAX(rgb_[2], 0.0), 255.0) * 255.0);
    rgb_value.intensity = static_cast<unsigned char>(MIN(MAX(rgb_[3], 0.0), 255.0) * 255.0);

    lightState_.brake_lights.red       = rgb_value.red;
    lightState_.brake_lights.blue      = rgb_value.green;
    lightState_.brake_lights.green     = rgb_value.blue;
    lightState_.brake_lights.intensity = rgb_value.intensity;

    double current_time = 0.033;
    int    no_of_obj    = 1;
    int    total_time   = 6;

    for (int i = 0; i < total_time; i++)
    {
        if (i == 4 || i == 5)
        {
            h = 6.0;
        }
        if (i == 1)
        {
            diffuseRgb[0] = 0.5;
            rgb_[0]       = diffuseRgb[0] + emissionRgb[0];
            rgb_[1]       = diffuseRgb[1] + emissionRgb[1];
            rgb_[2]       = diffuseRgb[2] + emissionRgb[2];
            rgb_[3]       = emissionRgb[0] / rgb_[0];

            rgb_value.red       = static_cast<unsigned char>(MIN(MAX(rgb_[0], 0.0), 255.0) * 255.0);
            rgb_value.green     = static_cast<unsigned char>(MIN(MAX(rgb_[1], 0.0), 255.0) * 255.0);
            rgb_value.blue      = static_cast<unsigned char>(MIN(MAX(rgb_[2], 0.0), 255.0) * 255.0);
            rgb_value.intensity = static_cast<unsigned char>(MIN(MAX(rgb_[3], 0.0), 255.0) * 255.0);

            lightState_.brake_lights.red       = rgb_value.red;
            lightState_.brake_lights.blue      = rgb_value.green;
            lightState_.brake_lights.green     = rgb_value.blue;
            lightState_.brake_lights.intensity = rgb_value.intensity;
        }
        logger->simTimeTemp = current_time;
        for (int j = 0; j < no_of_obj; j++)
        {
            int object_id = j;
            logger->AddObject(object_id);
            logger->WriteObjPos(object_id, x, y, z, h, p, r);
            logger->WriteObjSpeed(object_id, speed);
            logger->WriteLightState(object_id, lightState_);
            logger->ObjIdPkgAdded = false;
        }
        if (i != 3)
        {
            speed += 1.0;
        }
        current_time += 1.089;
        logger->deleteObject();
        logger->TimePkgAdded = false;
    }

    delete logger;

    std::unique_ptr<scenarioengine::Replay> replayer_ = std::make_unique<scenarioengine::Replay>(fileName);

    Object::VehicleLightActionStatus light_state[Object::VehicleLightType::NUMBER_OF_VEHICLE_LIGHTS];

    ASSERT_EQ(replayer_->pkgs_.size(), 23);
    ASSERT_EQ(replayer_->scenarioState.sim_time, replayer_->GetTimeFromCnt(1));
    ASSERT_EQ(replayer_->scenarioState.obj_states.size(), 1);
    ASSERT_EQ(replayer_->scenarioState.obj_states[0].pkgs.size(), 3);
    ASSERT_DOUBLE_EQ(replayer_->GetTimeFromCnt(2), 1.1220000000000001);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(2));
    datLogger::Pos pos = replayer_->GetPos(replayer_->scenarioState.obj_states[0].id);
    ASSERT_DOUBLE_EQ(pos.x, 1);
    ASSERT_DOUBLE_EQ(pos.y, 2);
    ASSERT_DOUBLE_EQ(pos.z, 3);
    ASSERT_DOUBLE_EQ(pos.h, 4);
    ASSERT_DOUBLE_EQ(pos.r, 5);
    ASSERT_DOUBLE_EQ(pos.p, 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 2.0);
    replayer_->GetRgbValues(replayer_->scenarioState.obj_states[0].id, light_state);
    EXPECT_NEAR(light_state[5].diffuseRgb[0], diffuseRgb[0], 1E-2);
    EXPECT_NEAR(light_state[5].diffuseRgb[1], diffuseRgb[1], 1E-2);
    EXPECT_NEAR(light_state[5].diffuseRgb[2], diffuseRgb[2], 1E-2);
    EXPECT_NEAR(light_state[5].emissionRgb[0], emissionRgb[0], 1E-2);
    EXPECT_NEAR(light_state[5].emissionRgb[1], emissionRgb[1], 1E-2);
    EXPECT_NEAR(light_state[5].emissionRgb[2], emissionRgb[2], 1E-2);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(4));
    datLogger::Pos pos1 = replayer_->GetPos(replayer_->scenarioState.obj_states[0].id);
    ASSERT_DOUBLE_EQ(pos1.x, 1);
    ASSERT_DOUBLE_EQ(pos1.y, 2);
    ASSERT_DOUBLE_EQ(pos1.z, 3);
    ASSERT_DOUBLE_EQ(pos1.h, 4);
    ASSERT_DOUBLE_EQ(pos1.r, 5);
    ASSERT_DOUBLE_EQ(pos1.p, 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 4.0);

    replayer_->MoveToTime(replayer_->GetTimeFromCnt(5));
    datLogger::Pos pos2 = replayer_->GetPos(replayer_->scenarioState.obj_states[0].id);
    ASSERT_DOUBLE_EQ(pos2.x, 1);
    ASSERT_DOUBLE_EQ(pos2.y, 2);
    ASSERT_DOUBLE_EQ(pos2.z, 3);
    ASSERT_DOUBLE_EQ(pos2.h, 6);
    ASSERT_DOUBLE_EQ(pos2.r, 5);
    ASSERT_DOUBLE_EQ(pos2.p, 6);
    ASSERT_DOUBLE_EQ(replayer_->GetSpeed(replayer_->scenarioState.obj_states[0].id), 4.0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}