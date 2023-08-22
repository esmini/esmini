/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "CommonMini.hpp"
#include "ScenarioEngine.hpp"
#include "ActionServer.hpp"
#include "UDP.hpp"

using namespace scenarioengine;

// #define SWAP_BYTE_ORDER_ESMINI  // Set when Ego state is sent from non Intel platforms, such as dSPACE

enum
{
    SERV_NOT_STARTED,
    SERV_RUNNING,
    SERV_STOP,
    SERV_STOPPED
};

static int                             state = SERV_NOT_STARTED;
static SE_Thread                       thread;
static scenarioengine::ScenarioEngine *scenarioEngine = nullptr;

namespace actionserver
{
    ServerActions::~ServerActions()
    {
        for (size_t i = 0; i < action_.size(); i++)
        {
            delete action_[i].osc_action_;
        }
        action_.clear();
    }

    int ServerActions::AddAction(ServerAction action)
    {
        // add only if no action of the same type is already running
        for (size_t i = 0; i < action_.size(); i++)
        {
            if (action_[i].type_ == action.type_)
            {
                LOG("UDP action of type %s already ongoing. Skipping UDP %s action.",
                    Type2Name(action_[i].type_).c_str(),
                    action_[i].osc_action_->name_.c_str());
                return -1;
            }
        }

        mutex.Lock();
        action_.push_back(action);
        mutex.Unlock();

        return 0;
    }

    void ServerActions::DeleteAction(int index)
    {
        action_.erase(action_.begin() + index);
    }

    int ServerActions::NumberOfActions()
    {
        return static_cast<int>(action_.size());
    }

    void ServerActions::Step(double simTime, double dt)
    {
        for (size_t i = 0; i < action_.size(); i++)
        {
            if (!action_[i].osc_action_->IsActive())
            {
                mutex.Lock();
                LOG("Starting UDP %s action %s", Type2Name(action_[i].type_).c_str(), action_[i].osc_action_->name_.c_str());
                action_[i].osc_action_->Start(simTime, dt);
                mutex.Unlock();
            }
            else
            {
                mutex.Lock();
                action_[i].osc_action_->Step(simTime, dt);
                action_[i].osc_action_->UpdateState();
                mutex.Unlock();

                if (action_[i].osc_action_->state_ == OSCAction::State::COMPLETE)
                {
                    LOG("UDP action %s finished", action_[i].osc_action_->name_.c_str());
                    scenarioEngine->serverActions_.DeleteAction(static_cast<int>(i));
                    i++;
                }
            }
        }
    }

    std::string ServerActions::Type2Name(UDP_ACTION_TYPE type)
    {
        switch (type)
        {
            case UDP_ACTION_TYPE::LANE_CHANGE_ACTION:
                return "LANE_CHANGE";
            case UDP_ACTION_TYPE::LANE_OFFSET_ACTION:
                return "LANE_CHANGE";
            case UDP_ACTION_TYPE::SPEED_ACTION:
                return "LANE_CHANGE";
            default:
                return "Unknown";
        }
    }

    static void SetTransitionShape(OSCPrivateAction::TransitionDynamics &t, int shape)
    {
        switch (shape)
        {
            case 0:  // cubic
                t.shape_ = OSCPrivateAction::DynamicsShape::CUBIC;
                break;
            case 1:  // linear
                t.shape_ = OSCPrivateAction::DynamicsShape::LINEAR;
                break;
            case 2:  // sinusoidal
                t.shape_ = OSCPrivateAction::DynamicsShape::SINUSOIDAL;
                break;
            case 3:  // step
                t.shape_ = OSCPrivateAction::DynamicsShape::STEP;
                break;
            default:
                LOG("Unsupported transition shape: %d", shape);
        }
    }

    static void SetTransitionDimension(OSCPrivateAction::TransitionDynamics &t, int dimension)
    {
        switch (dimension)
        {
            case 0:  // distance
                t.dimension_ = OSCPrivateAction::DynamicsDimension::DISTANCE;
                break;
            case 1:  // rate
                t.dimension_ = OSCPrivateAction::DynamicsDimension::RATE;
                break;
            case 2:  // time
                t.dimension_ = OSCPrivateAction::DynamicsDimension::TIME;
                break;
            default:
                LOG("Unsupported transition dimension: %d", dimension);
        }
    }

    void InjectSpeedAction(SpeedActionStruct &action)
    {
        LongSpeedAction *a = new LongSpeedAction;
        a->name_           = "SpeedAction";
        a->object_         = scenarioEngine->entities_.GetObjectById(action.id);

        SetTransitionShape(a->transition_, action.transition_shape);
        SetTransitionDimension(a->transition_, action.transition_dim);
        a->transition_.SetParamTargetVal(action.transition_value);
        a->max_num_executions_ = 1;

        LongSpeedAction::TargetAbsolute *target = new LongSpeedAction::TargetAbsolute;
        target->value_                          = action.speed;
        a->target_.reset(target);

        scenarioEngine->serverActions_.AddAction({UDP_ACTION_TYPE::SPEED_ACTION, a});
    }

    void InjectLaneChangeAction(LaneChangeActionStruct &action)
    {
        LatLaneChangeAction *a = new LatLaneChangeAction;
        a->name_               = "LaneChangeAction";
        a->object_             = scenarioEngine->entities_.GetObjectById(action.id);

        SetTransitionShape(a->transition_, action.transition_shape);
        SetTransitionDimension(a->transition_, action.transition_dim);
        a->transition_.SetParamTargetVal(action.transition_value);
        a->max_num_executions_ = 1;

        if (action.mode == 0)
        {
            LatLaneChangeAction::TargetAbsolute *target = new LatLaneChangeAction::TargetAbsolute;
            target->value_                              = action.target;
            a->target_.reset(target);
        }
        else
        {
            LatLaneChangeAction::TargetRelative *target = new LatLaneChangeAction::TargetRelative;
            target->object_                             = a->object_;
            target->value_                              = action.target;
            a->target_.reset(target);
        }

        scenarioEngine->serverActions_.AddAction({UDP_ACTION_TYPE::LANE_CHANGE_ACTION, a});
    }

    void InjectLaneOffsetAction(LaneOffsetActionStruct &action)
    {
        LatLaneOffsetAction *a = new LatLaneOffsetAction;
        a->name_               = "LaneOffsetAction";
        a->object_             = scenarioEngine->entities_.GetObjectById(action.id);

        SetTransitionShape(a->transition_, action.transition_shape);
        a->max_num_executions_ = 1;

        LatLaneOffsetAction::TargetAbsolute *target = new LatLaneOffsetAction::TargetAbsolute;
        target->value_                              = action.offset;
        a->target_.reset(target);
        a->max_lateral_acc_ = action.maxLateralAcc;

        scenarioEngine->serverActions_.AddAction({UDP_ACTION_TYPE::LANE_OFFSET_ACTION, a});
    }

    static void ServerThread(void *args)
    {
        (void)args;
        static unsigned short int iPortIn = ESMINI_DEFAULT_ACTION_INPORT;  // Port for incoming packages
        (void)iPortIn;
        ActionStruct buf;
        state                = SERV_NOT_STARTED;
        UDPServer *udpServer = new UDPServer(ESMINI_DEFAULT_ACTION_INPORT);

        LOG("ActionServer listening on port %d", ESMINI_DEFAULT_ACTION_INPORT);

        state = SERV_RUNNING;

        while (state == SERV_RUNNING)
        {
            int ret = udpServer->Receive(reinterpret_cast<char *>(&buf), sizeof(ActionStruct));

#ifdef SWAP_BYTE_ORDER_ESMINI
            SwapByteOrder((unsigned char *)&buf, 4, sizeof(buf));
#endif

            if (ret >= 0)
            {
                switch (buf.action_type)
                {
                    case static_cast<int>(UDP_ACTION_TYPE::SPEED_ACTION):
                        InjectSpeedAction(buf.message.speed);
                        break;
                    case static_cast<int>(UDP_ACTION_TYPE::LANE_CHANGE_ACTION):
                        InjectLaneChangeAction(buf.message.laneChange);
                        break;
                    case static_cast<int>(UDP_ACTION_TYPE::LANE_OFFSET_ACTION):
                        InjectLaneOffsetAction(buf.message.laneOffset);
                        break;
                    default:
                        LOG("Action of type %d not supported", buf.action_type);
                }
            }
        }

        delete udpServer;

        state = SERV_STOPPED;
    }

    void StartActionServer(ScenarioEngine *scenario_engine)
    {
        scenarioEngine = scenario_engine;
        thread.Start(ServerThread, NULL);
    }

    void StopActionServer()
    {
        // Flag time to stop
        if (state == SERV_RUNNING)
        {
            state = SERV_STOP;
        }
        else
        {
            state = SERV_STOPPED;
        }

        // Wait/block until UDP server closed gracefully
        thread.Wait();
    }

}  // namespace actionserver
