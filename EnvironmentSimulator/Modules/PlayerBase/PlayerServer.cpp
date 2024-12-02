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
#include "PlayerServer.hpp"
#include "playerbase.hpp"
#include "UDP.hpp"
#include "logger.hpp"

using namespace scenarioengine;

// #define SWAP_BYTE_ORDER_ESMINI  // Set when Ego state is sent from non Intel platforms, such as dSPACE

enum
{
    SERV_NOT_STARTED,
    SERV_RUNNING,
    SERV_STOP,
    SERV_STOPPED
};

static int       state = SERV_NOT_STARTED;
static SE_Thread thread;

namespace scenarioengine
{
    PlayerServer::~PlayerServer()
    {
        for (size_t i = 0; i < action_.size(); i++)
        {
            delete action_[i];
        }
        action_.clear();
    }

    int PlayerServer::AddAction(OSCAction *action)
    {
        LOG_INFO("Adding action {}", action->GetName());

        // abort any action of same type and object
        for (auto &a : action_)
        {
            if (a->action_type_ == action->action_type_ &&
                ((a->GetBaseType() != OSCAction::BaseType::PRIVATE) ||
                 (static_cast<OSCPrivateAction *>(a)->object_ == static_cast<OSCPrivateAction *>(action)->object_)))
            {
                LOG_WARN("Action {} of type {} already ongoing{}, stopping it",
                         a->GetName(),
                         a->Type2Str(),
                         a->GetBaseType() == OSCAction::BaseType::PRIVATE ? (" for " + static_cast<OSCPrivateAction *>(a)->object_->GetName()) : "");
                a->End();
            }
        }

        action_.push_back(action);

        counter_++;

        return 0;
    }

    void PlayerServer::DeleteAction(unsigned int index)
    {
        if (index < action_.size())
        {
            delete action_[index];
        }
        action_.erase(action_.begin() + index);
    }

    int PlayerServer::NumberOfActions()
    {
        return static_cast<int>(action_.size());
    }

    void PlayerServer::Step()
    {
        for (size_t i = 0; i < action_.size(); i++)
        {
            if (action_[i]->GetCurrentState() == OSCAction::State::COMPLETE)
            {
                LOG_INFO("Injected action {} finished", action_[i]->GetName().c_str());
                DeleteAction(static_cast<unsigned int>(i));
                i++;
            }
        }
    }

    std::string PlayerServer::Type2Name(UDP_ACTION_TYPE type)
    {
        switch (type)
        {
            case UDP_ACTION_TYPE::LANE_CHANGE_ACTION:
                return "LANE_CHANGE";
            case UDP_ACTION_TYPE::LANE_OFFSET_ACTION:
                return "LANE_OFFSET";
            case UDP_ACTION_TYPE::SPEED_ACTION:
                return "SPEED";
            default:
                return "Unknown";
        }
    }

    OSCAction::ActionType PlayerServer::Type2OSCActionType(UDP_ACTION_TYPE type)
    {
        switch (type)
        {
            case UDP_ACTION_TYPE::LANE_CHANGE_ACTION:
                return OSCAction::ActionType::LAT_LANE_CHANGE;
            case UDP_ACTION_TYPE::LANE_OFFSET_ACTION:
                return OSCAction::ActionType::LAT_LANE_OFFSET;
            case UDP_ACTION_TYPE::SPEED_ACTION:
                return OSCAction::ActionType::LONG_SPEED;
            default:
                return OSCAction::ActionType::UNDEFINED;
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
                LOG_ERROR("Unsupported transition shape: {}", shape);
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
                LOG_ERROR("Unsupported transition dimension: {}", dimension);
        }
    }

    void PlayerServer::InjectSpeedAction(SpeedActionStruct &action)
    {
        LongSpeedAction *a = new LongSpeedAction(nullptr);
        a->SetName("SpeedAction_" + std::to_string(counter_));
        a->object_ = player_->scenarioEngine->entities_.GetObjectById(action.id);

        SetTransitionShape(a->transition_, action.transition_shape);
        SetTransitionDimension(a->transition_, action.transition_dim);
        a->transition_.SetParamTargetVal(action.transition_value);
        a->max_num_executions_ = 1;

        LongSpeedAction::TargetAbsolute *target = new LongSpeedAction::TargetAbsolute;
        target->value_                          = action.speed;
        a->target_.reset(target);

        AddAction(a);
    }

    void PlayerServer::InjectLaneChangeAction(LaneChangeActionStruct &action)
    {
        LatLaneChangeAction *a = new LatLaneChangeAction(nullptr);
        a->SetName("LaneChangeAction_" + std::to_string(counter_));
        a->object_ = player_->scenarioEngine->entities_.GetObjectById(action.id);

        SetTransitionShape(a->transition_, action.transition_shape);
        SetTransitionDimension(a->transition_, action.transition_dim);
        a->transition_.SetParamTargetVal(action.transition_value);
        a->max_num_executions_ = 1;

        if (action.mode == 0)
        {
            LatLaneChangeAction::TargetAbsolute *target = new LatLaneChangeAction::TargetAbsolute;
            target->value_                              = action.target;
            a->target_                                  = target;
        }
        else
        {
            LatLaneChangeAction::TargetRelative *target = new LatLaneChangeAction::TargetRelative;
            target->object_                             = a->object_;
            target->value_                              = action.target;
            a->target_                                  = target;
        }

        AddAction(a);
    }

    void PlayerServer::InjectLaneOffsetAction(LaneOffsetActionStruct &action)
    {
        LatLaneOffsetAction *a = new LatLaneOffsetAction(nullptr);
        a->SetName("LaneOffsetAction_" + std::to_string(counter_));
        a->object_ = player_->scenarioEngine->entities_.GetObjectById(action.id);

        SetTransitionShape(a->transition_, action.transition_shape);
        a->max_num_executions_ = 1;

        LatLaneOffsetAction::TargetAbsolute *target = new LatLaneOffsetAction::TargetAbsolute;
        target->value_                              = action.offset;
        a->target_.reset(target);
        a->max_lateral_acc_ = action.maxLateralAcc;

        AddAction(a);
    }

    bool PlayerServer::InjectedActionOngoing(int action_type)
    {
        if (action_type < 0)
        {
            return action_.size() > 0;
        }
        else
        {
            for (size_t i = 0; i < action_.size(); i++)
            {
                if (action_[i]->action_type_ == static_cast<OSCAction::ActionType>(action_type))
                {
                    return true;
                }
            }
        }

        return false;
    }

    static void ServerThread(void *args)
    {
        ScenarioPlayer           *player  = reinterpret_cast<ScenarioPlayer *>(args);
        static unsigned short int iPortIn = ESMINI_DEFAULT_ACTION_INPORT;  // Port for incoming packages
        (void)iPortIn;
        ActionStruct buf;
        state                = SERV_NOT_STARTED;
        UDPServer *udpServer = new UDPServer(ESMINI_DEFAULT_ACTION_INPORT);
        if (udpServer->GetStatus() != 0)
        {
            LOG_ERROR("PlayerServer: Failed to open UDP socket");
            return;
        }

        LOG_INFO("PlayerServer listening on port {}", ESMINI_DEFAULT_ACTION_INPORT);

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
                        player->player_server_->InjectSpeedAction(buf.message.speed);
                        break;
                    case static_cast<int>(UDP_ACTION_TYPE::LANE_CHANGE_ACTION):
                        player->player_server_->InjectLaneChangeAction(buf.message.laneChange);
                        break;
                    case static_cast<int>(UDP_ACTION_TYPE::LANE_OFFSET_ACTION):
                        player->player_server_->InjectLaneOffsetAction(buf.message.laneOffset);
                        break;
                    case static_cast<int>(UDP_ACTION_TYPE::PLAY):
                        player->SetState(ScenarioPlayer::PlayerState::PLAYER_STATE_PLAYING);
                        break;
                    case static_cast<int>(UDP_ACTION_TYPE::PAUSE):
                        player->SetState(ScenarioPlayer::PlayerState::PLAYER_STATE_PAUSE);
                        break;
                    case static_cast<int>(UDP_ACTION_TYPE::STEP):
                        player->SetState(ScenarioPlayer::PlayerState::PLAYER_STATE_STEP);
                        player->Frame(true);
                        break;
                    case static_cast<int>(UDP_ACTION_TYPE::STEP_DT):
                        player->SetState(ScenarioPlayer::PlayerState::PLAYER_STATE_STEP);
                        player->Frame(buf.message.stepDT.dt, true);
                        break;
                    case static_cast<int>(UDP_ACTION_TYPE::QUIT):
                        player->SetQuitRequest(true);
                        break;
                    default:
                        LOG_ERROR("Action of type {} not supported", buf.action_type);
                }
            }
        }

        delete udpServer;

        state = SERV_STOPPED;
    }

    void PlayerServer::Start()
    {
        if (player_ != nullptr)
        {
            thread.Start(ServerThread, reinterpret_cast<void *>(player_));
        }
    }

    void PlayerServer::Stop()
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

        Reset();
    }

}  // namespace scenarioengine
