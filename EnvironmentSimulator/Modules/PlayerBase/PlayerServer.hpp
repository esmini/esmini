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

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include "Storyboard.hpp"

#define ESMINI_DEFAULT_ACTION_INPORT 48197

namespace scenarioengine
{
    enum class UDP_ACTION_TYPE
    {
        UNDEFINED          = 0,
        SPEED_ACTION       = 1,
        LANE_CHANGE_ACTION = 2,
        LANE_OFFSET_ACTION = 3,
        PLAY               = 4,
        PAUSE              = 5,
        STEP               = 6,
        STEP_DT            = 7,
        QUIT               = 8,
        NR_OF_ACTIONS      = 9,
    };

    struct SpeedActionStruct
    {
        int   id;  // id of object to perform action
        float speed;
        int   transition_shape;  // 0 = cubic, 1 = linear, 2 = sinusoidal, 3 = step
        int   transition_dim;    // 0 = distance, 1 = rate, 2 = time
        float transition_value;
    };

    struct LaneChangeActionStruct
    {
        int   id;                // id of object to perform action
        int   mode;              // 0 = absolute, 1 = relative (own vehicle)
        int   target;            // target lane id (absolute or relative)
        int   transition_shape;  // 0 = cubic, 1 = linear, 2 = sinusoidal, 3 = step
        int   transition_dim;    // 0 = distance, 1 = rate, 2 = time
        float transition_value;
    };

    struct LaneOffsetActionStruct
    {
        int   id;  // id of object to perform action
        float offset;
        float maxLateralAcc;
        int   transition_shape;  // 0 = cubic, 1 = linear, 2 = sinusoidal, 3 = step
    };

    struct StepDTStruct
    {
        float dt;  // timestep in seconds
    };

    struct ActionStruct
    {
        int action_type = 0;
        union
        {
            SpeedActionStruct      speed;
            LaneChangeActionStruct laneChange;
            LaneOffsetActionStruct laneOffset;
            StepDTStruct           stepDT;
        } message;
    };

    class ScenarioPlayer;  // forward declaration

    class PlayerServer
    {
    public:
        PlayerServer(ScenarioPlayer* player)
        {
            player_ = player;
        }
        ~PlayerServer();

        void InjectSpeedAction(SpeedActionStruct& action);
        void InjectLaneChangeAction(LaneChangeActionStruct& action);
        void InjectLaneOffsetAction(LaneOffsetActionStruct& action);

        int                                      AddAction(OSCAction* action);
        void                                     DeleteAction(int index);
        int                                      NumberOfActions();
        void                                     Step();
        std::string                              Type2Name(UDP_ACTION_TYPE type);
        std::vector<scenarioengine::OSCAction*>* GetInjectedActionsPtr()
        {
            return &action_;
        }

        void Start();
        void Stop();

    private:
        std::vector<OSCAction*> action_;
        ScenarioPlayer*         player_;
    };

}  // namespace scenarioengine
