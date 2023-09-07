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
#include "OSCAction.hpp"

#define ESMINI_DEFAULT_ACTION_INPORT 48197

namespace actionserver
{
    enum class UDP_ACTION_TYPE
    {
        UNDEFINED          = 0,
        SPEED_ACTION       = 1,
        LANE_CHANGE_ACTION = 2,
        LANE_OFFSET_ACTION = 3,
        NR_OF_ACTIONS      = 3
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
        float maxLateralAcc;     // 0 = distance, 1 = rate, 2 = time
        int   transition_shape;  // 0 = cubic, 1 = linear, 2 = sinusoidal, 3 = step
    };

    struct ActionStruct
    {
        int action_type = 0;
        union
        {
            SpeedActionStruct      speed;
            LaneChangeActionStruct laneChange;
            LaneOffsetActionStruct laneOffset;
        } message;
    };

    class ServerActions
    {
    public:
        struct ServerAction
        {
            UDP_ACTION_TYPE            type_;
            scenarioengine::OSCAction* osc_action_;
        };

        ~ServerActions();

        int         AddAction(ServerAction action);
        void        DeleteAction(int index);
        int         NumberOfActions();
        void        Step(double simTime, double dt);
        std::string Type2Name(UDP_ACTION_TYPE type);

    private:
        std::vector<ServerAction> action_;
        SE_Mutex                  mutex;
    };

    void StartActionServer(scenarioengine::ScenarioEngine* scenarioEngine);
    void StopActionServer();

}  // namespace actionserver
