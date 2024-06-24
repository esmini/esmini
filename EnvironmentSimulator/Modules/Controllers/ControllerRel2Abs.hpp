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

#include <string>
#include "EmbeddedController.hpp"
#include "pugixml.hpp"
#include "Parameters.hpp"
#include "Entities.hpp"
#include "OSCPrivateAction.hpp"

#define CONTROLLER_REL2ABS_TYPE_NAME "ControllerRel2Abs"

namespace scenarioengine::controller
{
    // base class for controllers
    class ControllerRel2Abs : public controller::EmbeddedController
    {
    public:
        struct PreSimData
        {
            std::vector<double> time;
            std::vector<double> speeds;
            std::vector<double> posX;
            std::vector<double> posY;
        };

        struct position_copy
        {
            Object*                object;
            roadmanager::Position* pos;
            double                 speed;
            int                    dirtyBits;
        };

        enum ctrl_mode
        {
            RELATIVE,
            ABSOLUTE
        };

        std::vector<PreSimData> data_vector;
        PreSimData              actualData;
        PreSimData              data;
        double                  timestamp;

        // Presim parameters
        double                       pred_horizon;
        double                       pred_timestep;
        double                       pred_nbr_timesteps;
        OSCPrivateAction::ActionType action_whitelist[7] = {OSCPrivateAction::ActionType::LONG_SPEED,
                                                            OSCPrivateAction::ActionType::LONG_DISTANCE,
                                                            OSCPrivateAction::ActionType::LAT_LANE_CHANGE,
                                                            OSCPrivateAction::ActionType::LAT_LANE_OFFSET,
                                                            OSCPrivateAction::ActionType::LAT_DISTANCE,
                                                            OSCPrivateAction::ActionType::ASSIGN_ROUTE,
                                                            OSCPrivateAction::ActionType::FOLLOW_TRAJECTORY};
        double                       switching_threshold_dist;
        double                       switching_threshold_speed;

        // CSV loggning
        std::ofstream logData;
        int           csv_iter;

        ControllerRel2Abs(InitArgs* args);

        //std::string GetName() const override;
    
        controller::Type GetType() const override;

        void Init();
        void Step(double timeStep);
        int  Activate(ControlActivationMode lat_activation_mode,
                      ControlActivationMode long_activation_mode,
                      ControlActivationMode light_activation_mode,
                      ControlActivationMode anim_activation_mode);
        virtual void ReportKeyEvent(int key, bool down);
        void CopyPosition(Object* object, position_copy* obj_copy);
        // Riz
        /*
        static const char* GetTypeNameStatic()
        {
            return CONTROLLER_REL2ABS_TYPE_NAME;
        }
        virtual const char* GetTypeName()
        {
            return GetTypeNameStatic();
        }
        static int GetTypeStatic()
        {
            return controller::Type::CONTROLLER_TYPE_REL2ABS;
        }
        virtual int GetType()
        {
            return GetTypeStatic();
        }
        */
    private:
        int    ego_obj;
        double prev_ego_speed;
        double prev_target_speed;
        bool   switchNow;
        // vector with pairs of timestep and speed before that timestep
        std::vector<std::pair<double, double>> speeds;

        // alters ego_obj to correct entities index for ego
        void findEgo();
    };

    ControllerBase* InstantiateControllerRel2Abs(void* args);
}  // namespace scenarioengine::controller