#include "esminijs.hpp"
#include "iostream"

namespace esmini
{

    void copyStateFromScenarioGateway(ScenarioObjectState *state, scenarioengine::ObjectStateStruct *gw_state)
    {
        state->id              = gw_state->info.id;
        state->model_id        = gw_state->info.model_id;
        state->ctrl_type       = gw_state->info.ctrl_type;
        state->name            = std::string(gw_state->info.name, NAME_LEN);
        state->timestamp       = gw_state->info.timeStamp;
        state->x               = (float)gw_state->pos.GetX();
        state->y               = (float)gw_state->pos.GetY();
        state->z               = (float)gw_state->pos.GetZ();
        state->h               = (float)gw_state->pos.GetH();
        state->p               = (float)gw_state->pos.GetP();
        state->r               = (float)gw_state->pos.GetR();
        state->speed           = (float)gw_state->info.speed;
        state->road_id         = (int)gw_state->pos.GetTrackId();
        state->junction_id     = (int)gw_state->pos.GetJunctionId();
        state->t               = (float)gw_state->pos.GetT();
        state->lane_id         = (int)gw_state->pos.GetLaneId();
        state->s               = (float)gw_state->pos.GetS();
        state->lane_offset     = (float)gw_state->pos.GetOffset();
        state->center_offset_x = gw_state->info.boundingbox.center_.x_;
        state->center_offset_y = gw_state->info.boundingbox.center_.y_;
        state->center_offset_z = gw_state->info.boundingbox.center_.z_;
        state->width           = gw_state->info.boundingbox.dimensions_.width_;
        state->length          = gw_state->info.boundingbox.dimensions_.length_;
        state->height          = gw_state->info.boundingbox.dimensions_.height_;
        state->object_type     = gw_state->info.obj_type;
        state->object_category = gw_state->info.obj_category;
        
        // Extract wheel information from wheel_data vector
        if (!gw_state->info.wheel_data.empty()) {
            // Use the first wheel's data for wheel angle and rotation
            const auto& wheel = gw_state->info.wheel_data[0];
            state->wheel_angle = (float)wheel.h;  // heading/yaw for wheel angle
            state->wheel_rot   = (float)wheel.rotation_rate;  // rotation rate for wheel rotation
        } else {
            // Default values when no wheel data available
            state->wheel_angle = 0.0f;
            state->wheel_rot   = 0.0f;
        }
    }

    OpenScenario::OpenScenario(const std::string &xosc_file, const OpenScenarioConfig &config) : xosc_file(xosc_file), config(config)
    {
        std::cout << "xosc file path:" << xosc_file << ",this->xosc_file path:" << this->xosc_file << std::endl;
        this->scenarioEngine  = new scenarioengine::ScenarioEngine(this->xosc_file, false);
        this->scenarioGateway = this->scenarioEngine->getScenarioGateway();
        std::cout << "init scenario success" << std::endl;
    }

    std::vector<ScenarioObjectState> OpenScenario::get_object_state(const OpenScenarioConfig *config)
    {
        OpenScenarioConfig _config = this->config;
        if (config != nullptr)
        {
            _config = *config;
        }

        std::cout << "config:" << _config << std::endl;

        std::vector<ScenarioObjectState> objects_sts;
        int                              retval = 0;
        double                           dt;
        int64_t                          time_stamp = 0;
        while (retval == 0 && _config.max_loop > 0)
        {
            if (_config.dt == 0)
            {
                dt = SE_getSimTimeStep(time_stamp, _config.min_time_step, _config.max_time_step);
            }
            else
            {
                dt = _config.dt;
            }
            // std::cout << "time stamp is: " << time_stamp << std::endl;
            // std::cout << "simulationtime is: " << scenarioEngine->getSimulationTime() << std::endl;

            retval = this->scenarioEngine->step(dt);
            // std::cout << "retval is: " << retval << std::endl;

            int numberofObjects = this->scenarioGateway->getNumberOfObjects();
            // std::cout << "number of objects: " << numberofObjects << std::endl;
            for (int i = 0; i < numberofObjects; i++)
            {
                scenarioengine::ObjectState obj_state;
                if (this->scenarioGateway->getObjectStateById(i, obj_state) != -1)
                {
                    ScenarioObjectState state;
                    copyStateFromScenarioGateway(&state, &obj_state.state_);
                    // std::cout << state << std::endl;
                    objects_sts.push_back(state);
                }

                this->scenarioEngine->prepareGroundTruth(dt);
            }
            --_config.max_loop;
        }

        return objects_sts;
    }

    std::vector<ScenarioObjectState> OpenScenario::get_object_state_by_second(const int second, const int fps)
    {
        OpenScenarioConfig _config;
        _config.max_loop = second * fps;
        _config.dt = 1.0 / fps;

        return this->get_object_state(&_config);
    }

}  // namespace esmini