#ifdef __EMSCRIPTEN__

#include "esminijs.hpp"

#include <cstddef>
#include <emscripten/bind.h>

namespace esmini
{
    EMSCRIPTEN_BINDINGS(OpenScenario)
    {
        emscripten::register_vector<std::string>("vector<string>");
        emscripten::register_vector<ScenarioObjectState>("vector<ScenarioObjectState>");

        emscripten::value_object<OpenScenarioConfig>("OpenScenarioConfig")
            .field("max_loop", &OpenScenarioConfig::max_loop)
            .field("min_time_step", &OpenScenarioConfig::min_time_step)
            .field("max_time_step", &OpenScenarioConfig::max_time_step)
            .field("dt", &OpenScenarioConfig::dt);

        emscripten::value_object<ScenarioObjectState>("ScenarioObjectState")
            .field("name", &ScenarioObjectState::name)
            .field("id", &ScenarioObjectState::id)
            .field("model_id", &ScenarioObjectState::model_id)
            .field("ctrl_type", &ScenarioObjectState::ctrl_type)
            .field("timestamp", &ScenarioObjectState::timestamp)
            .field("x", &ScenarioObjectState::x)
            .field("y", &ScenarioObjectState::y)
            .field("z", &ScenarioObjectState::z)
            .field("h", &ScenarioObjectState::h)
            .field("p", &ScenarioObjectState::p)
            .field("r", &ScenarioObjectState::r)
            .field("road_id", &ScenarioObjectState::road_id)
            .field("junction_id", &ScenarioObjectState::junction_id)
            .field("t", &ScenarioObjectState::t)
            .field("lane_id", &ScenarioObjectState::lane_id)
            .field("lane_offset", &ScenarioObjectState::lane_offset)
            .field("speed", &ScenarioObjectState::speed)
            .field("center_offset_x", &ScenarioObjectState::center_offset_x)
            .field("center_offset_y", &ScenarioObjectState::center_offset_y)
            .field("center_offset_z", &ScenarioObjectState::center_offset_z)
            .field("width", &ScenarioObjectState::width)
            .field("length", &ScenarioObjectState::length)
            .field("height", &ScenarioObjectState::height)
            .field("object_type", &ScenarioObjectState::object_type)
            .field("object_category", &ScenarioObjectState::object_category)
            .field("wheel_angle", &ScenarioObjectState::wheel_angle)
            .field("wheel_rot", &ScenarioObjectState::wheel_rot);

        emscripten::class_<OpenScenario>("OpenScenario")
            .constructor<std::string, OpenScenarioConfig>()
            .function("get_object_state", &OpenScenario::get_object_state, emscripten::allow_raw_pointers())
            .function("get_object_state_by_second", &OpenScenario::get_object_state_by_second);
    }
}  // namespace esmini

#endif