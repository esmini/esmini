#ifdef __EMSCRIPTEN__

#include "esminijs.hpp"

#include <cstddef>
#include <emscripten/bind.h>

namespace esmini
{
    EMSCRIPTEN_BINDINGS(OpenScenario)
    {
        emscripten::register_vector<std::string>("vector<string>");
        emscripten::register_vector<RoadGeometryPoint>("vector<RoadGeometryPoint>");
        emscripten::register_vector<LaneSurfaceGeometry>("vector<LaneSurfaceGeometry>");
        emscripten::register_vector<RoadMarkGeometry>("vector<RoadMarkGeometry>");
        emscripten::register_vector<LaneCenterGeometry>("vector<LaneCenterGeometry>");
        emscripten::register_vector<RoadObjectGeometry>("vector<RoadObjectGeometry>");
        emscripten::register_vector<RoadFeatureBoxGeometry>("vector<RoadFeatureBoxGeometry>");
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
            .field("s", &ScenarioObjectState::s)
            .field("speed", &ScenarioObjectState::speed)
            .field("center_offset_x", &ScenarioObjectState::center_offset_x)
            .field("center_offset_y", &ScenarioObjectState::center_offset_y)
            .field("center_offset_z", &ScenarioObjectState::center_offset_z)
            .field("width", &ScenarioObjectState::width)
            .field("length", &ScenarioObjectState::length)
            .field("height", &ScenarioObjectState::height)
            .field("object_type", &ScenarioObjectState::object_type)
            .field("object_category", &ScenarioObjectState::object_category)
            .field("has_ghost", &ScenarioObjectState::has_ghost)
            .field("sensor_x", &ScenarioObjectState::sensor_x)
            .field("sensor_y", &ScenarioObjectState::sensor_y)
            .field("sensor_z", &ScenarioObjectState::sensor_z)
            .field("trail_x", &ScenarioObjectState::trail_x)
            .field("trail_y", &ScenarioObjectState::trail_y)
            .field("trail_z", &ScenarioObjectState::trail_z)
            .field("wheel_angle", &ScenarioObjectState::wheel_angle)
            .field("wheel_rot", &ScenarioObjectState::wheel_rot);

        emscripten::value_object<ScenarioFrame>("ScenarioFrame")
            .field("simulation_time", &ScenarioFrame::simulation_time)
            .field("time_step", &ScenarioFrame::time_step)
            .field("status", &ScenarioFrame::status)
            .field("quit", &ScenarioFrame::quit)
            .field("object_states", &ScenarioFrame::object_states);

        emscripten::value_object<RoadGeometryPoint>("RoadGeometryPoint")
            .field("s", &RoadGeometryPoint::s)
            .field("x", &RoadGeometryPoint::x)
            .field("y", &RoadGeometryPoint::y)
            .field("z", &RoadGeometryPoint::z)
            .field("h", &RoadGeometryPoint::h)
            .field("p", &RoadGeometryPoint::p)
            .field("r", &RoadGeometryPoint::r)
            .field("endpoint", &RoadGeometryPoint::endpoint);

        emscripten::value_object<LaneSurfaceGeometry>("LaneSurfaceGeometry")
            .field("road_id", &LaneSurfaceGeometry::road_id)
            .field("lane_section_index", &LaneSurfaceGeometry::lane_section_index)
            .field("lane_id", &LaneSurfaceGeometry::lane_id)
            .field("lane_type", &LaneSurfaceGeometry::lane_type)
            .field("road_edge", &LaneSurfaceGeometry::road_edge)
            .field("left_boundary", &LaneSurfaceGeometry::left_boundary)
            .field("right_boundary", &LaneSurfaceGeometry::right_boundary);

        emscripten::value_object<RoadMarkGeometry>("RoadMarkGeometry")
            .field("road_id", &RoadMarkGeometry::road_id)
            .field("lane_section_index", &RoadMarkGeometry::lane_section_index)
            .field("lane_id", &RoadMarkGeometry::lane_id)
            .field("type", &RoadMarkGeometry::type)
            .field("color", &RoadMarkGeometry::color)
            .field("width", &RoadMarkGeometry::width)
            .field("line_length", &RoadMarkGeometry::line_length)
            .field("line_space", &RoadMarkGeometry::line_space)
            .field("points", &RoadMarkGeometry::points);

        emscripten::value_object<LaneCenterGeometry>("LaneCenterGeometry")
            .field("road_id", &LaneCenterGeometry::road_id)
            .field("lane_section_index", &LaneCenterGeometry::lane_section_index)
            .field("lane_id", &LaneCenterGeometry::lane_id)
            .field("reference_line", &LaneCenterGeometry::reference_line)
            .field("center_lane", &LaneCenterGeometry::center_lane)
            .field("points", &LaneCenterGeometry::points);

        emscripten::value_object<RoadObjectGeometry>("RoadObjectGeometry")
            .field("road_id", &RoadObjectGeometry::road_id)
            .field("id", &RoadObjectGeometry::id)
            .field("name", &RoadObjectGeometry::name)
            .field("type", &RoadObjectGeometry::type)
            .field("width_start", &RoadObjectGeometry::width_start)
            .field("width_end", &RoadObjectGeometry::width_end)
            .field("height_start", &RoadObjectGeometry::height_start)
            .field("height_end", &RoadObjectGeometry::height_end)
            .field("points", &RoadObjectGeometry::points);

        emscripten::value_object<RoadFeatureBoxGeometry>("RoadFeatureBoxGeometry")
            .field("road_id", &RoadFeatureBoxGeometry::road_id)
            .field("id", &RoadFeatureBoxGeometry::id)
            .field("name", &RoadFeatureBoxGeometry::name)
            .field("type", &RoadFeatureBoxGeometry::type)
            .field("kind", &RoadFeatureBoxGeometry::kind)
            .field("x", &RoadFeatureBoxGeometry::x)
            .field("y", &RoadFeatureBoxGeometry::y)
            .field("z", &RoadFeatureBoxGeometry::z)
            .field("h", &RoadFeatureBoxGeometry::h)
            .field("p", &RoadFeatureBoxGeometry::p)
            .field("r", &RoadFeatureBoxGeometry::r)
            .field("width", &RoadFeatureBoxGeometry::width)
            .field("length", &RoadFeatureBoxGeometry::length)
            .field("height", &RoadFeatureBoxGeometry::height);

        emscripten::value_object<ScenarioRoadGeometry>("ScenarioRoadGeometry")
            .field("odr_filename", &ScenarioRoadGeometry::odr_filename)
            .field("lane_surfaces", &ScenarioRoadGeometry::lane_surfaces)
            .field("road_marks", &ScenarioRoadGeometry::road_marks)
            .field("lane_centers", &ScenarioRoadGeometry::lane_centers)
            .field("road_objects", &ScenarioRoadGeometry::road_objects)
            .field("road_feature_boxes", &ScenarioRoadGeometry::road_feature_boxes);

        emscripten::class_<OpenScenario>("OpenScenario")
            .constructor<std::string, OpenScenarioConfig>()
            .function("step", &OpenScenario::step)
            .function("step_frame", &OpenScenario::step_frame)
            .function("reset", &OpenScenario::reset)
            .function("get_object_count", &OpenScenario::get_object_count)
            .function("get_object_states", &OpenScenario::get_object_states)
            .function("get_object_state_by_index", &OpenScenario::get_object_state_by_index)
            .function("get_road_geometry", &OpenScenario::get_road_geometry)
            .function("get_simulation_time", &OpenScenario::get_simulation_time)
            .function("is_quit", &OpenScenario::is_quit)
            .function("get_object_state", &OpenScenario::get_object_state, emscripten::allow_raw_pointers())
            .function("get_object_state_by_second", &OpenScenario::get_object_state_by_second)
            .function("get_time_to_collision", &OpenScenario::get_time_to_collision);
    }
}  // namespace esmini

#endif