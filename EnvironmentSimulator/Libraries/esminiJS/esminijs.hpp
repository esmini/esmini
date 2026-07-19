#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "ScenarioEngine.hpp"

namespace esmini
{
    struct OpenScenarioConfig
    {
        int    max_loop      = 10000;
        double min_time_step = 1.0 / 120.0;
        double max_time_step = 1.0 / 25.0;
        double dt            = 0.0;

        friend std::ostream& operator<<(std::ostream& output, const OpenScenarioConfig& config)
        {
            output << "max_loop:" << config.max_loop << " min_time_step:" << config.min_time_step << ",max_time_step:" << config.max_time_step
                   << ",dt:" << config.dt;
            return output;
        }
    };

    struct ScenarioObjectState
    {
        std::string name;
        int         id;
        int         model_id;
        int         ctrl_type;
        float       timestamp;
        float       x;
        float       y;
        float       z;
        float       h;
        float       p;
        float       r;
        int         road_id;
        int         junction_id;
        float       t;
        int         lane_id;
        float       lane_offset;
        float       s;
        float       speed;
        float       center_offset_x;
        float       center_offset_y;
        float       center_offset_z;
        float       width;
        float       length;
        float       height;
        int         object_type;
        int         object_category;
        bool        has_ghost;
        float       sensor_x;
        float       sensor_y;
        float       sensor_z;
        float       trail_x;
        float       trail_y;
        float       trail_z;
        float       wheel_angle;
        float       wheel_rot;

        friend std::ostream& operator<<(std::ostream& output, const ScenarioObjectState& state)
        {
            output << "name:" << state.name << "time:" << state.timestamp << ",id:" << state.id << ",model_id:" << state.model_id
                   << ",ctrl_type:" << state.ctrl_type << ",pos.x:" << std::setprecision(3) << state.x << ",pos.y:" << std::setprecision(3) << state.y
                   << ",pos.z:" << std::setprecision(3) << state.z << ",speed:" << state.speed << ",type:" << state.object_type
                   << ",category:" << state.object_category << ",width:" << state.width << ",height:" << state.height << ",length:" << state.length
                   << ",has_ghost:" << state.has_ghost << ",sensor_x:" << state.sensor_x << ",sensor_y:" << state.sensor_y
                   << ",sensor_z:" << state.sensor_z << ",trail_x:" << state.trail_x << ",trail_y:" << state.trail_y << ",trail_z:" << state.trail_z;
            return output;
        }
    };

    struct ScenarioFrame
    {
        double                           simulation_time = 0.0;
        double                           time_step       = 0.0;
        int                              status          = 0;
        bool                             quit            = false;
        std::vector<ScenarioObjectState> object_states;
    };

    struct RoadGeometryPoint
    {
        float s        = 0.0f;
        float x        = 0.0f;
        float y        = 0.0f;
        float z        = 0.0f;
        float h        = 0.0f;
        float p        = 0.0f;
        float r        = 0.0f;
        bool  endpoint = false;
    };

    struct LaneSurfaceGeometry
    {
        int                            road_id            = 0;
        int                            lane_section_index = 0;
        int                            lane_id            = 0;
        int                            lane_type          = 0;
        bool                           road_edge          = false;
        std::vector<RoadGeometryPoint> left_boundary;
        std::vector<RoadGeometryPoint> right_boundary;
    };

    struct RoadMarkGeometry
    {
        int                            road_id            = 0;
        int                            lane_section_index = 0;
        int                            lane_id            = 0;
        std::string                    type;
        std::string                    color;
        float                          width       = 0.0f;
        float                          line_length = 0.0f;
        float                          line_space  = 0.0f;
        std::vector<RoadGeometryPoint> points;
    };

    struct LaneCenterGeometry
    {
        int                            road_id            = 0;
        int                            lane_section_index = 0;
        int                            lane_id            = 0;
        bool                           reference_line     = false;
        bool                           center_lane        = false;
        std::vector<RoadGeometryPoint> points;
    };

    struct RoadObjectGeometry
    {
        int                            road_id = 0;
        int                            id      = 0;
        std::string                    name;
        std::string                    type;
        float                          width_start  = 0.0f;
        float                          width_end    = 0.0f;
        float                          height_start = 0.0f;
        float                          height_end   = 0.0f;
        std::vector<RoadGeometryPoint> points;
    };

    struct RoadFeatureBoxGeometry
    {
        int         road_id = 0;
        int         id      = 0;
        std::string name;
        std::string type;
        std::string kind;
        float       x      = 0.0f;
        float       y      = 0.0f;
        float       z      = 0.0f;
        float       h      = 0.0f;
        float       p      = 0.0f;
        float       r      = 0.0f;
        float       width  = 0.0f;
        float       length = 0.0f;
        float       height = 0.0f;
    };

    struct ScenarioRoadGeometry
    {
        std::string                         odr_filename;
        std::vector<LaneSurfaceGeometry>    lane_surfaces;
        std::vector<RoadMarkGeometry>       road_marks;
        std::vector<LaneCenterGeometry>     lane_centers;
        std::vector<RoadObjectGeometry>     road_objects;
        std::vector<RoadFeatureBoxGeometry> road_feature_boxes;
    };

    class OpenScenario
    {
    public:
        OpenScenario(const std::string& xosc_file, const OpenScenarioConfig& config = OpenScenarioConfig{});
        ~OpenScenario();

        std::vector<ScenarioObjectState> get_object_state(const OpenScenarioConfig* config = nullptr);
        std::vector<ScenarioObjectState> get_object_state_by_second(int second, int fps = 30);
        std::vector<ScenarioObjectState> get_object_states() const;
        ScenarioObjectState              get_object_state_by_index(int index) const;
        ScenarioRoadGeometry             get_road_geometry();
        ScenarioFrame                    step_frame(double dt = -1.0);
        int                              step(double dt = -1.0);
        int                              get_object_count() const;
        double                           get_simulation_time() const;
        bool                             is_quit() const;
        void                             reset();

        double get_time_to_collision(int object_a_id, int object_b_id, bool free_space = true, int cs = 1, int dist_type = 2) const;

        OpenScenario(const OpenScenario&)            = delete;
        OpenScenario& operator=(const OpenScenario&) = delete;

    private:
        std::vector<ScenarioObjectState> collect_object_states() const;
        ScenarioRoadGeometry             collect_road_geometry() const;
        ScenarioFrame                    make_frame(int status, double time_step) const;
        double                           resolve_time_step(double requested_time_step);

        std::string                     xosc_file_;
        OpenScenarioConfig              config_;
        scenarioengine::ScenarioEngine* scenario_engine_ = nullptr;
        __int64                         time_stamp_      = 0;
        ScenarioRoadGeometry            road_geometry_cache_;
        bool                            has_road_geometry_cache_ = false;
    };
}  // namespace esmini
