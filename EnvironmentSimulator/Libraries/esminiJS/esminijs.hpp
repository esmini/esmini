#include <string>
#include <iostream>
#include <iomanip>
#include <vector>

#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "ScenarioEngine.hpp"

// make a class for  js interface
namespace esmini
{
    struct OpenScenarioConfig
    {
        int max_loop = 10e3;              // 最大循环次数
        double min_time_step = 1.0 / 120; // 最小的时间步长
        double max_time_step = 1.0 / 25;  // 最大的时间步长
        double dt = 0;                    // 时间步长

        friend std::ostream &operator<<(std::ostream &output,
                                        const OpenScenarioConfig &config)
        {
            output << "max_loop:" << config.max_loop
                   << " min_time_step:" << config.min_time_step
                   << ",max_time_step:" << config.max_time_step
                   << ",dt:" << config.dt;
            return output;
        }
    };

    struct ScenarioObjectState
    {
        std::string name;      // name xosc define
        int id;                // Automatically generated unique object id
        int model_id;          // Id to control what 3D model to represent the vehicle - see carModelsFiles_[] in scenarioenginedll.cpp
        int ctrl_type;         // 0: DefaultController 1: External. Further values see Controller::Type enum
        float timestamp;       // Not used yet (idea is to use it to interpolate position for increased sync bewtween simulators)
        float x;               // global x coordinate of position
        float y;               // global y coordinate of position
        float z;               // global z coordinate of position
        float h;               // heading/yaw in global coordinate system
        float p;               // pitch in global coordinate system
        float r;               // roll in global coordinate system
        int road_id;           // road ID
        int junction_id;       // Junction ID (-1 if not in a junction)
        float t;               // lateral position in road coordinate system
        int lane_id;           // lane ID
        float lane_offset;     // lateral offset from lane center
        float s;               // longitudinal position in road coordinate system
        float speed;           // speed
        float center_offset_x; // x coordinate of bounding box center relative object reference point (local coordinate system)
        float center_offset_y; // y coordinate of bounding box center relative object reference point (local coordinate system)
        float center_offset_z; // z coordinate of bounding box center relative object reference point (local coordinate system)
        float width;           // width
        float length;          // length
        float height;          // height
        int object_type;       // Main type according to entities.hpp / Object / Type
        int object_category;   // Sub category within type, according to entities.hpp / Vehicle, Pedestrian, MiscObject / Category
        float wheel_angle;     // Steering angle of the wheel
        float wheel_rot;       // Rotation angle of the wheel

        friend std::ostream &operator<<(std::ostream &output,
                                        const ScenarioObjectState &state)
        {
            output << "name:" << state.name
                   << "time:" << state.timestamp
                   << ",id:" << state.id
                   << ",model_id:" << state.model_id
                   << ",ctrl_type:" << state.ctrl_type
                   << ",pos.x:" << std::setprecision(3) << state.x << ",pos.y:" << std::setprecision(3) << state.y << ",pos.z:" << std::setprecision(3) << state.z
                   << ",speed:" << state.speed
                   << ",type:" << state.object_type
                   << ",category:" << state.object_category
                   << ",width:" << state.width
                   << ",height:" << state.height
                   << ",length:" << state.length;
            return output;
        }
    };

    class OpenScenario
    {
    public:
        OpenScenario(const std::string &xosc_file, const OpenScenarioConfig &config = OpenScenarioConfig{});
        // get object state use config
        std::vector<ScenarioObjectState> get_object_state(OpenScenarioConfig *config = nullptr);
        // get object state use second and frame
        std::vector<ScenarioObjectState> get_object_state_by_second(const int second, const int fps = 30);

    private:
        std::string xosc_file;
        OpenScenarioConfig config;
        scenarioengine::ScenarioEngine *scenarioEngine;
        scenarioengine::ScenarioGateway *scenarioGateway;
    };
}
