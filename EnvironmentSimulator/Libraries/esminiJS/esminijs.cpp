#include "esminijs.hpp"

#include <algorithm>
#include <stdexcept>

namespace esmini
{
    namespace
    {
        RoadGeometryPoint copy_geometry_point(const roadmanager::PointStruct& point)
        {
            RoadGeometryPoint geometry_point;
            geometry_point.s        = static_cast<float>(point.s);
            geometry_point.x        = static_cast<float>(point.x);
            geometry_point.y        = static_cast<float>(point.y);
            geometry_point.z        = static_cast<float>(point.z);
            geometry_point.h        = static_cast<float>(point.h);
            geometry_point.p        = static_cast<float>(point.p);
            geometry_point.r        = static_cast<float>(point.r);
            geometry_point.endpoint = point.endpoint;
            return geometry_point;
        }

        RoadGeometryPoint copy_geometry_point(const roadmanager::Position& position)
        {
            RoadGeometryPoint geometry_point;
            geometry_point.s = static_cast<float>(position.GetS());
            geometry_point.x = static_cast<float>(position.GetX());
            geometry_point.y = static_cast<float>(position.GetY());
            geometry_point.z = static_cast<float>(position.GetZ());
            geometry_point.h = static_cast<float>(position.GetH());
            geometry_point.p = static_cast<float>(position.GetP());
            geometry_point.r = static_cast<float>(position.GetR());
            return geometry_point;
        }

        int resolve_controller_type(const scenarioengine::Object* object)
        {
            const auto long_type = object->GetControllerTypeActiveOnDomain(ControlDomains::DOMAIN_LONG);
            if (long_type != scenarioengine::Controller::Type::CONTROLLER_TYPE_UNDEFINED)
            {
                return static_cast<int>(long_type);
            }

            const auto lat_type = object->GetControllerTypeActiveOnDomain(ControlDomains::DOMAIN_LAT);
            if (lat_type != scenarioengine::Controller::Type::CONTROLLER_TYPE_UNDEFINED)
            {
                return static_cast<int>(lat_type);
            }

            return static_cast<int>(scenarioengine::Controller::Type::CONTROLLER_TYPE_UNDEFINED);
        }

        ScenarioObjectState copy_state_from_object(const scenarioengine::Object* object, double simulation_time)
        {
            ScenarioObjectState state;

            state.name            = object->GetName();
            state.id              = object->GetId();
            state.model_id        = object->model_id_;
            state.ctrl_type       = resolve_controller_type(object);
            state.timestamp       = static_cast<float>(simulation_time);
            state.x               = static_cast<float>(object->pos_.GetX());
            state.y               = static_cast<float>(object->pos_.GetY());
            state.z               = static_cast<float>(object->pos_.GetZ());
            state.h               = static_cast<float>(object->pos_.GetH());
            state.p               = static_cast<float>(object->pos_.GetP());
            state.r               = static_cast<float>(object->pos_.GetR());
            state.road_id         = static_cast<int>(object->pos_.GetTrackId());
            state.junction_id     = static_cast<int>(object->pos_.GetJunctionId());
            state.t               = static_cast<float>(object->pos_.GetT());
            state.lane_id         = static_cast<int>(object->pos_.GetLaneId());
            state.lane_offset     = static_cast<float>(object->pos_.GetOffset());
            state.s               = static_cast<float>(object->pos_.GetS());
            state.speed           = static_cast<float>(object->GetSpeed());
            state.center_offset_x = static_cast<float>(object->boundingbox_.center_.x_);
            state.center_offset_y = static_cast<float>(object->boundingbox_.center_.y_);
            state.center_offset_z = static_cast<float>(object->boundingbox_.center_.z_);
            state.width           = static_cast<float>(object->boundingbox_.dimensions_.width_);
            state.length          = static_cast<float>(object->boundingbox_.dimensions_.length_);
            state.height          = static_cast<float>(object->boundingbox_.dimensions_.height_);
            state.object_type     = static_cast<int>(object->GetType());
            state.object_category = object->category_;
            state.has_ghost       = object->ghost_ != nullptr;
            state.sensor_x        = static_cast<float>(object->sensor_pos_[0]);
            state.sensor_y        = static_cast<float>(object->sensor_pos_[1]);
            state.sensor_z        = static_cast<float>(object->sensor_pos_[2]);
            state.trail_x         = static_cast<float>(object->trail_closest_pos_.x);
            state.trail_y         = static_cast<float>(object->trail_closest_pos_.y);
            state.trail_z         = static_cast<float>(object->trail_closest_pos_.z);
            state.wheel_angle     = static_cast<float>(object->GetWheelAngle());
            state.wheel_rot       = static_cast<float>(object->GetWheelRotation());

            return state;
        }

        bool nearly_equal(double lhs, double rhs)
        {
            return std::fabs(lhs - rhs) <= SMALL_NUMBER;
        }

        std::vector<double> collect_section_samples(roadmanager::LaneSection* lane_section)
        {
            std::vector<double> samples;

            if (lane_section == nullptr)
            {
                return samples;
            }

            const double section_start = lane_section->GetS();
            const double section_end   = lane_section->GetS() + lane_section->GetLength();

            const auto& ref_points = lane_section->GetRefLineOSIPoints().GetPoints();
            samples.reserve(ref_points.size() + 2);
            for (const auto& point : ref_points)
            {
                if (point.s + SMALL_NUMBER < section_start || point.s > section_end + SMALL_NUMBER)
                {
                    continue;
                }

                samples.push_back(std::clamp(point.s, section_start, section_end));
            }

            samples.push_back(section_start);
            if (section_end - section_start > SMALL_NUMBER)
            {
                samples.push_back(section_end - SMALL_NUMBER);
            }
            else
            {
                samples.push_back(section_end);
            }

            std::sort(samples.begin(), samples.end());
            samples.erase(std::unique(samples.begin(), samples.end(), nearly_equal), samples.end());
            return samples;
        }

        bool append_lane_surface_sample(int                       road_id,
                                        int                       lane_id,
                                        roadmanager::LaneSection* lane_section,
                                        int                       lane_section_index,
                                        double                    s,
                                        LaneSurfaceGeometry&      lane_surface)
        {
            if (lane_section == nullptr)
            {
                return false;
            }

            const double lane_width = lane_section->GetWidth(s, lane_id);
            if (lane_width <= SMALL_NUMBER)
            {
                return false;
            }

            roadmanager::Position left_boundary;
            roadmanager::Position right_boundary;

            if (left_boundary.SetLanePos(road_id, lane_id, s, lane_width / 2.0, lane_section_index) != roadmanager::Position::ReturnCode::OK)
            {
                return false;
            }

            if (right_boundary.SetLanePos(road_id, lane_id, s, -lane_width / 2.0, lane_section_index) != roadmanager::Position::ReturnCode::OK)
            {
                return false;
            }

            lane_surface.left_boundary.push_back(copy_geometry_point(left_boundary));
            lane_surface.right_boundary.push_back(copy_geometry_point(right_boundary));

            return true;
        }

        std::vector<double> collect_road_samples(roadmanager::Road* road, double start_s, double end_s)
        {
            std::vector<double> samples;

            if (road == nullptr)
            {
                return samples;
            }

            const double clamped_start = std::clamp(start_s, 0.0, road->GetLength());
            const double clamped_end   = std::clamp(end_s, clamped_start, road->GetLength());

            for (unsigned int lane_section_index = 0; lane_section_index < road->GetNumberOfLaneSections(); ++lane_section_index)
            {
                roadmanager::LaneSection* lane_section = road->GetLaneSectionByIdx(static_cast<int>(lane_section_index));
                if (lane_section == nullptr)
                {
                    continue;
                }

                const auto& ref_points = lane_section->GetRefLineOSIPoints().GetPoints();
                samples.reserve(samples.size() + ref_points.size());
                for (const auto& point : ref_points)
                {
                    if (point.s + SMALL_NUMBER < clamped_start || point.s > clamped_end + SMALL_NUMBER)
                    {
                        continue;
                    }

                    samples.push_back(std::clamp(point.s, clamped_start, clamped_end));
                }
            }

            samples.push_back(clamped_start);
            if (clamped_end - clamped_start > SMALL_NUMBER)
            {
                samples.push_back(std::max(clamped_start, clamped_end - SMALL_NUMBER));
            }
            else
            {
                samples.push_back(clamped_end);
            }

            std::sort(samples.begin(), samples.end());
            samples.erase(std::unique(samples.begin(), samples.end(), nearly_equal), samples.end());
            return samples;
        }

        bool append_road_object_sample(int road_id, double s, double t, double z_offset, std::vector<RoadGeometryPoint>& points)
        {
            roadmanager::Position position;
            if (position.SetTrackPos(road_id, s, t) != roadmanager::Position::ReturnCode::OK)
            {
                return false;
            }

            RoadGeometryPoint geometry_point = copy_geometry_point(position);
            geometry_point.z += static_cast<float>(z_offset);
            points.push_back(std::move(geometry_point));
            return true;
        }

        bool is_supported_roadside_object(const roadmanager::RMObject* object)
        {
            if (object == nullptr)
            {
                return false;
            }

            const auto type = object->GetType();
            return type == roadmanager::RMObject::ObjectType::BARRIER || type == roadmanager::RMObject::ObjectType::RAILING ||
                   type == roadmanager::RMObject::ObjectType::POLE;
        }

        bool is_supported_overlay_object(const roadmanager::RMObject* object)
        {
            if (object == nullptr)
            {
                return false;
            }

            const auto type = object->GetType();
            return type != roadmanager::RMObject::ObjectType::NONE && type != roadmanager::RMObject::ObjectType::ROADMARK;
        }

        float sanitize_box_dimension(double value, double fallback)
        {
            const double resolved = value > SMALL_NUMBER ? value : fallback;
            return static_cast<float>(std::max(resolved, 0.05));
        }

        RoadFeatureBoxGeometry make_feature_box(int                          road_id,
                                                int                          id,
                                                const std::string&           name,
                                                const std::string&           type,
                                                const std::string&           kind,
                                                const roadmanager::Position& position,
                                                double                       width,
                                                double                       length,
                                                double                       height,
                                                double                       h_offset,
                                                double                       pitch,
                                                double                       roll,
                                                double                       z_offset)
        {
            RoadFeatureBoxGeometry box;
            box.road_id = road_id;
            box.id      = id;
            box.name    = name;
            box.type    = type;
            box.kind    = kind;
            box.width   = sanitize_box_dimension(width, length > SMALL_NUMBER ? length : 0.05);
            box.length  = sanitize_box_dimension(length, width > SMALL_NUMBER ? width : 0.05);
            box.height  = sanitize_box_dimension(height, 0.05);
            box.x       = static_cast<float>(position.GetX());
            box.y       = static_cast<float>(position.GetY());
            box.z       = static_cast<float>(position.GetZ() + z_offset + 0.5 * box.height);
            box.h       = static_cast<float>(position.GetH() + h_offset);
            box.p       = static_cast<float>(position.GetP() + pitch);
            box.r       = static_cast<float>(position.GetR() + roll);
            return box;
        }

        void append_feature_box_instance(roadmanager::Road*                   road,
                                         int                                  id,
                                         const std::string&                   name,
                                         const std::string&                   type,
                                         const std::string&                   kind,
                                         double                               s,
                                         double                               t,
                                         double                               z_offset,
                                         double                               width,
                                         double                               length,
                                         double                               height,
                                         double                               h_offset,
                                         double                               pitch,
                                         double                               roll,
                                         std::vector<RoadFeatureBoxGeometry>& boxes)
        {
            if (road == nullptr)
            {
                return;
            }

            roadmanager::Position position;
            if (position.SetTrackPos(static_cast<int>(road->GetId()), s, t) != roadmanager::Position::ReturnCode::OK)
            {
                return;
            }

            boxes.push_back(make_feature_box(static_cast<int>(road->GetId()),
                                             id,
                                             name,
                                             type,
                                             kind,
                                             position,
                                             width,
                                             length,
                                             height,
                                             h_offset,
                                             pitch,
                                             roll,
                                             z_offset));
        }

        void append_repeated_feature_boxes(roadmanager::Road*                   road,
                                           int                                  id,
                                           const std::string&                   name,
                                           const std::string&                   type,
                                           const std::string&                   kind,
                                           double                               start_s,
                                           double                               total_length,
                                           double                               distance,
                                           double                               t_start,
                                           double                               t_end,
                                           double                               z_offset_start,
                                           double                               z_offset_end,
                                           double                               width_start,
                                           double                               width_end,
                                           double                               length_start,
                                           double                               length_end,
                                           double                               height_start,
                                           double                               height_end,
                                           double                               h_offset,
                                           double                               pitch,
                                           double                               roll,
                                           std::vector<RoadFeatureBoxGeometry>& boxes)
        {
            if (road == nullptr || distance <= SMALL_NUMBER)
            {
                return;
            }

            const double end_s       = std::min(start_s + std::max(total_length, 0.0), road->GetLength());
            const double total_range = std::max(end_s - start_s, SMALL_NUMBER);

            for (double sample_s = start_s; sample_s <= end_s + SMALL_NUMBER; sample_s += distance)
            {
                const double clamped_s = std::clamp(sample_s, start_s, end_s);
                const double ratio     = std::clamp((clamped_s - start_s) / total_range, 0.0, 1.0);
                const double sample_t  = t_start + (t_end - t_start) * ratio;
                const double sample_z  = z_offset_start + (z_offset_end - z_offset_start) * ratio;
                const double sample_w  = width_start + (width_end - width_start) * ratio;
                const double sample_l  = length_start + (length_end - length_start) * ratio;
                const double sample_h  = height_start + (height_end - height_start) * ratio;

                append_feature_box_instance(road,
                                            id,
                                            name,
                                            type,
                                            kind,
                                            clamped_s,
                                            sample_t,
                                            sample_z,
                                            sample_w,
                                            sample_l,
                                            sample_h,
                                            h_offset,
                                            pitch,
                                            roll,
                                            boxes);

                if (clamped_s >= end_s - SMALL_NUMBER)
                {
                    break;
                }
            }
        }

        void append_linear_feature_box(roadmanager::Road*                   road,
                                       int                                  id,
                                       const std::string&                   name,
                                       const std::string&                   type,
                                       const std::string&                   kind,
                                       double                               start_s,
                                       double                               total_length,
                                       double                               t_start,
                                       double                               t_end,
                                       double                               z_offset_start,
                                       double                               z_offset_end,
                                       double                               width_start,
                                       double                               width_end,
                                       double                               length_start,
                                       double                               length_end,
                                       double                               height_start,
                                       double                               height_end,
                                       double                               h_offset,
                                       double                               pitch,
                                       double                               roll,
                                       std::vector<RoadFeatureBoxGeometry>& boxes)
        {
            if (road == nullptr)
            {
                return;
            }

            const double clamped_length = std::max(total_length, 0.0);
            const double center_s       = std::clamp(start_s + clamped_length * 0.5, 0.0, road->GetLength());
            const double sample_t       = (t_start + t_end) * 0.5;
            const double sample_z       = (z_offset_start + z_offset_end) * 0.5;
            const double sample_w       = (width_start + width_end) * 0.5;
            const double sample_l       = (length_start + length_end) * 0.5;
            const double sample_h       = (height_start + height_end) * 0.5;

            append_feature_box_instance(road,
                                        id,
                                        name,
                                        type,
                                        kind,
                                        center_s,
                                        sample_t,
                                        sample_z,
                                        sample_w,
                                        sample_l,
                                        sample_h,
                                        h_offset,
                                        pitch,
                                        roll,
                                        boxes);
        }

        void append_signal_feature_box(roadmanager::Road* road, const roadmanager::Signal* signal, std::vector<RoadFeatureBoxGeometry>& boxes)
        {
            if (road == nullptr || signal == nullptr)
            {
                return;
            }

            RoadFeatureBoxGeometry box;
            box.road_id = static_cast<int>(road->GetId());
            box.id      = signal->GetId();
            box.name    = signal->GetName();
            box.type    = signal->GetCombinedType().empty() ? signal->GetType() : signal->GetCombinedType();
            box.kind    = "signal";
            box.width   = sanitize_box_dimension(signal->GetWidth(), signal->GetDepth());
            box.length  = sanitize_box_dimension(signal->GetDepth(), signal->GetWidth());
            box.height  = sanitize_box_dimension(signal->GetHeight(), 0.05);
            box.x       = static_cast<float>(signal->GetX());
            box.y       = static_cast<float>(signal->GetY());
            box.z       = static_cast<float>(signal->GetZ() + signal->GetZOffset() + 0.5 * box.height);
            box.h       = static_cast<float>(signal->GetH() + signal->GetHOffset());
            box.p       = static_cast<float>(signal->GetPitch());
            box.r       = static_cast<float>(signal->GetRoll());
            boxes.push_back(std::move(box));
        }

        void append_linear_road_object_samples(roadmanager::Road*              road,
                                               double                          start_s,
                                               double                          length,
                                               double                          t_start,
                                               double                          t_end,
                                               double                          z_offset_start,
                                               double                          z_offset_end,
                                               std::vector<RoadGeometryPoint>& points)
        {
            if (road == nullptr || length <= SMALL_NUMBER)
            {
                return;
            }

            const double end_s       = std::min(start_s + length, road->GetLength());
            const double total_range = std::max(end_s - start_s, SMALL_NUMBER);
            const auto   samples     = collect_road_samples(road, start_s, end_s);

            for (double sample_s : samples)
            {
                const double ratio    = std::clamp((sample_s - start_s) / total_range, 0.0, 1.0);
                const double sample_t = t_start + (t_end - t_start) * ratio;
                const double sample_z = z_offset_start + (z_offset_end - z_offset_start) * ratio;
                append_road_object_sample(static_cast<int>(road->GetId()), sample_s, sample_t, sample_z, points);
            }
        }

        void append_repeated_road_object_instances(roadmanager::Road*              road,
                                                   double                          start_s,
                                                   double                          length,
                                                   double                          distance,
                                                   double                          t_start,
                                                   double                          t_end,
                                                   double                          z_offset_start,
                                                   double                          z_offset_end,
                                                   std::vector<RoadGeometryPoint>& points)
        {
            if (road == nullptr || length < 0.0 || distance <= SMALL_NUMBER)
            {
                return;
            }

            const double end_s       = std::min(start_s + length, road->GetLength());
            const double total_range = std::max(end_s - start_s, SMALL_NUMBER);

            for (double sample_s = start_s; sample_s <= end_s + SMALL_NUMBER; sample_s += distance)
            {
                const double clamped_s = std::clamp(sample_s, start_s, end_s);
                const double ratio     = std::clamp((clamped_s - start_s) / total_range, 0.0, 1.0);
                const double sample_t  = t_start + (t_end - t_start) * ratio;
                const double sample_z  = z_offset_start + (z_offset_end - z_offset_start) * ratio;
                append_road_object_sample(static_cast<int>(road->GetId()), clamped_s, sample_t, sample_z, points);

                if (clamped_s >= end_s - SMALL_NUMBER)
                {
                    break;
                }
            }
        }
    }  // namespace

    OpenScenario::OpenScenario(const std::string& xosc_file, const OpenScenarioConfig& config) : xosc_file_(xosc_file), config_(config)
    {
        reset();
    }

    OpenScenario::~OpenScenario()
    {
        delete scenario_engine_;
    }

    void OpenScenario::reset()
    {
        delete scenario_engine_;
        scenario_engine_ = new scenarioengine::ScenarioEngine(xosc_file_, false);
        time_stamp_      = 0;
        DirtyBits::SetReadFront();
        has_road_geometry_cache_ = false;
        road_geometry_cache_     = ScenarioRoadGeometry{};

        if (scenario_engine_->GetInitStatus() != 0)
        {
            throw std::runtime_error("Failed to initialize scenario: " + xosc_file_);
        }
    }

    double OpenScenario::resolve_time_step(double requested_time_step)
    {
        if (requested_time_step > 0.0)
        {
            return requested_time_step;
        }

        if (config_.dt > 0.0)
        {
            return config_.dt;
        }

        return SE_getSimTimeStep(time_stamp_, config_.min_time_step, config_.max_time_step);
    }

    std::vector<ScenarioObjectState> OpenScenario::collect_object_states() const
    {
        std::vector<ScenarioObjectState> object_states;
        object_states.reserve(scenario_engine_->entities_.object_.size());

        const double simulation_time = scenario_engine_->getSimulationTime();
        for (const auto* object : scenario_engine_->entities_.object_)
        {
            if (object == nullptr)
            {
                continue;
            }

            object_states.push_back(copy_state_from_object(object, simulation_time));
        }

        return object_states;
    }

    ScenarioFrame OpenScenario::make_frame(int status, double time_step) const
    {
        ScenarioFrame frame;
        frame.simulation_time = scenario_engine_->getSimulationTime();
        frame.time_step       = time_step;
        frame.status          = status;
        frame.quit            = scenario_engine_->GetQuitFlag();
        frame.object_states   = collect_object_states();
        return frame;
    }

    int OpenScenario::step(double dt)
    {
        DirtyBits::SetReadFront();
        const double time_step = resolve_time_step(dt);
        const int    status    = scenario_engine_->step(time_step);

        if (status >= 0)
        {
            scenario_engine_->prepareGroundTruth(time_step);
            scenario_engine_->SwapAndClearDirtyBits();
            DirtyBits::SetReadBack();
        }

        return status;
    }

    ScenarioFrame OpenScenario::step_frame(double dt)
    {
        DirtyBits::SetReadFront();
        const double time_step = resolve_time_step(dt);
        const int    status    = scenario_engine_->step(time_step);

        if (status >= 0)
        {
            scenario_engine_->prepareGroundTruth(time_step);
            scenario_engine_->SwapAndClearDirtyBits();
            DirtyBits::SetReadBack();
        }

        return make_frame(status, time_step);
    }

    int OpenScenario::get_object_count() const
    {
        return static_cast<int>(scenario_engine_->entities_.object_.size());
    }

    double OpenScenario::get_simulation_time() const
    {
        return scenario_engine_->getSimulationTime();
    }

    bool OpenScenario::is_quit() const
    {
        return scenario_engine_->GetQuitFlag();
    }

    std::vector<ScenarioObjectState> OpenScenario::get_object_states() const
    {
        return collect_object_states();
    }

    ScenarioObjectState OpenScenario::get_object_state_by_index(int index) const
    {
        if (index < 0 || index >= get_object_count())
        {
            throw std::out_of_range("Object index out of range");
        }

        const auto* object = scenario_engine_->entities_.object_[static_cast<size_t>(index)];
        return copy_state_from_object(object, scenario_engine_->getSimulationTime());
    }

    double OpenScenario::get_time_to_collision(int object_a_id, int object_b_id, bool free_space, int cs, int dist_type) const
    {
        if (scenario_engine_ == nullptr)
        {
            return -1.0;
        }

        scenarioengine::Object* obj_a = scenario_engine_->entities_.GetObjectById(object_a_id);
        scenarioengine::Object* obj_b = scenario_engine_->entities_.GetObjectById(object_b_id);
        if (obj_a == nullptr || obj_b == nullptr)
        {
            return -1.0;
        }

        double ttc = -1.0;
        if (obj_a->TimeToCollision(obj_b,
                                   static_cast<roadmanager::CoordinateSystem>(cs),
                                   static_cast<roadmanager::RelativeDistanceType>(dist_type),
                                   free_space,
                                   ttc) != 0)
        {
            return -1.0;
        }
        return ttc;
    }

    ScenarioRoadGeometry OpenScenario::collect_road_geometry() const
    {
        ScenarioRoadGeometry geometry;

        if (scenario_engine_ == nullptr || scenario_engine_->getRoadManager() == nullptr)
        {
            return geometry;
        }

        roadmanager::OpenDrive* open_drive = scenario_engine_->getRoadManager();
        geometry.odr_filename              = open_drive->GetOpenDriveFilename();

        for (unsigned int road_index = 0; road_index < open_drive->GetNumOfRoads(); ++road_index)
        {
            roadmanager::Road* road = open_drive->GetRoadByIdx(static_cast<int>(road_index));
            if (road == nullptr)
            {
                continue;
            }

            for (unsigned int lane_section_index = 0; lane_section_index < road->GetNumberOfLaneSections(); ++lane_section_index)
            {
                roadmanager::LaneSection* lane_section = road->GetLaneSectionByIdx(static_cast<int>(lane_section_index));
                if (lane_section == nullptr)
                {
                    continue;
                }

                const std::vector<double> section_samples = collect_section_samples(lane_section);
                if (section_samples.empty())
                {
                    continue;
                }

                const auto& ref_line_points = lane_section->GetRefLineOSIPoints().GetPoints();
                if (ref_line_points.size() >= 2)
                {
                    LaneCenterGeometry reference_line_geometry;
                    reference_line_geometry.road_id            = static_cast<int>(road->GetId());
                    reference_line_geometry.lane_section_index = static_cast<int>(lane_section_index);
                    reference_line_geometry.lane_id            = 0;
                    reference_line_geometry.reference_line     = true;
                    reference_line_geometry.center_lane        = false;
                    reference_line_geometry.points.reserve(ref_line_points.size());

                    for (const auto& point : ref_line_points)
                    {
                        reference_line_geometry.points.push_back(copy_geometry_point(point));
                    }

                    geometry.lane_centers.push_back(std::move(reference_line_geometry));
                }

                for (unsigned int lane_index = 0; lane_index < lane_section->GetNumberOfLanes(); ++lane_index)
                {
                    roadmanager::Lane* lane = lane_section->GetLaneByIdx(static_cast<int>(lane_index));
                    if (lane == nullptr || lane->GetId() == 0)
                    {
                        continue;
                    }

                    LaneSurfaceGeometry lane_surface;
                    lane_surface.road_id            = static_cast<int>(road->GetId());
                    lane_surface.lane_section_index = static_cast<int>(lane_section_index);
                    lane_surface.lane_id            = lane->GetId();
                    lane_surface.lane_type          = static_cast<int>(lane->GetLaneType());
                    lane_surface.road_edge          = lane->IsRoadEdge();
                    lane_surface.left_boundary.reserve(section_samples.size());
                    lane_surface.right_boundary.reserve(section_samples.size());

                    for (double sample_s : section_samples)
                    {
                        append_lane_surface_sample(static_cast<int>(road->GetId()),
                                                   lane->GetId(),
                                                   lane_section,
                                                   static_cast<int>(lane_section_index),
                                                   sample_s,
                                                   lane_surface);
                    }

                    if (lane_surface.left_boundary.size() >= 2 && lane_surface.left_boundary.size() == lane_surface.right_boundary.size())
                    {
                        geometry.lane_surfaces.push_back(std::move(lane_surface));
                    }

                    if (lane->IsDriving() && lane->GetOSIPoints() != nullptr)
                    {
                        const auto& lane_center_points = lane->GetOSIPoints()->GetPoints();
                        if (lane_center_points.size() >= 2)
                        {
                            LaneCenterGeometry lane_center_geometry;
                            lane_center_geometry.road_id            = static_cast<int>(road->GetId());
                            lane_center_geometry.lane_section_index = static_cast<int>(lane_section_index);
                            lane_center_geometry.lane_id            = lane->GetId();
                            lane_center_geometry.reference_line     = false;
                            lane_center_geometry.center_lane        = false;
                            lane_center_geometry.points.reserve(lane_center_points.size());

                            for (const auto& point : lane_center_points)
                            {
                                lane_center_geometry.points.push_back(copy_geometry_point(point));
                            }

                            geometry.lane_centers.push_back(std::move(lane_center_geometry));
                        }
                    }

                    for (unsigned int road_mark_index = 0; road_mark_index < lane->GetNumberOfRoadMarks(); ++road_mark_index)
                    {
                        roadmanager::LaneRoadMark* lane_road_mark = lane->GetLaneRoadMarkByIdx(static_cast<int>(road_mark_index));
                        if (lane_road_mark == nullptr)
                        {
                            continue;
                        }

                        for (unsigned int road_mark_type_index = 0; road_mark_type_index < lane_road_mark->GetNumberOfRoadMarkTypes();
                             ++road_mark_type_index)
                        {
                            roadmanager::LaneRoadMarkType* road_mark_type = lane_road_mark->GetLaneRoadMarkTypeByIdx(road_mark_type_index);
                            if (road_mark_type == nullptr)
                            {
                                continue;
                            }

                            for (unsigned int road_mark_line_index = 0; road_mark_line_index < road_mark_type->GetNumberOfRoadMarkTypeLines();
                                 ++road_mark_line_index)
                            {
                                roadmanager::LaneRoadMarkTypeLine* road_mark_line =
                                    road_mark_type->GetLaneRoadMarkTypeLineByIdx(road_mark_line_index);
                                if (road_mark_line == nullptr)
                                {
                                    continue;
                                }

                                const auto& points = road_mark_line->GetOSIPoints()->GetPoints();
                                if (points.size() < 2)
                                {
                                    continue;
                                }

                                RoadMarkGeometry road_mark_geometry;
                                road_mark_geometry.road_id            = static_cast<int>(road->GetId());
                                road_mark_geometry.lane_section_index = static_cast<int>(lane_section_index);
                                road_mark_geometry.lane_id            = lane->GetId();
                                road_mark_geometry.type               = lane_road_mark->Type2Str();

                                roadmanager::RoadMarkColor color = road_mark_line->GetColor();
                                if (color == roadmanager::RoadMarkColor::UNDEFINED)
                                {
                                    color = lane_road_mark->GetColor();
                                }

                                road_mark_geometry.color = roadmanager::LaneRoadMark::RoadMarkColor2Str(color);
                                road_mark_geometry.width = static_cast<float>(road_mark_line->GetWidth() > SMALL_NUMBER ? road_mark_line->GetWidth()
                                                                                                                        : lane_road_mark->GetWidth());
                                road_mark_geometry.line_length = static_cast<float>(road_mark_line->GetLength());
                                road_mark_geometry.line_space  = static_cast<float>(road_mark_line->GetSpace());
                                road_mark_geometry.points.reserve(points.size());

                                for (const auto& point : points)
                                {
                                    road_mark_geometry.points.push_back(copy_geometry_point(point));
                                }

                                geometry.road_marks.push_back(std::move(road_mark_geometry));
                            }
                        }
                    }
                }
            }

            for (unsigned int signal_index = 0; signal_index < road->GetNumberOfSignals(); ++signal_index)
            {
                append_signal_feature_box(road, road->GetSignal(signal_index), geometry.road_feature_boxes);
            }

            for (unsigned int object_index = 0; object_index < road->GetNumberOfObjects(); ++object_index)
            {
                roadmanager::RMObject* object = road->GetRoadObject(object_index);
                if (is_supported_overlay_object(object))
                {
                    if (object->GetNumberOfRepeats() > 0)
                    {
                        for (unsigned int repeat_index = 0; repeat_index < object->GetNumberOfRepeats(); ++repeat_index)
                        {
                            roadmanager::Repeat* repeat = object->GetRepeatByIdx(repeat_index);
                            if (repeat == nullptr)
                            {
                                continue;
                            }

                            const double width_start  = repeat->GetWidthStart() > SMALL_NUMBER ? repeat->GetWidthStart() : object->GetWidth();
                            const double width_end    = repeat->GetWidthEnd() > SMALL_NUMBER ? repeat->GetWidthEnd() : width_start;
                            const double length_start = repeat->GetLengthStart() > SMALL_NUMBER
                                                            ? repeat->GetLengthStart()
                                                            : (object->GetLength() > SMALL_NUMBER ? object->GetLength() : width_start);
                            const double length_end   = repeat->GetLengthEnd() > SMALL_NUMBER ? repeat->GetLengthEnd() : length_start;
                            const double height_start = repeat->GetHeightStart() > SMALL_NUMBER ? repeat->GetHeightStart() : object->GetHeight();
                            const double height_end   = repeat->GetHeightEnd() > SMALL_NUMBER ? repeat->GetHeightEnd() : height_start;

                            if (repeat->GetDistance() > SMALL_NUMBER)
                            {
                                append_repeated_feature_boxes(road,
                                                              static_cast<int>(object->GetId()),
                                                              object->GetName(),
                                                              object->GetTypeStr(),
                                                              "object",
                                                              repeat->GetS(),
                                                              repeat->GetLength(),
                                                              repeat->GetDistance(),
                                                              repeat->GetTStart(),
                                                              repeat->GetTEnd(),
                                                              repeat->GetZOffsetStart(),
                                                              repeat->GetZOffsetEnd(),
                                                              width_start,
                                                              width_end,
                                                              length_start,
                                                              length_end,
                                                              height_start,
                                                              height_end,
                                                              object->GetHOffset(),
                                                              object->GetPitch(),
                                                              object->GetRoll(),
                                                              geometry.road_feature_boxes);
                            }
                            else
                            {
                                append_linear_feature_box(road,
                                                          static_cast<int>(object->GetId()),
                                                          object->GetName(),
                                                          object->GetTypeStr(),
                                                          "object",
                                                          repeat->GetS(),
                                                          repeat->GetLength(),
                                                          repeat->GetTStart(),
                                                          repeat->GetTEnd(),
                                                          repeat->GetZOffsetStart(),
                                                          repeat->GetZOffsetEnd(),
                                                          width_start,
                                                          width_end,
                                                          length_start,
                                                          length_end,
                                                          height_start,
                                                          height_end,
                                                          object->GetHOffset(),
                                                          object->GetPitch(),
                                                          object->GetRoll(),
                                                          geometry.road_feature_boxes);
                            }
                        }
                    }
                    else
                    {
                        const double object_length = object->GetLength() > SMALL_NUMBER ? object->GetLength() : object->GetWidth();
                        append_linear_feature_box(road,
                                                  static_cast<int>(object->GetId()),
                                                  object->GetName(),
                                                  object->GetTypeStr(),
                                                  "object",
                                                  object->GetS(),
                                                  object_length,
                                                  object->GetT(),
                                                  object->GetT(),
                                                  object->GetZOffset(),
                                                  object->GetZOffset(),
                                                  object->GetWidth(),
                                                  object->GetWidth(),
                                                  object_length,
                                                  object_length,
                                                  object->GetHeight(),
                                                  object->GetHeight(),
                                                  object->GetHOffset(),
                                                  object->GetPitch(),
                                                  object->GetRoll(),
                                                  geometry.road_feature_boxes);
                    }
                }

                if (!is_supported_roadside_object(object))
                {
                    continue;
                }

                if (object->GetNumberOfRepeats() > 0)
                {
                    for (unsigned int repeat_index = 0; repeat_index < object->GetNumberOfRepeats(); ++repeat_index)
                    {
                        roadmanager::Repeat* repeat = object->GetRepeatByIdx(repeat_index);
                        if (repeat == nullptr || repeat->GetLength() <= SMALL_NUMBER)
                        {
                            continue;
                        }

                        RoadObjectGeometry road_object_geometry;
                        road_object_geometry.road_id = static_cast<int>(road->GetId());
                        road_object_geometry.id      = static_cast<int>(object->GetId());
                        road_object_geometry.name    = object->GetName();
                        road_object_geometry.type    = object->GetTypeStr();
                        road_object_geometry.width_start =
                            static_cast<float>(repeat->GetWidthStart() > SMALL_NUMBER ? repeat->GetWidthStart() : object->GetWidth());
                        road_object_geometry.width_end =
                            static_cast<float>(repeat->GetWidthEnd() > SMALL_NUMBER ? repeat->GetWidthEnd() : road_object_geometry.width_start);
                        road_object_geometry.height_start =
                            static_cast<float>(repeat->GetHeightStart() > SMALL_NUMBER ? repeat->GetHeightStart() : object->GetHeight());
                        road_object_geometry.height_end =
                            static_cast<float>(repeat->GetHeightEnd() > SMALL_NUMBER ? repeat->GetHeightEnd() : road_object_geometry.height_start);

                        if (repeat->GetDistance() > SMALL_NUMBER)
                        {
                            append_repeated_road_object_instances(road,
                                                                  repeat->GetS(),
                                                                  repeat->GetLength(),
                                                                  repeat->GetDistance(),
                                                                  repeat->GetTStart(),
                                                                  repeat->GetTEnd(),
                                                                  repeat->GetZOffsetStart(),
                                                                  repeat->GetZOffsetEnd(),
                                                                  road_object_geometry.points);
                        }
                        else
                        {
                            append_linear_road_object_samples(road,
                                                              repeat->GetS(),
                                                              repeat->GetLength(),
                                                              repeat->GetTStart(),
                                                              repeat->GetTEnd(),
                                                              repeat->GetZOffsetStart(),
                                                              repeat->GetZOffsetEnd(),
                                                              road_object_geometry.points);
                        }

                        if ((repeat->GetDistance() > SMALL_NUMBER && !road_object_geometry.points.empty()) || road_object_geometry.points.size() >= 2)
                        {
                            geometry.road_objects.push_back(std::move(road_object_geometry));
                        }
                    }

                    continue;
                }

                if (object->GetLength() <= SMALL_NUMBER)
                {
                    continue;
                }

                RoadObjectGeometry road_object_geometry;
                road_object_geometry.road_id      = static_cast<int>(road->GetId());
                road_object_geometry.id           = static_cast<int>(object->GetId());
                road_object_geometry.name         = object->GetName();
                road_object_geometry.type         = object->GetTypeStr();
                road_object_geometry.width_start  = static_cast<float>(object->GetWidth());
                road_object_geometry.width_end    = static_cast<float>(object->GetWidth());
                road_object_geometry.height_start = static_cast<float>(object->GetHeight());
                road_object_geometry.height_end   = static_cast<float>(object->GetHeight());

                append_linear_road_object_samples(road,
                                                  object->GetS(),
                                                  object->GetLength(),
                                                  object->GetT(),
                                                  object->GetT(),
                                                  object->GetZOffset(),
                                                  object->GetZOffset(),
                                                  road_object_geometry.points);

                if (road_object_geometry.points.size() >= 2)
                {
                    geometry.road_objects.push_back(std::move(road_object_geometry));
                }
            }
        }

        return geometry;
    }

    ScenarioRoadGeometry OpenScenario::get_road_geometry()
    {
        if (!has_road_geometry_cache_)
        {
            road_geometry_cache_     = collect_road_geometry();
            has_road_geometry_cache_ = true;
        }

        return road_geometry_cache_;
    }

    std::vector<ScenarioObjectState> OpenScenario::get_object_state(const OpenScenarioConfig* config)
    {
        OpenScenarioConfig effective_config = config_;
        if (config != nullptr)
        {
            effective_config = *config;
        }

        std::vector<ScenarioObjectState> objects_sts;
        int                              status           = 0;
        __int64                          local_time_stamp = 0;

        while (status == 0 && effective_config.max_loop > 0)
        {
            DirtyBits::SetReadFront();
            double dt = effective_config.dt;
            if (dt <= 0.0)
            {
                dt = SE_getSimTimeStep(local_time_stamp, effective_config.min_time_step, effective_config.max_time_step);
            }

            status = scenario_engine_->step(dt);
            if (status >= 0)
            {
                scenario_engine_->prepareGroundTruth(dt);
                scenario_engine_->SwapAndClearDirtyBits();
                DirtyBits::SetReadBack();
                auto frame_states = collect_object_states();
                objects_sts.insert(objects_sts.end(), frame_states.begin(), frame_states.end());
            }

            --effective_config.max_loop;
        }

        return objects_sts;
    }

    std::vector<ScenarioObjectState> OpenScenario::get_object_state_by_second(int second, int fps)
    {
        OpenScenarioConfig config = config_;
        config.max_loop           = second * fps;
        config.dt                 = 1.0 / static_cast<double>(fps);

        return get_object_state(&config);
    }

}  // namespace esmini
