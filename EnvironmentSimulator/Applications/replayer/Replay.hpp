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
#include <fstream>
#include <variant>
#include <set>
#include <optional>
#include "CommonMini.hpp"
#include "ScenarioGateway.hpp"

namespace scenarioengine
{
    // Reworked data structures to hold properties over time
    template <typename T>
    struct Timeline
    {
        std::vector<std::pair<float, T>> values;          // Pairs of timestamp and value
        mutable size_t                   last_index = 0;  // Set as mutable to allow modification in const methods

        const T& get_value_incremental(float time) const noexcept
        {
            if (values.empty())
            {
                LOG_ERROR_AND_QUIT("Timeline is empty, cannot get value at time {}", time);
            }

            size_t idx        = last_index;
            float  moved_dt   = 0.0f;
            float  desired_dt = time - values[last_index].first;

            if (time >= values[idx].first)  // Requested time is after last searched value, we increment
            {
                while (idx + 1 < values.size())
                {
                    float step = values[idx + 1].first - values[idx].first;
                    if (moved_dt + step > desired_dt + SMALL_NUMBERF)
                        break;

                    moved_dt += step;
                    idx++;
                }
            }
            else  // Requested time is before last searched value, we decrement
            {
                while (idx > 0)
                {
                    float step = values[idx].first - values[idx - 1].first;
                    if (moved_dt + step > -desired_dt - SMALL_NUMBERF)
                    {
                        idx--;
                        break;
                    }

                    moved_dt += step;
                    idx--;
                }
            }

            last_index = idx;  // Save the index for next call

            return values[idx].second;
        }

        const T& get_value_binary(float time) const noexcept
        {
            if (values.empty())
            {
                LOG_ERROR_AND_QUIT("Timeline is empty, cannot get value at time {}", time);
            }

            auto search_begin = values.begin();
            auto search_end   = values.end();

            if (time >= values[last_index].first)
            {
                // Time moved forward — only search ahead
                search_begin = values.begin() + static_cast<typename std::vector<std::pair<float, T>>::difference_type>(last_index);
            }
            else
            {
                // Time moved backward — only search behind
                search_end = values.begin() + static_cast<typename std::vector<std::pair<float, T>>::difference_type>(last_index) + 1;
            }

            auto it = std::upper_bound(search_begin, search_end, time, [](float t, const std::pair<float, T>& v) { return t < v.first; });

            if (it == values.begin())
            {
                last_index = 0;
                return it->second;
            }

            --it;
            last_index = static_cast<size_t>(std::distance(values.begin(), it));
            return it->second;
        }

        size_t get_index_binary(float time) const noexcept
        {
            if (values.empty())
            {
                LOG_ERROR_AND_QUIT("Timeline is empty, cannot get value at time {}", time);
            }

            auto search_begin = values.begin();
            auto search_end   = values.end();

            if (time >= values[last_index].first)
            {
                // Time moved forward — only search ahead
                search_begin = values.begin();
            }
            else
            {
                // Time moved backward — only search behind
                search_end = values.begin() + 1;
            }

            auto it = std::upper_bound(search_begin, search_end, time, [](float t, const std::pair<float, T>& v) { return t < v.first; });

            if (it == values.begin())
            {
                return 0;
            }

            return static_cast<size_t>(std::distance(values.begin(), it));
        }
    };

    struct PropertyTimeline
    {
        Timeline<int>            model_id_;
        Timeline<int>            obj_type_;
        Timeline<int>            obj_category_;
        Timeline<int>            ctrl_type_;
        Timeline<std::string>    name_;
        Timeline<float>          speed_;
        Timeline<float>          wheel_angle_;
        Timeline<float>          wheel_rot_;
        Timeline<OSCBoundingBox> bounding_box_;
        Timeline<int>            scale_mode_;
        Timeline<int>            visibility_mask_;
        Timeline<Dat::Pose>      pose_;
        Timeline<id_t>           road_id_;
        Timeline<int>            lane_id_;
        Timeline<float>          pos_offset_;
        Timeline<float>          pos_t_;
        Timeline<float>          pos_s_;
        Timeline<bool>           active_;
        Timeline<float>          odometer_;

        float last_restart_time = -1.0f;
    };

    struct MapComparator
    {
        bool operator()(int lhs, int rhs) const
        {
            if (lhs >= 0 && rhs < 0)
                return true;
            if (lhs < 0 && rhs >= 0)
                return false;
            if (lhs >= 0 && rhs >= 0)
                return lhs < rhs;
            return lhs > rhs;
        }
    };

    typedef struct
    {
        ObjectStateStructDat state;
        double               odometer;
    } ReplayEntry;

    class Replay
    {
    public:
        Dat::DatHeader                                 header_;
        std::vector<ReplayEntry>                       data_;
        std::map<int, PropertyTimeline, MapComparator> objects_timeline_;
        std::vector<float>                             timestamps_;
        std::unordered_map<int, ReplayEntry>           object_state_cache_;
        int                                            ghost_ghost_counter_ = -1;

        Replay(std::string filename, bool clean, float fixed_timestep = 0.0f);
        Replay(const std::string directory, const std::string scenario, std::string create_datfile);
        ~Replay();

        /**
                Go to specific time
                @param time timestamp (0 = beginning, -1 end)
                @param stop_at_next_frame If true move max to next/previous time frame
        */
        void                  GoToTime(double target_time, bool stop_at_next_frame = false);
        void                  GoToDeltaTime(double dt, bool stop_at_next_frame = false);
        void                  GetReplaysFromDirectory(const std::string dir, const std::string sce);
        size_t                GetNumberOfScenarios() const;
        void                  GoToStart();
        void                  GoToEnd();
        int                   GoToNextFrame();
        void                  GoToPreviousFrame();
        unsigned int          FindNextTimestamp(bool wrap = false) const;
        unsigned int          FindPreviousTimestamp(bool wrap = false) const;
        ReplayEntry*          GetEntry(int id);
        ObjectStateStructDat* GetState(int id);
        void                  SetStartTime(double time);
        void                  SetStopTime(double time);
        double                GetStartTime() const
        {
            return startTime_;
        }
        double GetStopTime() const
        {
            return stopTime_;
        }
        double GetTime() const
        {
            return time_;
        }
        int GetIndex() const
        {
            return static_cast<int>(index_);
        }
        void SetRepeat(bool repeat)
        {
            repeat_ = repeat;
        }
        void CleanEntries(std::vector<ReplayEntry>& entries);
        void BuildData(std::vector<std::pair<std::string, std::vector<ReplayEntry>>>& scenarios);
        void CreateMergedDatfile(const std::string filename) const;

        /* Anything PacketHandler related, maybe move there later */
        struct LoggedEvent
        {
            double                                                                          timestamp;
            Dat::PacketId                                                                   packet_id;
            int                                                                             obj_id;
            std::variant<int, bool, double, std::string, id_t, Dat::Pose, Dat::BoundingBox> value;
        };

        template <typename... Data>
        int ReadPacket(const Dat::PacketHeader& header, Data&... data);
        template <typename T, typename D>
        void AddToTimeline(Timeline<T>& timeline, D data);

        int ReadStringPacket(std::string& str);

        int ParsePackets(const std::string& filename);
        int FillHeader();

        void BuildDataFromPackets();

        void BuildDataFixedTimestep();
        void BuildDataVariableTimestep();

        void IsDataFixedTimestep(const std::vector<scenarioengine::ReplayEntry>* entry);

        void ClearData();

        // void SetIncludeGhostReset(bool include)
        // {
        //     include_ghost_reset_ = include;
        // }

        ReplayEntry GetReplayEntryAtTimeIncremental(int id, float t) const
        {
            ReplayEntry entry;

            const auto& timeline = objects_timeline_.at(id);

            entry.state.info.id             = id;
            entry.state.info.timeStamp      = t;
            entry.state.info.model_id       = timeline.model_id_.get_value_incremental(t);
            entry.state.info.obj_type       = timeline.obj_type_.get_value_incremental(t);
            entry.state.info.obj_category   = timeline.obj_category_.get_value_incremental(t);
            entry.state.info.ctrl_type      = timeline.ctrl_type_.get_value_incremental(t);
            entry.state.info.name           = timeline.name_.get_value_incremental(t).c_str();
            entry.state.info.speed          = timeline.speed_.get_value_incremental(t);
            entry.state.info.wheel_angle    = timeline.wheel_angle_.get_value_incremental(t);
            entry.state.info.wheel_rot      = timeline.wheel_rot_.get_value_incremental(t);
            entry.state.info.boundingbox    = timeline.bounding_box_.get_value_incremental(t);
            entry.state.info.scaleMode      = timeline.scale_mode_.get_value_incremental(t);
            entry.state.info.visibilityMask = timeline.visibility_mask_.get_value_incremental(t);
            entry.state.info.active         = timeline.active_.get_value_incremental(t);
            entry.state.pos.x               = timeline.pose_.get_value_incremental(t).x;
            entry.state.pos.y               = timeline.pose_.get_value_incremental(t).y;
            entry.state.pos.z               = timeline.pose_.get_value_incremental(t).z;
            entry.state.pos.h               = timeline.pose_.get_value_incremental(t).h;
            entry.state.pos.p               = timeline.pose_.get_value_incremental(t).p;
            entry.state.pos.r               = timeline.pose_.get_value_incremental(t).r;
            entry.state.pos.roadId          = timeline.road_id_.get_value_incremental(t);
            entry.state.pos.laneId          = timeline.lane_id_.get_value_incremental(t);
            entry.state.pos.offset          = timeline.pos_offset_.get_value_incremental(t);
            entry.state.pos.t               = timeline.pos_t_.get_value_incremental(t);
            entry.state.pos.s               = timeline.pos_s_.get_value_incremental(t);
            entry.odometer                  = timeline.odometer_.get_value_incremental(t);

            return entry;
        }

        ReplayEntry GetReplayEntryAtTimeBinary(int id, float t) const
        {
            ReplayEntry entry;

            const auto& timeline = objects_timeline_.at(id);

            entry.state.info.id             = id;
            entry.state.info.model_id       = timeline.model_id_.get_value_binary(t);
            entry.state.info.obj_type       = timeline.obj_type_.get_value_binary(t);
            entry.state.info.obj_category   = timeline.obj_category_.get_value_binary(t);
            entry.state.info.ctrl_type      = timeline.ctrl_type_.get_value_binary(t);
            entry.state.info.name           = timeline.name_.get_value_binary(t).c_str();
            entry.state.info.speed          = timeline.speed_.get_value_binary(t);
            entry.state.info.wheel_angle    = timeline.wheel_angle_.get_value_binary(t);
            entry.state.info.wheel_rot      = timeline.wheel_rot_.get_value_binary(t);
            entry.state.info.boundingbox    = timeline.bounding_box_.get_value_binary(t);
            entry.state.info.scaleMode      = timeline.scale_mode_.get_value_binary(t);
            entry.state.info.visibilityMask = timeline.visibility_mask_.get_value_binary(t);
            entry.state.info.active         = timeline.active_.get_value_binary(t);
            entry.state.pos.x               = timeline.pose_.get_value_binary(t).x;
            entry.state.pos.y               = timeline.pose_.get_value_binary(t).y;
            entry.state.pos.z               = timeline.pose_.get_value_binary(t).z;
            entry.state.pos.h               = timeline.pose_.get_value_binary(t).h;
            entry.state.pos.p               = timeline.pose_.get_value_binary(t).p;
            entry.state.pos.r               = timeline.pose_.get_value_binary(t).r;
            entry.state.pos.roadId          = timeline.road_id_.get_value_binary(t);
            entry.state.pos.laneId          = timeline.lane_id_.get_value_binary(t);
            entry.state.pos.offset          = timeline.pos_offset_.get_value_binary(t);
            entry.state.pos.t               = timeline.pos_t_.get_value_binary(t);
            entry.state.pos.s               = timeline.pos_s_.get_value_binary(t);
            entry.odometer                  = timeline.odometer_.get_value_binary(t);

            return entry;
        }

    private:
        std::ifstream            file_;
        std::vector<std::string> scenarios_;
        double                   time_;
        double                   startTime_  = 0.0;
        double                   stopTime_   = 0.0;
        unsigned int             startIndex_ = 0;
        unsigned int             stopIndex_  = 0;
        unsigned int             index_      = 0;
        bool                     repeat_     = false;
        bool                     clean_      = true;
        std::string              create_datfile_;

        /* PacketHandler stuff */
        std::unordered_map<int, std::vector<ReplayEntry>> obj_events_map_;
        std::set<int>                                     object_ids_;  // Keep track of object IDs
        std::unordered_map<int, std::string>              id_to_name_;  // Keep track of object IDs
        std::unordered_map<int, size_t>                   id_to_search_idx_;
        float                                             timestamp_             = 0.0f;
        float                                             fixed_timestep_        = -1.0f;  // Fixed timestep for replay, if specified
        bool                                              logged_timestep_fixed_ = true;   // Deduced from fixed_timestep_ or dt in data
        id_t                                              previous_p_id_         = 22;     // 22 outside length of PacketId

        Timeline<float>                   dt_timeline_;
        int                               current_object_id_;
        scenarioengine::PropertyTimeline* current_object_timeline_;
        std::optional<float>              min_timestep_ = std::nullopt;  // Minimum timestep in data
        // bool                              include_ghost_reset_;

        int FindIndexAtTimestamp(double timestamp, int startSearchIndex = 0);
    };

}  // namespace scenarioengine