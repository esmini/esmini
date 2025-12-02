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
#ifdef _USE_OSG
#include "trafficlightmodel.hpp"
#endif  // _USE_OSG

namespace scenarioengine
{
    // Reworked data structures to hold properties over time
    template <typename T>
    struct Timeline
    {
        std::vector<std::pair<double, T>> values;          // Pairs of timestamp and value
        mutable size_t                    last_index = 0;  // Set as mutable to allow modification in const methods
        mutable double                    last_time  = LARGE_NUMBER;

        std::optional<T> get_value_incremental(double time) const noexcept
        {
            if (values.empty())
            {
                return std::nullopt;
            }

            if (values.size() == 1)
            {
                last_index = 0;
                last_time  = values[0].first;
                return values[0].second;
            }

            size_t idx = last_index;

            if (NEAR_NUMBERS(last_time, time))
            {
                return values[idx].second;
            }

            if (time >= values[idx].first)  // moving forward
            {
                while (idx + 1 < values.size() && values[idx + 1].first <= time + SMALL_NUMBER)
                {
                    idx++;
                }
            }
            else  // moving backward
            {
                while (idx > 0 && values[idx].first > time + SMALL_NUMBER)
                {
                    idx--;
                }
            }

            last_index = idx;
            last_time  = time;

            return values[idx].second;
        }

        std::optional<T> get_value_and_time_incremental(double time) const noexcept
        {
            if (values.empty())
            {
                return std::nullopt;
            }

            if (values.size() == 1)
            {
                last_index = 0;
                last_time  = values[0].first;
                return values[0].second;
            }

            size_t idx = last_index;

            if (NEAR_NUMBERS(last_time, time))
            {
                return values[idx].second;
            }

            if (time >= values[idx].first)  // moving forward
            {
                while (idx + 1 < values.size() && values[idx + 1].first <= time + SMALL_NUMBER)
                {
                    idx++;
                }
            }
            else  // moving backward
            {
                while (idx > 0 && values[idx].first > time + SMALL_NUMBER)
                {
                    idx--;
                }
            }

            last_index = idx;
            last_time  = time;

            return values[idx].second;
        }

        std::optional<T> get_value_binary(double time, bool upper = false) const noexcept
        {
            if (values.empty())
            {
                return std::nullopt;
            }

            if (values.size() == 1)
            {
                return values.front().second;
            }

            auto search_begin = values.begin();
            auto search_end   = values.end();

            auto it =
                std::upper_bound(search_begin, search_end, time + SMALL_NUMBER, [](double t, const std::pair<double, T>& v) { return t < v.first; });

            if (it == values.begin())
            {
                return it->second;
            }

            // end is past-the-end, so go to last element
            if (it == values.end())
            {
                --it;
            }

            if (!upper)  // We snap to lowest value, which is default
            {
                --it;
            }

            return it->second;
        }

        std::optional<size_t> get_index_binary(double time) const noexcept
        {
            if (values.empty())
            {
                return std::nullopt;
            }

            if (values.size() == 1)
            {
                return 1;
            }

            auto search_begin = values.begin();
            auto search_end   = values.end();

            auto it = std::upper_bound(search_begin, search_end, time, [](double t, const std::pair<double, T>& v) { return t < v.first; });

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
        Timeline<float>          refpoint_x_offset_;
        Timeline<float>          model_x_offset_;
        Timeline<std::string>    model3d_;
    };

    // Custom comparator ensuring map has ids ordered as:
    // <0, 1, 2, 3, -1, -2, -3, ...>
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

    struct ReplayEntry
    {
        ObjectStateStructDat state;
        double               odometer;
    };

#ifdef _USE_OSG
    struct ReplayTrafficLight
    {
        TrafficLightModel*                         model  = nullptr;
        std::vector<roadmanager::Signal::LampMode> modes_ = {};
    };
#endif  // _USE_OSG

    class Replay
    {
    public:
        // Timelines
        Timeline<double>                                                  dt_;
        std::map<int, PropertyTimeline, MapComparator>                    objects_timeline_;
        std::unordered_map<unsigned int, Timeline<Dat::TrafficLightLamp>> traffic_lights_timeline_;

        std::vector<ReplayEntry>             data_;
        Dat::DatHeader                       dat_header_;
        std::vector<double>                  timestamps_;
        std::unordered_map<int, ReplayEntry> object_state_cache_;
#ifdef _USE_OSG
        std::unordered_map<int, ReplayTrafficLight> traffic_light_cache_;
#endif  // _USE_OSG

        int ghost_ghost_counter_ = -1;

        Replay(std::string filename);
        Replay(const std::string directory, const std::string scenario, std::string create_datfile);
        ~Replay();

        void SetupGhostsTimeline();
        int  ParsePackets(const std::string& filename);
        void FillInTimestamps();
        void FillEmptyTimestamps(const double start, const double end, const double dt, std::vector<double>& v);
        void CreateMergedDatfile(const std::string filename) const;
        void ParseDatHeader(Dat::DatReader& dat_reader, const std::string& filename);

        /**
                Go to specific time
                @param time timestamp (0 = beginning, -1 end)
                @param stop_at_next_frame If true move max to next/previous time frame
        */
        void                  SetTimeToNearestTimestamp();
        unsigned int          FindIndexAtTimestamp(double timestamp);
        void                  GoToTime(double target_time, bool stop_at_next_frame = false);
        void                  GoToDeltaTime(double dt, bool stop_at_next_frame = false);
        void                  GetReplaysFromDirectory(const std::string dir, const std::string sce);
        size_t                GetNumberOfScenarios() const;
        void                  GoToStart(bool ignore_repeat = false);
        void                  GoToEnd(bool ignore_repeat = false);
        int                   GoToNextFrame();
        void                  GoToPreviousFrame();
        ObjectStateStructDat* GetState(int id);
        int                   ReadOldDatHeader(const std::string& filename);
        void                  SetStartTime(double time);
        void                  SetStopTime(double time);
        ReplayEntry           GetReplayEntryAtTimeIncremental(int id, double t) const;
        ReplayEntry           GetReplayEntryAtTimeBinary(int id, double t) const;
        std::vector<int>      GetAllObjectIDs() const;
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

    private:
        std::vector<std::string> scenarios_;
        double                   time_;
        double                   startTime_  = 0.0;
        double                   stopTime_   = 0.0;
        unsigned int             startIndex_ = 0;
        unsigned int             stopIndex_  = 0;
        unsigned int             index_      = 0;
        bool                     repeat_     = false;
        std::string              create_datfile_;
        std::vector<id_t>        unknown_pids;

        /* PacketHandler stuff */
        double                            timestamp_            = 0.0;
        bool                              ghost_timeline_setup_ = false;
        int                               current_object_id_;
        int                               ghost_controller_id_;
        scenarioengine::PropertyTimeline* current_object_timeline_;
    };

}  // namespace scenarioengine