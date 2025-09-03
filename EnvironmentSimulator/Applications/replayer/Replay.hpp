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
        std::vector<std::pair<double, T>> values;          // Pairs of timestamp and value
        mutable size_t                    last_index = 0;  // Set as mutable to allow modification in const methods
        mutable double                    last_time  = LARGE_NUMBER;

        const T& get_value_incremental(double time) const noexcept
        {
            if (values.empty())
            {
                LOG_ERROR_AND_QUIT("Timeline is empty, cannot get value at time {}", time);
            }

            size_t idx        = last_index;
            double desired_dt = time - values[idx].first;

            if (NEAR_NUMBERS(last_time, time))
            {
                return values[idx].second;
            }

            if (time >= values[idx].first)  // Requested time is after last searched value
            {
                while (idx + 1 < values.size())
                {
                    double step = values[idx + 1].first - values[last_index].first;
                    if (desired_dt < step - SMALL_NUMBER)
                        break;

                    idx++;
                }
            }
            else  // Requested time is before last searched value
            {
                while (idx > 0)
                {
                    double step = values[last_index].first - values[idx - 1].first;
                    if (-desired_dt < step - SMALL_NUMBER)
                        break;

                    idx--;
                }
            }

            last_index = idx;  // Save the index for next call
            last_time  = time;

            return values[idx].second;
        }

        const T& get_value_binary(double time, bool upper = false) const noexcept
        {
            if (values.empty())
            {
                LOG_ERROR_AND_QUIT("Timeline is empty, cannot get value at time {}", time);
            }

            auto search_begin = values.begin();
            auto search_end   = values.end();

            auto it = std::upper_bound(search_begin, search_end, time, [](double t, const std::pair<double, T>& v) { return t < v.first; });

            if (it == values.begin())
            {
                // last_index = 0;
                return it->second;
            }

            if (!upper)  // We snap to lowest value, which is default
            {
                --it;
            }

            return it->second;
        }

        size_t get_index_binary(double time) const noexcept
        {
            if (values.empty())
            {
                LOG_ERROR_AND_QUIT("Timeline is empty, cannot get value at time {}", time);
            }

            if (NEAR_NUMBERS(last_time, time))
            {
                return last_index;
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

        double last_restart_time = -1.0f;
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
        std::vector<ReplayEntry>                       data_;
        Dat::DatHeader                                 dat_header_;
        Timeline<double>                               dt_;
        std::map<int, PropertyTimeline, MapComparator> objects_timeline_;
        std::vector<std::pair<double, bool>>           timestamps_;
        std::unordered_map<int, ReplayEntry>           object_state_cache_;
        int                                            ghost_ghost_counter_ = -1;

        Replay(std::string filename);
        Replay(const std::string directory, const std::string scenario, std::string create_datfile);
        ~Replay();

        // void CreateMergedDatfile(const std::string filename) const;

        template <typename T, typename Data>
        void AddToTimeline(Timeline<T>& timeline, Data data);

        int  ParsePackets(const std::string& filename);
        void FillInTimestamps();
        void CreateMergedDatfile(const std::string filename) const;

        /**
                Go to specific time
                @param time timestamp (0 = beginning, -1 end)
                @param stop_at_next_frame If true move max to next/previous time frame
        */
        void                  RoundTime();
        size_t                FindIndexAtTimestamp(double timestamp);
        void                  GoToSignificantTimestamp(bool search_forward);
        void                  GoToTime(double target_time, bool stop_at_next_frame = false);
        void                  GoToDeltaTime(double dt, bool stop_at_next_frame = false);
        void                  GetReplaysFromDirectory(const std::string dir, const std::string sce);
        size_t                GetNumberOfScenarios() const;
        void                  GoToStart(bool ignore_repeat = false);
        void                  GoToEnd(bool ignore_repeat = false);
        int                   GoToNextFrame();
        void                  GoToPreviousFrame();
        ObjectStateStructDat* GetState(int id);
        void                  SetStartTime(double time);
        void                  SetStopTime(double time);
        ReplayEntry           GetReplayEntryAtTimeIncremental(int id, double t) const;
        ReplayEntry           GetReplayEntryAtTimeBinary(int id, double t) const;
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
        void SetNearestTimestepAtTime(double time)
        {
            if (timestamps_.empty())
            {
                return;
            }

            auto it = std::lower_bound(timestamps_.begin(),
                                       timestamps_.end(),
                                       time,
                                       [](const std::pair<float, bool>& p, float value) { return p.first < value; });

            if (it == timestamps_.begin())
            {
                time_ = timestamps_.begin()->first;
            }
            else if (it == timestamps_.end())
            {
                time_ = timestamps_.back().first;
            }
            else
            {
                time_ = it->first;
            }
        }
        // void SetIncludeGhostReset(bool include)
        // {
        //     include_ghost_reset_ = include;
        // }

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

        /* PacketHandler stuff */
        double                            timestamp_          = 0.0;
        std::optional<double>             first_timestamp_    = std::nullopt;
        id_t                              previous_packet_id_ = static_cast<id_t>(Dat::PacketId::PACKET_ID_SIZE);
        double                            prev_dt_            = -1.0;
        int                               current_object_id_;
        scenarioengine::PropertyTimeline* current_object_timeline_;
    };

}  // namespace scenarioengine