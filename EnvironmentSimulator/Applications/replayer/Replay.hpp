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

        std::optional<T> get_value_incremental(double time) const
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

            if (idx != last_index || idx == 0)
            {
                last_index = idx;
                last_time  = values[idx].first;
            }

            return values[idx].second;
        }

        std::optional<T> get_value_binary(double time, bool upper = false) const
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

        std::optional<size_t> get_index_binary(double time) const
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

            auto it =
                std::upper_bound(search_begin, search_end, time, [](double t, const std::pair<double, T>& v) { return t < v.first - SMALL_NUMBER; });

            if (it == values.begin())
            {
                return 0;
            }

            return static_cast<size_t>(std::distance(values.begin(), it));
        }

        /* Finds all values from previously found value until given time */
        std::vector<std::pair<double, T>> get_values_until_time(double time) const
        {
            if (values.empty())
            {
                return {};
            }

            if (values.size() == 1 || NEAR_NUMBERS(time, values[0].first))
            {
                last_index = 0;
                last_time  = values[0].first;
                return {values[0]};
            }

            if (NEAR_NUMBERS(last_time, time))
            {
                return {values[last_index]};
            }

            size_t                            idx = last_index;
            std::vector<std::pair<double, T>> ret = {};

            if (time < last_time - SMALL_NUMBER)  // move backwards to find right index later
            {
                while (idx > 0 && time < values[idx].first)
                {
                    idx--;
                }
                last_index = idx;
                last_time  = values[idx].first;
            }
            else
            {
                while (idx < values.size() && values[idx].first <= time + SMALL_NUMBER)
                {
                    ret.push_back(values[idx]);
                    last_index = idx;
                    last_time  = values[idx].first;
                    idx++;
                }
            }

            return ret;
        }

        std::vector<T> get_values_at_time(double time) const
        {
            std::vector<T> ret = {};
            if (values.empty())
            {
                return ret;
            }

            if (values.size() == 1)
            {
                last_index = 0;
                last_time  = values[0].first;
                return {values[0].second};
            }

            size_t idx = last_index;

            if (idx == 0 || values[idx - 1].first < time - SMALL_NUMBER)  // Find forward
            {
                while (NEAR_NUMBERS(values[idx].first, time))
                {
                    ret.push_back(values[idx].second);
                    idx++;
                }
            }
            else if (idx == values.size() - 1 || values[idx + 1].first > time + SMALL_NUMBER)  // Find backward
            {
                while (NEAR_NUMBERS(values[idx].first, time))
                {
                    ret.push_back(values[idx].second);
                    idx--;
                }
            }
            else  // We are in the middle of a range, search backward then forward
            {
                auto temp_idx = idx;
                while (NEAR_NUMBERS(values[temp_idx].first, time))
                {
                    ret.push_back(values[temp_idx].second);
                    temp_idx--;
                }

                temp_idx = idx + 1;

                while (NEAR_NUMBERS(values[temp_idx].first, time))
                {
                    ret.push_back(values[temp_idx].second);
                    temp_idx++;
                }

                idx        = temp_idx;
                last_index = idx;  // Save so we dont have to search backwards and forwards again
            }

            return ret;
        }

        // Function will delete all data between the [start_time, end_time] and insert input instead
        void replace_data_between_times(const double                             start_time,
                                        const double                             end_time,
                                        const std::vector<std::pair<double, T>>& input,
                                        bool                                     clean = false)
        {
            if (input.empty() || values.empty())
            {
                return;
            }

            auto first =
                std::lower_bound(values.begin(), values.end(), start_time, [](const auto& v, double t) { return v.first < t - SMALL_NUMBER; });

            auto last = std::upper_bound(values.begin(),
                                         values.end(),
                                         end_time,
                                         [](double t, const std::pair<double, T>& v) { return t < v.first - SMALL_NUMBER; });

            auto insert_pos = values.erase(first, last);

            values.insert(insert_pos, input.begin(), input.end());

            if (clean && values.size() > 1)
            {
                clean_duplicate_timestamps();
            }

            return;
        }

        void clean_duplicate_timestamps()
        {
            auto it = values.begin();
            while (it + 1 != values.end())
            {
                if (NEAR_NUMBERS(it->first, (it + 1)->first))
                {
                    it = values.erase(it);  // erase first of the duplicates
                }
                else
                {
                    it++;
                }
            }
        }
    };

    struct PropertyTimeline
    {
        Timeline<int>                     model_id_;
        Timeline<int>                     obj_type_;
        Timeline<int>                     obj_category_;
        Timeline<int>                     ctrl_type_;
        Timeline<std::string>             name_;
        Timeline<float>                   speed_;
        Timeline<float>                   wheel_angle_;
        Timeline<float>                   wheel_rot_;
        Timeline<OSCBoundingBox>          bounding_box_;
        Timeline<int>                     scale_mode_;
        Timeline<int>                     visibility_mask_;
        Timeline<Dat::Pose>               pose_;
        Timeline<id_t>                    road_id_;
        Timeline<int>                     lane_id_;
        Timeline<float>                   pos_offset_;
        Timeline<float>                   pos_t_;
        Timeline<float>                   pos_s_;
        Timeline<bool>                    active_;
        Timeline<float>                   odometer_;
        Timeline<float>                   refpoint_x_offset_;
        Timeline<float>                   model_x_offset_;
        Timeline<std::string>             model3d_;
        Timeline<std::vector<SE_Point2D>> outline_;

        Timeline<Dat::LightState> light_state_[static_cast<size_t>(Object::VehicleLightType::VEHICLE_LIGHT_SIZE)];
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
        bool                 has_lightstate = false;
    };

#ifdef _USE_OSG
    struct ReplayTrafficLight
    {
        TrafficLightModel*                         model  = nullptr;
        std::vector<roadmanager::Signal::LampMode> modes_ = {};
    };
#endif  // _USE_OSG

    // Used in dat-merging
    struct PacketSlice
    {
        double                          timestamp;
        std::vector<Dat::PacketGeneric> packets;
    };

    class Replay
    {
    public:
        // Timelines
        Timeline<double>                                                  dt_;
        Timeline<std::string>                                             element_state_changes_;
        std::map<int, PropertyTimeline, MapComparator>                    objects_timeline_;
        std::unordered_map<unsigned int, Timeline<Dat::TrafficLightLamp>> traffic_lights_timeline_;

        std::vector<ReplayEntry>             data_;
        Dat::DatHeader                       dat_header_;
        std::vector<double>                  timestamps_;
        std::unordered_map<int, ReplayEntry> object_state_cache_;
#ifdef _USE_OSG
        std::unordered_map<int, ReplayTrafficLight> traffic_light_cache_;
#endif  // _USE_OSG

        Replay(std::string filename, bool quiet = false);
        Replay(const std::string directory, const std::string scenario, std::string create_datfile);
        ~Replay();

        void           UpdateGhostsTimelineAfterRestart(size_t idx);
        int            ParsePackets();
        std::string    BuildElementStateChange(const std::string& element_state);
        void           FillInTimestamps();
        void           FillEmptyTimestamps(const double start, const double end, const double dt, std::vector<double>& v);
        void           CalculateNewDt();
        void           CreateMergedDatfile(const std::string filename) const;
        Dat::DatHeader ParseDatHeader(const std::string& filename);
        bool           ExtractPacketsAsSlices(bool dt_in_slice = true, size_t scenario_idx = 0);
        void           ExtractGhostRestarts();
        void           FlattenSlices(bool add_ghost_restart = true);
        void           RemoveDuplicateTimestampsInSlices(const Timeline<double>& dts);
        void           MergeAndCleanTimestamps(const std::vector<std::vector<double>>& scenarios_timestamps);

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
        bool HasLightStates() const
        {
            return has_lightstates_;
        }

    private:
        std::vector<std::string> scenarios_;
        double                   time_;
        bool                     quiet_;
        double                   startTime_  = 0.0;
        double                   stopTime_   = 0.0;
        unsigned int             startIndex_ = 0;
        unsigned int             stopIndex_  = 0;
        unsigned int             index_      = 0;
        bool                     repeat_     = false;
        std::string              create_datfile_;
        std::vector<id_t>        unknown_pids;
        bool                     eos_received_       = false;  // end of scenario packet
        bool                     has_lightstates_    = false;
        std::vector<uint8_t>     lightstate_entities = {};

        /* PacketHandler stuff */
        std::unique_ptr<Dat::DatReader>              dat_reader_;
        std::unique_ptr<Dat::DatWriter>              dat_writer_;
        double                                       timestamp_ = 0.0;
        int                                          current_object_id_;
        int                                          ghost_controller_id_;
        scenarioengine::PropertyTimeline*            current_object_timeline_;
        std::vector<std::vector<Dat::PacketGeneric>> generic_packets_ = {};
        std::vector<PacketSlice>                     packet_slices_   = {};
        std::vector<std::vector<PacketSlice>>        ghost_restarts_;
        std::vector<std::pair<double, double>>       restart_timestamps_;  // start & stop of each ghost reset
        Timeline<double>                             dts_;
    };

}  // namespace scenarioengine