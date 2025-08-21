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

        const T& get_value_incremental(float time) const noexcept;
        const T& get_value_binary(float time) const noexcept;
        size_t   get_index_binary(float time) const noexcept;
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
        std::vector<std::pair<float, bool>>            timestamps_;
        std::unordered_map<int, ReplayEntry>           object_state_cache_;
        int                                            ghost_ghost_counter_ = -1;

        Replay(std::string filename, bool clean, float fixed_timestep = 0.0f);
        Replay(const std::string directory, const std::string scenario, std::string create_datfile);
        ~Replay();

        void CreateMergedDatfile(const std::string filename) const;

        template <typename... Data>
        int ReadPacket(const Dat::PacketHeader& header, Data&... data);
        int ReadStringPacket(std::string& str);

        template <typename T, typename Data>
        void AddToTimeline(Timeline<T>& timeline, Data data);

        int ParsePackets(const std::string& filename);
        int FillHeader();

        /**
                Go to specific time
                @param time timestamp (0 = beginning, -1 end)
                @param stop_at_next_frame If true move max to next/previous time frame
        */
        size_t                FindIndexAtTimestamp(double timestamp);
        void                  GoToTime(double target_time, bool stop_at_next_frame = false);
        void                  GoToDeltaTime(double dt, bool stop_at_next_frame = false);
        void                  GetReplaysFromDirectory(const std::string dir, const std::string sce);
        size_t                GetNumberOfScenarios() const;
        void                  GoToStart();
        void                  GoToEnd();
        int                   GoToNextFrame();
        void                  GoToPreviousFrame();
        ObjectStateStructDat* GetState(int id);
        void                  SetStartTime(double time);
        void                  SetStopTime(double time);
        ReplayEntry           GetReplayEntryAtTimeIncremental(int id, float t) const;
        ReplayEntry           GetReplayEntryAtTimeBinary(int id, float t) const;
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
        // void SetIncludeGhostReset(bool include)
        // {
        //     include_ghost_reset_ = include;
        // }

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
        float                             timestamp_          = 0.0f;
        float                             fixed_timestep_     = -1.0f;  // Fixed timestep for replay, if specified
        id_t                              previous_packet_id_ = static_cast<id_t>(Dat::PacketId::PACKET_ID_SIZE);
        Timeline<float>                   dt_timeline_;
        int                               current_object_id_;
        scenarioengine::PropertyTimeline* current_object_timeline_;
        // bool                              include_ghost_reset_;
    };

}  // namespace scenarioengine