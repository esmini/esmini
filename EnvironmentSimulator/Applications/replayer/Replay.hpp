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
    template <typename T>
    struct ReplayValue
    {
        float timestamp;
        T     value;
    };

    typedef struct
    {
        ObjectStateStructDat state;
        double               odometer;
    } ReplayEntry;

    class Replay
    {
    public:
        Dat::DatHeader           header_;
        std::vector<ReplayEntry> data_;

        Replay(std::string filename, bool clean, float fixed_timestep = 0.0f);
        Replay(const std::string directory, const std::string scenario, std::string create_datfile);
        ~Replay();

        /**
                Go to specific time
                @param time timestamp (0 = beginning, -1 end)
                @param stop_at_next_frame If true move max to next/previous time frame
        */
        void                  GoToTime(double time, bool stop_at_next_frame = false);
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

        int ReadStringPacket(std::string& str);

        int ParsePackets(const std::string& filename);
        int FillHeader();

        void BuildDataFromPackets();

        void BuildDataFixedTimestep();
        void BuildDataVariableTimestep();

        void IsDataFixedTimestep(const std::vector<scenarioengine::ReplayEntry>* entry);

        void ClearData();

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
        std::unordered_map<int, ReplayEntry>              object_state_cache_;
        std::set<int>                                     object_ids_;  // Keep track of object IDs
        std::unordered_map<int, std::string>              id_to_name_;  // Keep track of object IDs
        std::unordered_map<int, size_t>                   id_to_search_idx_;
        float                                             timestamp_             = 0.0f;
        float                                             fixed_timestep_        = -1.0f;  // Fixed timestep for replay, if specified
        bool                                              logged_timestep_fixed_ = true;   // Deduced from fixed_timestep_ or dt in data
        id_t                                              previous_p_id_         = 22;     // 22 outside length of PacketId

        // Reworked data structures to hold properties over time
        template <typename T>
        struct Timeline
        {
            std::vector<std::pair<float, T>> values;  // Pairs of timestamp and value
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
        };

        std::unordered_map<int, PropertyTimeline> objects_timeline_;
        Timeline<float>                           dt_timeline_;
        Timeline<float>                           timestamps_;
        int                                       current_object_id_;
        scenarioengine::Replay::PropertyTimeline* current_object_timeline_;
        std::optional<float>                      min_timestep_ = std::nullopt;  // Minimum timestep in data

        int FindIndexAtTimestamp(double timestamp, int startSearchIndex = 0);
    };

}  // namespace scenarioengine