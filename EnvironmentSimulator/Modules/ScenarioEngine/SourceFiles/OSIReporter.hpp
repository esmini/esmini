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

#include "UDP.hpp"
#include "IdealSensor.hpp"
#include "ScenarioGateway.hpp"
#include "ScenarioEngine.hpp"
#include "osi_sensordata.pb.h"
#include "osi_object.pb.h"
#include "osi_groundtruth.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_version.pb.h"
#include "osi_common.pb.h"
#include "osi_trafficcommand.pb.h"
#include "osi_trafficupdate.pb.h"
#include "osi_version.pb.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <math.h>

#define DEFAULT_OSI_TRACE_FILENAME "ground_truth.osi"

using namespace scenarioengine;

class OSIReporter
{
public:
    OSIReporter(ScenarioEngine* scenarioengine);
    ~OSIReporter();

    typedef struct
    {
        OSCPrivateAction*             action;
        StoryBoardElement::State      state;
        StoryBoardElement::Transition transition;
    } TrafficCommandStateChange;

    enum class OSIStaticReportMode
    {
        DEFAULT,
        API,
        API_AND_LOG,
    };

    /**
    Creates and opens osi file
    @param filename Optional filename, including path. Set to 0 to use default.
    */
    bool OpenOSIFile(const char* filename);
    /**
    Closes any open osi file
    */
    void CloseOSIFile();
    /**
    Writes GroundTruth in the OSI file
    */
    bool WriteOSIFile();
    /**
    Flush (force write) the OSI file
    */
    void FlushOSIFile();
    /**
    Decide how the static data should be handled during each frame
    */
    void SetOSIStaticReportMode(OSIStaticReportMode mode);
    /**
    Calls UpdateOSIStaticGroundTruth and UpdateOSIDynamicGroundTruth
    */
    int UpdateOSIGroundTruth(const std::vector<std::unique_ptr<ObjectState>>& objectState);
    /**
    Fills up the osi message with  static GroundTruth
    */
    int UpdateOSIStaticGroundTruth(const std::vector<std::unique_ptr<ObjectState>>& objectState);
    /**
    Fills up the osi message with dynamic GroundTruth
    */
    int UpdateOSIDynamicGroundTruth(const std::vector<std::unique_ptr<ObjectState>>& objectState);
    /**
    Fills up the osi message with Stationary Object from the OpenDRIVE description
    */
    int UpdateOSIStationaryObjectODR(roadmanager::RMObject* object);
    /**
    Fills up the osi message with Stationary Object
    */
    int UpdateOSIStationaryObject(ObjectState* objectState);
    /**
    Fills up the osi message with Host Vehicle data
    */
    int UpdateOSIHostVehicleData(ObjectState* objectState);
    /**
    Fills up the osi message with Moving Object
    */
    int UpdateOSIMovingObject(ObjectState* objectState);
    /**
    Fills up the osi message with Lane Boundary
    */
    int UpdateOSILaneBoundary();
    /**
    Fills up the osi message with Lanes
    */
    int UpdateOSIRoadLane();
    /**
    Fills the intersection type of lanes
    */
    int UpdateOSIIntersection();
    /**
    Fills the Traffic Signals
    */
    int UpdateTrafficSignals();
    /**
    Fills the Traffic Commands (scenario events)
    */
    int UpdateOSITrafficCommand();
    /**
    Crops the dynamic groundtruth around an object
    */
    void CropOSIDynamicGroundTruth(const int id, const double radius);

    void ExcludeGhost()
    {
        report_ghost_ = false;
        LOG_INFO("Excluding ghost from ground truth");
    }

    std::vector<TrafficCommandStateChange> traffic_command_state_changes_;

    void RegisterTrafficCommandStateChange(OSCPrivateAction* action, StoryBoardElement::State state, StoryBoardElement::Transition transition)
    {
        traffic_command_state_changes_.push_back({action, state, transition});
    }

    /**
    Set model reference for stationary environment as defined in OpenScenario
    */
    void SetStationaryModelReference(std::string model_reference);

    /**
     Creates a SensorView from SensorData for plotting
    */
    int CreateSensorViewFromSensorData(const osi3::SensorData& sd);

    const char*       GetOSIGroundTruth(int* size);
    const char*       GetOSIGroundTruthRaw();
    const char*       GetOSITrafficCommandRaw();
    const char*       GetOSIRoadLane(const std::vector<std::unique_ptr<ObjectState>>& objectState, int* size, int object_id);
    const char*       GetOSIRoadLaneBoundary(int* size, int global_id);
    void              GetOSILaneBoundaryIds(const std::vector<std::unique_ptr<ObjectState>>& objectState, std::vector<id_t>& ids, int object_id);
    const char*       GetOSISensorDataRaw();
    osi3::SensorView* GetSensorView();
    void              CheckDynamicTypeAndUpdate(const std::unique_ptr<ObjectState>& objectState);
    bool              IsCentralOSILane(int lane_idx);
    idx_t             GetLaneIdxfromIdOSI(id_t lane_id);
    osi3::Lane*       GetOSILaneFromGlobalId(id_t lane_global_id);
    SE_SOCKET         OpenSocket(std::string ipaddr);
    void              SerializeDynamicData();
    void              SerializeDynamicAndStaticData();
    int               GetUDPClientStatus()
    {
        return (udp_client_ ? udp_client_->GetStatus() : -1);
    }
    bool IsFileOpen() const
    {
        return osi_file.is_open();
    }
    void ReportSensors(std::vector<ObjectSensor*> sensor);

    void SetUpdated(bool value)
    {
        osi_updated_ = value;
    }

    bool GetUpdated() const
    {
        return osi_updated_;
    }

    void SetCounterPtr(int* counter)
    {
        osi_update_counter_ = counter;
    }

    int GetCounter() const
    {
        return osi_update_counter_ == nullptr ? -1 : *osi_update_counter_;
    }

    void UpdateCounterOffset()
    {
        counter_offset_ = GetCounter() + 1;  // Add 1, since counter is incremented before next OSI update
    }

    void SetOSIFrequency(int freq)
    {
        osi_freq_ = freq;
        UpdateCounterOffset();
    }

    int GetOSIFrequency() const
    {
        return osi_freq_;
    }

    /**
    Set explicit timestap
    @param nanoseconds Nano (1e-9) seconds since 1970-01-01 (epoch time)
    @return 0 if successful, -1 if not
    */
    int SetOSITimeStampExplicit(unsigned long long nanoseconds);

private:
    UDPClient*                          udp_client_;
    ScenarioEngine*                     scenario_engine_;
    std::ofstream                       osi_file;
    int*                                osi_update_counter_ = nullptr;
    int                                 counter_offset_     = 0;
    int                                 osi_freq_           = 0;
    std::string                         stationary_model_reference;
    void                                CreateMovingObjectFromSensorData(const osi3::SensorData& sd, int obj_nr);
    void                                CreateLaneBoundaryFromSensordata(const osi3::SensorData& sd, int lane_boundary_nr);
    bool                                osi_updated_        = false;
    bool                                osi_initialized_    = false;
    bool                                report_ghost_       = true;
    OSIStaticReportMode                 static_update_mode_ = OSIStaticReportMode::DEFAULT;
    std::vector<std::pair<int, double>> osi_crop_           = {};  // id, radius
};
