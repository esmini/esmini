//
// Copyright 2016 -- 2018 PMSF IT Consulting Pierre R. Mai
// Copyright 2023 BMW AG
// SPDX-License-Identifier: MPL-2.0
//

#include "esminiLib.hpp"
#include "logger.hpp"

#include "osi_common.pb.h"
#include "osi_object.pb.h"
#include "osi_groundtruth.pb.h"
#include "osi_trafficupdate.pb.h"

#include "esmini.h"

/*
 * Debug Breaks
 *
 * If you define DEBUG_BREAKS the FMU will automatically break
 * into an attached Debugger on all major computation functions.
 * Note that the FMU is likely to break all environments if no
 * Debugger is actually attached when the breaks are triggered.
 */
#if defined(DEBUG_BREAKS) && !defined(NDEBUG)
#if defined(__has_builtin) && !defined(__ibmxl__)
#if __has_builtin(__builtin_debugtrap)
#define DEBUGBREAK() __builtin_debugtrap()
#elif __has_builtin(__debugbreak)
#define DEBUGBREAK() __debugbreak()
#endif
#endif
#if !defined(DEBUGBREAK)
#if defined(_MSC_VER) || defined(__INTEL_COMPILER)
#include <intrin.h>
#define DEBUGBREAK() __debugbreak()
#else
#include <signal.h>
#if defined(SIGTRAP)
#define DEBUGBREAK() raise(SIGTRAP)
#else
#define DEBUGBREAK() raise(SIGABRT)
#endif
#endif
#endif
#else
#define DEBUGBREAK()
#endif

#include <iostream>
#include <string>
#include <algorithm>
#include <cstdint>
#include <cmath>

using namespace std;

#ifdef PRIVATE_LOG_PATH
ofstream COSMPDummySource::private_log_file;
#endif

/*
 * esmini's Log Messages
 *
 * esmini's API is global (one scenario per loaded library), and its logger
 * calls a plain function without user data, so the instance that receives
 * esmini's messages is global as well: the one that initialized the scenario.
 */

namespace
{
    EsminiOsiSource* esmini_log_receiver            = nullptr;
    bool             esmini_log_callback_registered = false;

    void receive_esmini_log(const std::string& message)
    {
        if (esmini_log_receiver != nullptr)
        {
            esmini_log_receiver->forward_esmini_log(message);
        }
    }
}  // namespace

void EsminiOsiSource::log_message(fmi2Status status, const char* category, const std::string& message)
{
    if (functions.logger == nullptr)
    {
        return;
    }
    if (status == fmi2OK && !(loggingOn && loggingCategories.count(category)))
    {
        return;
    }
    functions.logger(functions.componentEnvironment, instanceName.c_str(), status, category, "%s", message.c_str());
}

void EsminiOsiSource::forward_esmini_log(const std::string& message)
{
    // esmini's format: "[<simulation time>] [<level>] <text>\n"; the line with the wall-clock time at the start of a
    // scenario has no level and is skipped
    const size_t level_start = message.find("] [");
    if (level_start == std::string::npos)
    {
        return;
    }
    const size_t level_end = message.find("] ", level_start + 3);
    if (level_end == std::string::npos)
    {
        return;
    }
    const std::string level = message.substr(level_start + 3, level_end - level_start - 3);
    std::string       text  = message.substr(level_end + 2);
    while (!text.empty() && (text.back() == '\n' || text.back() == '\r'))
    {
        text.pop_back();
    }
    if (text.empty())
    {
        return;
    }

    fmi2Status status = fmi2OK;
    if (level == "error")
    {
        status = fmi2Error;
    }
    else if (level == "warn")
    {
        status = fmi2Warning;
    }
    log_message(status, "esmini", text);
}

void EsminiOsiSource::setup_esmini_logging()
{
    esmini_log_receiver = this;
    if (!esmini_log_callback_registered)
    {
        txtLogger.RegisterCallback(receive_esmini_log);
        esmini_log_callback_registered = true;
    }
    SE_SetLogFilePath("");                     // no log.txt
    SE_SetOptionPersistent("disable_stdout");  // nothing on stdout
    SE_SetOptionValuePersistent("log_level", loggingOn ? "info" : "warn");
}

/*
 * ProtocolBuffer Accessors
 */

void* decode_integer_to_pointer(fmi2Integer hi, fmi2Integer lo)
{
#if PTRDIFF_MAX == INT64_MAX
    union addrconv
    {
        struct
        {
            int lo;
            int hi;
        } base;
        unsigned long long address;
    } myaddr;
    myaddr.base.lo = lo;
    myaddr.base.hi = hi;
    return reinterpret_cast<void*>(myaddr.address);
#elif PTRDIFF_MAX == INT32_MAX
    return reinterpret_cast<void*>(lo);
#else
#error "Cannot determine 32bit or 64bit environment!"
#endif
}

void encode_pointer_to_integer(const void* ptr, fmi2Integer& hi, fmi2Integer& lo)
{
#if PTRDIFF_MAX == INT64_MAX
    union addrconv
    {
        struct
        {
            int lo;
            int hi;
        } base;
        unsigned long long address;
    } myaddr;
    myaddr.address = reinterpret_cast<unsigned long long>(ptr);
    hi             = myaddr.base.hi;
    lo             = myaddr.base.lo;
#elif PTRDIFF_MAX == INT32_MAX
    hi = 0;
    lo = reinterpret_cast<int>(ptr);
#else
#error "Cannot determine 32bit or 64bit environment!"
#endif
}

bool EsminiOsiSource::get_fmi_traffic_update_in(osi3::TrafficUpdate& data)
{
    if (integer_vars[FMI_INTEGER_TRAFFICUPDATE_IN_SIZE_IDX] > 0)
    {
        void* buffer =
            decode_integer_to_pointer(integer_vars[FMI_INTEGER_TRAFFICUPDATE_IN_BASEHI_IDX], integer_vars[FMI_INTEGER_TRAFFICUPDATE_IN_BASELO_IDX]);
        normal_log("OSMP",
                   "Got %08X %08X, reading from %p ...",
                   integer_vars[FMI_INTEGER_TRAFFICUPDATE_IN_BASEHI_IDX],
                   integer_vars[FMI_INTEGER_TRAFFICUPDATE_IN_BASELO_IDX],
                   buffer);
        data.ParseFromArray(buffer, integer_vars[FMI_INTEGER_TRAFFICUPDATE_IN_SIZE_IDX]);
        return true;
    }
    else
    {
        return false;
    }
}

void EsminiOsiSource::set_fmi_sensor_view_out(const osi3::SensorView& data)
{
    data.SerializeToString(currentBufferSVOut);
    encode_pointer_to_integer(currentBufferSVOut->data(),
                              integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASEHI_IDX],
                              integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASELO_IDX]);
    integer_vars[FMI_INTEGER_SENSORVIEW_OUT_SIZE_IDX] = (fmi2Integer)currentBufferSVOut->length();
    normal_log("OSMP",
               "Providing %08X %08X, writing from %p ...",
               integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASEHI_IDX],
               integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASELO_IDX],
               currentBufferSVOut->data());
    swap(currentBufferSVOut, lastBufferSVOut);
}

void EsminiOsiSource::reset_fmi_sensor_view_out()
{
    integer_vars[FMI_INTEGER_SENSORVIEW_OUT_SIZE_IDX]   = 0;
    integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASEHI_IDX] = 0;
    integer_vars[FMI_INTEGER_SENSORVIEW_OUT_BASELO_IDX] = 0;
}

void EsminiOsiSource::set_fmi_traffic_command_out(const osi3::TrafficCommand& data)
{
    data.SerializeToString(currentBufferTCOut);
    encode_pointer_to_integer(currentBufferTCOut->data(),
                              integer_vars[FMI_INTEGER_TRAFFICCOMMAND_OUT_BASEHI_IDX],
                              integer_vars[FMI_INTEGER_TRAFFICCOMMAND_OUT_BASELO_IDX]);
    integer_vars[FMI_INTEGER_TRAFFICCOMMAND_OUT_SIZE_IDX] = (fmi2Integer)currentBufferTCOut->length();
    normal_log("OSMP",
               "Providing %08X %08X, writing from %p ...",
               integer_vars[FMI_INTEGER_TRAFFICCOMMAND_OUT_BASEHI_IDX],
               integer_vars[FMI_INTEGER_TRAFFICCOMMAND_OUT_BASELO_IDX],
               currentBufferTCOut->data());
    swap(currentBufferTCOut, lastBufferTCOut);
}

void EsminiOsiSource::reset_fmi_traffic_command_out()
{
    integer_vars[FMI_INTEGER_TRAFFICCOMMAND_OUT_SIZE_IDX]   = 0;
    integer_vars[FMI_INTEGER_TRAFFICCOMMAND_OUT_BASEHI_IDX] = 0;
    integer_vars[FMI_INTEGER_TRAFFICCOMMAND_OUT_BASELO_IDX] = 0;
}

/*
 * Actual Core Content
 */

fmi2Status EsminiOsiSource::doInit()
{
    DEBUGBREAK();

    /* Booleans */
    for (int i = 0; i < FMI_BOOLEAN_VARS; i++)
        boolean_vars[i] = fmi2False;

    /* Integers */
    for (int i = 0; i < FMI_INTEGER_VARS; i++)
        integer_vars[i] = 0;

    /* Reals */
    for (int i = 0; i < FMI_REAL_VARS; i++)
        real_vars[i] = 0.0;

    /* Strings */
    for (int i = 0; i < FMI_STRING_VARS; i++)
        string_vars[i] = "";

    return fmi2OK;
}

fmi2Status EsminiOsiSource::doStart(fmi2Boolean toleranceDefined,
                                    fmi2Real    tolerance,
                                    fmi2Real    startTime,
                                    fmi2Boolean stopTimeDefined,
                                    fmi2Real    stopTime)
{
    DEBUGBREAK();

    fmiSlaveTerminated = false;

    return fmi2OK;
}

fmi2Status EsminiOsiSource::doEnterInitializationMode()
{
    DEBUGBREAK();

    return fmi2OK;
}

fmi2Status EsminiOsiSource::doExitInitializationMode()
{
    DEBUGBREAK();

    setup_esmini_logging();

    std::string esmini_args = fmi_esmini_args();
    if (esmini_args.empty())
    {
        const std::string xosc_path = fmi_xosc_path();
        if (xosc_path.empty())
        {
            log_message(fmi2Error, "esmini", "No OpenSCENARIO file selected: set the parameter xosc_path (or esmini_args)");
            return fmi2Error;
        }
        int use_viewer = fmi_use_viewer() ? 1 : 0;
#ifndef _USE_OSG
        if (use_viewer != 0)
        {
            log_message(fmi2Warning, "esmini", "esmini's viewer is not part of this build; the parameter use_viewer is ignored");
            use_viewer = 0;
        }
#endif
        if (SE_Init(xosc_path.c_str(), 0, use_viewer, 0, 0) != 0)
        {
            log_message(fmi2Error, "esmini", "Failed to initialize the scenario: " + xosc_path);
            return fmi2Error;
        }
    }
    else
    {
        std::vector<std::string> args  = {"esmini(lib)"};
        size_t                   start = 0, end = 0;
        while ((end = esmini_args.find(' ', start)) != size_t(-1))
        {
            args.push_back(esmini_args.substr(start, end - start));
            start = end + 1;
        }
        args.push_back(esmini_args.substr(start).c_str());

        // create the argv array for SE_InitWithArgs
        int argc = static_cast<int>(args.size());
        std::vector<const char*> argv;
        argv.reserve(argc);
        for (const auto& arg : args)
        {
            argv.push_back(arg.c_str());
        }
        if (SE_InitWithArgs(argc, argv.data()) != 0)
        {
            log_message(fmi2Error, "esmini", "Failed to initialize the scenario with the arguments: " + esmini_args);
            return fmi2Error;
        }
    }

    return fmi2OK;
}

fmi2Status EsminiOsiSource::doCalc(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPoint)
{
    DEBUGBREAK();

    // Handle OSI TrafficUpdate input
    osi3::TrafficUpdate traffic_update;
    if (get_fmi_traffic_update_in(traffic_update))
    {
        for (const auto& obj : traffic_update.update())
        {
            const int              obj_id = (int)obj.id().value();
            SE_ScenarioObjectState vehicleState;
            SE_GetObjectState(obj_id, &vehicleState);
            if (obj.base().has_orientation())
            {
                vehicleState.h = (float)obj.base().orientation().yaw();
            }

            if (obj.base().has_position())
            {
                vehicleState.x =
                    obj.base().position().x() - vehicleState.centerOffsetX * cos(vehicleState.h) - vehicleState.centerOffsetY * sin(vehicleState.h);
                vehicleState.y =
                    obj.base().position().y() - vehicleState.centerOffsetX * sin(vehicleState.h) - vehicleState.centerOffsetY * cos(vehicleState.h);
            }

            SE_ReportObjectPosXYH(obj_id, vehicleState.x, vehicleState.y, vehicleState.h);

            if (obj.base().has_velocity())
            {
                SE_ReportObjectVel(obj_id, obj.base().velocity().x(), obj.base().velocity().y(), obj.base().velocity().z());
            }
        }
    }
    else
    {
        normal_log("OSMP", "No TrafficUpdate received.");
    }

    SE_SetOSIStaticReportMode(SE_OSIStaticReportMode::API);

    // Run simulation step
    if (SE_StepDT((float)communicationStepSize) != 0)
    {
        log_message(fmi2Error, "esmini", "Failed to run the simulation step");
        return fmi2Error;
    }

    const auto* se_osi_ground_truth = reinterpret_cast<const osi3::GroundTruth*>(SE_GetOSIGroundTruthRaw());  // Fetch OSI struct

    osi3::SensorView currentOut;
    currentOut.Clear();
    currentOut.mutable_sensor_id()->set_value(0);
    currentOut.mutable_version()->CopyFrom(se_osi_ground_truth->version());
    currentOut.mutable_host_vehicle_id()->set_value(se_osi_ground_truth->host_vehicle_id().value());
    double const time = currentCommunicationPoint + communicationStepSize;
    currentOut.mutable_timestamp()->set_seconds((long long int)floor(time));
    const double sec_to_nanos = 1000000000.0;
    currentOut.mutable_timestamp()->set_nanos((int)((time - floor(time)) * sec_to_nanos));
    osi3::GroundTruth* currentGT = currentOut.mutable_global_ground_truth();
    currentGT->CopyFrom(*se_osi_ground_truth);

    set_fmi_sensor_view_out(currentOut);

    // Handle OSI TrafficCommand output
    const auto* traffic_command =
        reinterpret_cast<const osi3::TrafficCommand*>(SE_GetOSITrafficCommandRaw());  // Fetch OSI struct (via pointer, no copying of data)
    set_fmi_traffic_command_out(*traffic_command);

    set_fmi_valid(1);
    if (SE_GetQuitFlag() > 0) {
        normal_log("OSMP", ("esmini terminated with flag " + std::to_string(SE_GetQuitFlag()) + ". Terminating agent!").c_str());
        fmiSlaveTerminated = true;
        return fmi2Discard;
    }

    return fmi2OK;
}

fmi2Status EsminiOsiSource::doTerm()
{
    DEBUGBREAK();
    return fmi2OK;
}

void EsminiOsiSource::doFree()
{
    DEBUGBREAK();

    // Release esmini's global state; its messages still reach this instance meanwhile
    if (esmini_log_receiver == this)
    {
        SE_Close();
        esmini_log_receiver = nullptr;
    }
}

/*
 * Generic C++ Wrapper Code
 */

EsminiOsiSource::EsminiOsiSource(fmi2String                   theinstanceName,
                                 fmi2Type                     thefmuType,
                                 fmi2String                   thefmuGUID,
                                 fmi2String                   thefmuResourceLocation,
                                 const fmi2CallbackFunctions* thefunctions,
                                 fmi2Boolean                  thevisible,
                                 fmi2Boolean                  theloggingOn)
    : instanceName(theinstanceName),
      fmuType(thefmuType),
      fmuGUID(thefmuGUID),
      fmuResourceLocation(thefmuResourceLocation),
      functions(*thefunctions),
      visible(!!thevisible),
      loggingOn(!!theloggingOn)
{
    currentBufferSVOut = new string();
    currentBufferTCOut = new string();
    lastBufferSVOut = new string();
    lastBufferTCOut = new string();
    loggingCategories.clear();
    loggingCategories.insert("FMI");
    loggingCategories.insert("OSMP");
    loggingCategories.insert("OSI");
    loggingCategories.insert("esmini");
}

EsminiOsiSource::~EsminiOsiSource()
{
    delete currentBufferSVOut;
    delete currentBufferTCOut;
    delete lastBufferSVOut;
    delete lastBufferTCOut;
}

fmi2Status EsminiOsiSource::SetDebugLogging(fmi2Boolean theloggingOn, size_t nCategories, const fmi2String categories[])
{
    fmi_verbose_log("fmi2SetDebugLogging(%s)", theloggingOn ? "true" : "false");
    loggingOn = theloggingOn ? true : false;
    if (categories && (nCategories > 0))
    {
        loggingCategories.clear();
        for (size_t i = 0; i < nCategories; i++)
        {
            if (0 == strcmp(categories[i], "FMI"))
                loggingCategories.insert("FMI");
            else if (0 == strcmp(categories[i], "OSMP"))
                loggingCategories.insert("OSMP");
            else if (0 == strcmp(categories[i], "OSI"))
                loggingCategories.insert("OSI");
            else if (0 == strcmp(categories[i], "esmini"))
                loggingCategories.insert("esmini");
        }
    }
    else
    {
        loggingCategories.clear();
        loggingCategories.insert("FMI");
        loggingCategories.insert("OSMP");
        loggingCategories.insert("OSI");
        loggingCategories.insert("esmini");
    }
    if (esmini_log_receiver == this)
    {
        SE_SetOptionValuePersistent("log_level", loggingOn ? "info" : "warn");
        txtLogger.SetLoggerVerbosity();
    }
    return fmi2OK;
}

fmi2Component EsminiOsiSource::Instantiate(fmi2String                   instanceName,
                                           fmi2Type                     fmuType,
                                           fmi2String                   fmuGUID,
                                           fmi2String                   fmuResourceLocation,
                                           const fmi2CallbackFunctions* functions,
                                           fmi2Boolean                  visible,
                                           fmi2Boolean                  loggingOn)
{
    EsminiOsiSource* myc = new EsminiOsiSource(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);

    if (myc == NULL)
    {
        fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = NULL (alloc failure)",
                               instanceName,
                               fmuType,
                               fmuGUID,
                               (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
                               "FUNCTIONS",
                               visible,
                               loggingOn);
        return NULL;
    }

    if (myc->doInit() != fmi2OK)
    {
        fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = NULL (doInit failure)",
                               instanceName,
                               fmuType,
                               fmuGUID,
                               (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
                               "FUNCTIONS",
                               visible,
                               loggingOn);
        delete myc;
        return NULL;
    }
    else
    {
        fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = %p",
                               instanceName,
                               fmuType,
                               fmuGUID,
                               (fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
                               "FUNCTIONS",
                               visible,
                               loggingOn,
                               myc);
        return (fmi2Component)myc;
    }
}

fmi2Status EsminiOsiSource::SetupExperiment(fmi2Boolean toleranceDefined,
                                            fmi2Real    tolerance,
                                            fmi2Real    startTime,
                                            fmi2Boolean stopTimeDefined,
                                            fmi2Real    stopTime)
{
    fmi_verbose_log("fmi2SetupExperiment(%d,%g,%g,%d,%g)", toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
    return doStart(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
}

fmi2Status EsminiOsiSource::EnterInitializationMode()
{
    fmi_verbose_log("fmi2EnterInitializationMode()");
    return doEnterInitializationMode();
}

fmi2Status EsminiOsiSource::ExitInitializationMode()
{
    fmi_verbose_log("fmi2ExitInitializationMode()");
    return doExitInitializationMode();
}

fmi2Status EsminiOsiSource::DoStep(fmi2Real    currentCommunicationPoint,
                                   fmi2Real    communicationStepSize,
                                   fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
{
    fmi_verbose_log("fmi2DoStep(%g,%g,%d)", currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
    return doCalc(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
}

fmi2Status EsminiOsiSource::Terminate()
{
    fmi_verbose_log("fmi2Terminate()");
    return doTerm();
}

fmi2Status EsminiOsiSource::Reset()
{
    fmi_verbose_log("fmi2Reset()");

    fmiSlaveTerminated = false;

    doFree();
    return doInit();
}

void EsminiOsiSource::FreeInstance()
{
    fmi_verbose_log("fmi2FreeInstance()");
    doFree();
}

fmi2Status EsminiOsiSource::GetReal(const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
{
    fmi_verbose_log("fmi2GetReal(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_REAL_VARS)
            value[i] = real_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status EsminiOsiSource::GetInteger(const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
{
    fmi_verbose_log("fmi2GetInteger(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_INTEGER_VARS)
            value[i] = integer_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status EsminiOsiSource::GetBoolean(const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
{
    fmi_verbose_log("fmi2GetBoolean(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_BOOLEAN_VARS)
            value[i] = boolean_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status EsminiOsiSource::GetString(const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
{
    fmi_verbose_log("fmi2GetString(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_STRING_VARS)
            value[i] = string_vars[vr[i]].c_str();
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status EsminiOsiSource::SetReal(const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
{
    fmi_verbose_log("fmi2SetReal(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_REAL_VARS)
            real_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status EsminiOsiSource::SetInteger(const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
{
    fmi_verbose_log("fmi2SetInteger(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_INTEGER_VARS)
            integer_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status EsminiOsiSource::SetBoolean(const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
{
    fmi_verbose_log("fmi2SetBoolean(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_BOOLEAN_VARS)
            boolean_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status EsminiOsiSource::SetString(const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
{
    fmi_verbose_log("fmi2SetString(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_STRING_VARS)
            string_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

/*
 * FMI 2.0 Co-Simulation Interface API
 */

extern "C"
{
    FMI2_Export const char* fmi2GetTypesPlatform()
    {
        return fmi2TypesPlatform;
    }

    FMI2_Export const char* fmi2GetVersion()
    {
        return fmi2Version;
    }

    FMI2_Export fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t nCategories, const fmi2String categories[])
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->SetDebugLogging(loggingOn, nCategories, categories);
    }

    /*
     * Functions for Co-Simulation
     */
    FMI2_Export fmi2Component fmi2Instantiate(fmi2String                   instanceName,
                                              fmi2Type                     fmuType,
                                              fmi2String                   fmuGUID,
                                              fmi2String                   fmuResourceLocation,
                                              const fmi2CallbackFunctions* functions,
                                              fmi2Boolean                  visible,
                                              fmi2Boolean                  loggingOn)
    {
        return EsminiOsiSource::Instantiate(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
    }

    FMI2_Export fmi2Status fmi2SetupExperiment(fmi2Component c,
                                               fmi2Boolean   toleranceDefined,
                                               fmi2Real      tolerance,
                                               fmi2Real      startTime,
                                               fmi2Boolean   stopTimeDefined,
                                               fmi2Real      stopTime)
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->SetupExperiment(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
    }

    FMI2_Export fmi2Status fmi2EnterInitializationMode(fmi2Component c)
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->EnterInitializationMode();
    }

    FMI2_Export fmi2Status fmi2ExitInitializationMode(fmi2Component c)
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->ExitInitializationMode();
    }

    FMI2_Export fmi2Status fmi2DoStep(fmi2Component c,
                                      fmi2Real      currentCommunicationPoint,
                                      fmi2Real      communicationStepSize,
                                      fmi2Boolean   noSetFMUStatePriorToCurrentPointfmi2Component)
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->DoStep(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
    }

    FMI2_Export fmi2Status fmi2Terminate(fmi2Component c)
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->Terminate();
    }

    FMI2_Export fmi2Status fmi2Reset(fmi2Component c)
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->Reset();
    }

    FMI2_Export void fmi2FreeInstance(fmi2Component c)
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        myc->FreeInstance();
        delete myc;
    }

    /*
     * Data Exchange Functions
     */
    FMI2_Export fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->GetReal(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->GetInteger(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->GetBoolean(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->GetString(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->SetReal(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->SetInteger(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->SetBoolean(vr, nvr, value);
    }

    FMI2_Export fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
    {
        EsminiOsiSource* myc = (EsminiOsiSource*)c;
        return myc->SetString(vr, nvr, value);
    }

    /*
     * Unsupported Features (FMUState, Derivatives, Async DoStep, Status Enquiries)
     */
    FMI2_Export fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate* FMUstate)
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate FMUstate)
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* FMUstate)
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate FMUstate, size_t* size)
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate FMUstate, fmi2Byte serializedState[], size_t size)
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2DeSerializeFMUstate(fmi2Component c, const fmi2Byte serializedState[], size_t size, fmi2FMUstate* FMUstate)
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2GetDirectionalDerivative(fmi2Component            c,
                                                        const fmi2ValueReference vUnknown_ref[],
                                                        size_t                   nUnknown,
                                                        const fmi2ValueReference vKnown_ref[],
                                                        size_t                   nKnown,
                                                        const fmi2Real           dvKnown[],
                                                        fmi2Real                 dvUnknown[])
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status
    fmi2SetRealInputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer order[], const fmi2Real value[])
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status
    fmi2GetRealOutputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer order[], fmi2Real value[])
    {
        return fmi2Error;
    }

    FMI2_Export fmi2Status fmi2CancelStep(fmi2Component c)
    {
        return fmi2OK;
    }

    FMI2_Export fmi2Status fmi2GetStatus(fmi2Component c, const fmi2StatusKind s, fmi2Status* value)
    {
        return fmi2Discard;
    }

    FMI2_Export fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real* value)
    {
        return fmi2Discard;
    }

    FMI2_Export fmi2Status fmi2GetIntegerStatus(fmi2Component c, const fmi2StatusKind s, fmi2Integer* value)
    {
        return fmi2Discard;
    }

    FMI2_Export fmi2Status fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind s, fmi2Boolean* value)
    {
        if (s == fmi2StatusKind::fmi2Terminated) {
            EsminiOsiSource* myc = (EsminiOsiSource*)c;
            *value = myc->get_slave_terminated() ? fmi2True : fmi2False;
            return fmi2OK;
        }
        return fmi2Discard;
    }

    FMI2_Export fmi2Status fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind s, fmi2String* value)
    {
        return fmi2Discard;
    }
}
