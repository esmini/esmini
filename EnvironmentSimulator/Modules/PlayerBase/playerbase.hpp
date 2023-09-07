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

#include <iostream>
#include <string>
#include <random>

#include "ScenarioEngine.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "Server.hpp"
#include "IdealSensor.hpp"
#ifdef _USE_OSI
#include "OSIReporter.hpp"
#endif  // _USE_OSI
#ifdef _USE_OSG
#include "viewer.hpp"
#endif

using namespace scenarioengine;

#ifdef _USE_OSG
void ReportKeyEvent(viewer::KeyEvent *keyEvent, void *data);

struct ImageCallBack
{
    viewer::ImageCallbackFunc func;
    void                     *data;
};

void RegisterImageCallback(viewer::ImageCallbackFunc func, void *data);

#endif

namespace scenarioengine
{
    class ScenarioPlayer
    {
    public:
        typedef enum
        {
            CONTROL_BY_OSC,
            CONTROL_INTERNAL,
            CONTROL_EXTERNAL,
            CONTROL_HYBRID
        } RequestControlMode;

        typedef enum
        {
            VIEWER_STATE_NOT_STARTED,
            VIEWER_STATE_STARTED,
            VIEWER_STATE_FAILED,
            VIEWER_STATE_DONE
        } ViewerState;

        typedef enum
        {
            PLAYER_STATE_UNDEFINED,
            PLAYER_STATE_PLAYING,
            PLAYER_STATE_PAUSE,
            PLAYER_STATE_STEP
        } PlayerState;

        typedef void (*ObjCallbackFunc)(ObjectStateStruct *, void *);

        typedef struct
        {
            int             id;
            ObjCallbackFunc func;
            void           *data;
        } ObjCallback;

        ScenarioPlayer(int argc, char *argv[]);
        ~ScenarioPlayer();
        int  Init();
        void PrintUsage();
        bool IsQuitRequested()
        {
            return quit_request;
        }
        void SetOSIFileStatus(bool is_on, const char *filename = 0);
        int  Frame();  // let player calculate actual time step
        void Draw();
        int  Frame(double timestep_s);
        void ScenarioPostFrame();
        int  ScenarioFrame(double timestep_s, bool keyframe);
        void ShowObjectSensors(bool mode);
        void
        AddObjectSensor(int object_index, double pos_x, double pos_y, double pos_z, double heading, double near, double far, double fovH, int maxObj);
#ifdef _USE_OSG
        void InitVehicleModel(Object *obj, viewer::CarModel *model);
#endif
        void AddOSIDetection(int object_index);
        void SteeringSensorSetVisible(int object_index, bool value);
        void SetFixedTimestep(double timestep)
        {
            fixed_timestep_ = timestep;
        }
        double GetFixedTimestep()
        {
            return fixed_timestep_;
        }
        int GetOSIFreq()
        {
            return osi_freq_;
        }
        void        RegisterObjCallback(int id, ObjCallbackFunc func, void *data);
        void        UpdateCSV_Log();
        int         GetNumberOfParameters();
        const char *GetParameterName(int index, OSCParameterDeclarations::ParameterType *type);
        int         SetParameterValue(const char *name, const void *value);
        int         GetParameterValue(const char *name, void *value);
        int         GetParameterValueInt(const char *name, int &value);
        int         GetParameterValueDouble(const char *name, double &value);
        int         GetParameterValueString(const char *name, const char *&value);
        int         GetParameterValueBool(const char *name, bool &value);
        int         SetParameterValue(const char *name, int value);
        int         SetParameterValue(const char *name, double value);
        int         SetParameterValue(const char *name, const char *value);
        int         SetParameterValue(const char *name, bool value);
        int         GetNumberOfVariables();
        const char *GetVariableName(int index, OSCParameterDeclarations::ParameterType *type);
        int         SetVariableValue(const char *name, const void *value);
        int         GetVariableValue(const char *name, void *value);
        int         GetVariableValueInt(const char *name, int &value);
        int         GetVariableValueDouble(const char *name, double &value);
        int         GetVariableValueString(const char *name, const char *&value);
        int         GetVariableValueBool(const char *name, bool &value);
        int         SetVariableValue(const char *name, int value);
        int         SetVariableValue(const char *name, double value);
        int         SetVariableValue(const char *name, const char *value);
        int         SetVariableValue(const char *name, bool value);
        void        SetQuitRequest(bool quit)
        {
            quit_request = quit;
        }
        void SetState(PlayerState state)
        {
            state_ = state;
        }
        PlayerState GetState()
        {
            return state_;
        }
        bool IsPaused()
        {
            return GetState() == PlayerState::PLAYER_STATE_PAUSE;
        }
        int GetCounter()
        {
            return frame_counter_;
        }
        int LoadParameterDistribution(std::string filename);

        // TODO
        // int GetNumberOfVehicleProperties(){return 4;};
        int                     GetNumberOfProperties(int index);
        const char             *GetPropertyName(int index, int propertyIndex);
        const char             *GetPropertyValue(int index, int propertyIndex);
        roadmanager::OpenDrive *GetODRManager()
        {
            return odr_manager;
        }

        CSV_Logger      *CSV_Log;
        ScenarioEngine  *scenarioEngine;
        ScenarioGateway *scenarioGateway;
#ifdef _USE_OSI
        OSIReporter *osiReporter;
#else
        void *osiReporter;
#endif  // _USE_OSI

#ifdef _USE_OSG
        viewer::Viewer                          *viewer_;
        std::vector<viewer::SensorViewFrustum *> sensorFrustum;
#ifdef _USE_OSI
        viewer::OSISensorDetection *OSISensorDetection;
#endif  // _USE_OSI
        ViewerState viewerState_;
        int         InitViewer();
        void        CloseViewer();
        void        ViewerFrame(bool init = false);

        int SaveImagesToRAM(bool state);
        int SaveImagesToFile(int nrOfFrames);

        OffScreenImage *FetchCapturedImagePtr();
        void            AddCustomCamera(double x, double y, double z, double h, double p, bool fixed_pos);
        void            AddCustomCamera(double x, double y, double z, bool fixed_pos);
        int             AddCustomLightSource(double x, double y, double z, double intensity);
        void            AddCustomFixedTopCamera(double x, double y, double z, double rot);
#else
        void *viewer_;
#endif
        roadmanager::OpenDrive     *odr_manager;
        std::vector<ObjectSensor *> sensor;
        const double                maxStepSize;
        const double                minStepSize;
        SE_Options                  opt;
        std::vector<ObjCallback>    objCallback;
        std::string                 exe_path_;
        SE_Semaphore                player_init_semaphore;
        SE_Semaphore                viewer_init_semaphore;

    private:
        double      trail_dt;
        SE_Thread   thread;
        SE_Mutex    mutex;
        bool        quit_request;
        bool        threads;
        bool        launch_server;
        bool        launch_action_server;
        bool        disable_controllers_;
        double      fixed_timestep_;
        int         osi_freq_;
        int         frame_counter_;
        std::string osi_receiver_addr;
        int         argc_;
        char      **argv_;
        std::string titleString;
        PlayerState state_;
    };

}  // namespace scenarioengine
