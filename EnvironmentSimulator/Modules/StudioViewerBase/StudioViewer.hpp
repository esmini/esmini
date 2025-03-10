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

#include <osgViewer/ViewerEventHandlers>
#include <osgText/Text>

#include <string>

#include "TopViewManipulator.hpp"
#include "RoadManager.hpp"
#include "CommonMini.hpp"
#include "roadgeom.hpp"
#include "viewer.hpp"
#include "Entities.hpp"

#include <string>
#include <vector>

using namespace scenarioengine;

class StudioDataModel;

namespace viewer
{
    class StudioViewer : public viewer::Viewer
    {
    public:
        StudioViewer(roadmanager::OpenDrive* odrManager,
                     const char*             modelFilename,
                     const char*             scenarioFilename,
                     const char*             exe_path,
                     osg::ArgumentParser     arguments,
                     SE_Options*             opt = 0);

        void                SetWindowTitle(const std::string& title);
        void                Cleanup();
        void                LoadRoad();
        void                UpdateRoad();
        const osg::Vec2d&   GetMousePosition() const;
        void                MoveCameraToMapCenter();
        void                MoveCameraTo(double x, double y);
        std::vector<double> GetCameraSettings();
        void                SetCameraSettings(const std::vector<double>& settings);
        void                BackupCameraSettings();
        void                RestoreCameraSettings();
        void                SetCameraDistance(double d);
        StudioDataModel*    GetDataModel()
        {
            return data_model_;
        }

    private:
        osg::ref_ptr<osgGA::TopViewManipulator> topViewManipulator_;
        StudioDataModel*                        data_model_;
    };

    class StudioViewerEventHandler : public osgGA::GUIEventHandler
    {
    public:
        StudioViewerEventHandler(Viewer* viewer) : viewer_(viewer)
        {
        }

        using osgGA::GUIEventHandler::handle;
        bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override;

    private:
        Viewer* viewer_;
    };
}  // namespace viewer
