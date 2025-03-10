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

#include "StudioViewer.hpp"

#include <osg/Group>
#include <osgGA/KeySwitchMatrixManipulator>

#ifdef __linux__
#include <X11/Xlib.h>
#include <osgViewer/api/X11/GraphicsWindowX11>
#else
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "StudioGui.hpp"

#include <experimental/filesystem>

#define PERSP_FOV 30.0
#define ORTHO_FOV 1.0

osg::ref_ptr<osg::Group> g_tmp_lines = nullptr;

class ImGuiInitOperation : public osg::Operation
{
public:
    ImGuiInitOperation() : osg::Operation("ImGuiInitOperation", false)
    {
    }

    void operator()(osg::Object* object) override
    {
        osg::GraphicsContext* context = dynamic_cast<osg::GraphicsContext*>(object);
        if (!context)
            return;

        static bool once = false;
        if (once)
            ImGui_ImplOpenGL3_Shutdown();
        once = true;

        if (!ImGui_ImplOpenGL3_Init())
        {
            std::cout << "ImGui_ImplOpenGL3_Init() failed\n";
        }
    }
};

class ImGuiDestroyOperation : public osg::Operation
{
public:
    ImGuiDestroyOperation() : osg::Operation("ImGuiDestroyOperation", false)
    {
    }

    void operator()(osg::Object* object) override
    {
        // ImGui_ImplOpenGL3_Shutdown();
    }
};

using namespace viewer;

StudioViewer::StudioViewer(roadmanager::OpenDrive* odrManager,
                           const char*             modelFilename,
                           const char*             scenarioFilename,
                           const char*             exe_path,
                           osg::ArgumentParser     arguments,
                           SE_Options*             opt)
    : Viewer(odrManager, modelFilename, scenarioFilename, exe_path, arguments, opt, false, false)
{
    data_model_ = nullptr;
    osgViewer_->setRealizeOperation(new ImGuiInitOperation);
    osgViewer_->setCleanUpOperation(new ImGuiDestroyOperation);
    auto* gui   = new StudioGui(this);
    data_model_ = gui->GetModel();
    osgViewer_->addEventHandler(gui);

    osgViewer_->addEventHandler(new StudioViewerEventHandler(this));

    topViewManipulator_                                                  = new osgGA::TopViewManipulator(origin_);
    osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;
    keyswitchManipulator->addMatrixManipulator('\0', "topview", topViewManipulator_.get());
    osgViewer_->setCameraManipulator(keyswitchManipulator.get());
    camMode_ = osgGA::TopViewManipulator::RB_MODE_TOP;
    topViewManipulator_->setMode(static_cast<unsigned int>(osgGA::TopViewManipulator::RB_MODE_TOP));

    auto fov = ORTHO_FOV;

    osg::ref_ptr<osg::GraphicsContext>         gc = osgViewer_->getCamera()->getGraphicsContext();
    osg::ref_ptr<osg::GraphicsContext::Traits> traits =
        const_cast<osg::GraphicsContext::Traits*>(osgViewer_->getCamera()->getGraphicsContext()->getTraits());

    osgViewer_->getCamera()->setProjectionMatrixAsPerspective(fov,
                                                              static_cast<double>(traits->width) / traits->height,
                                                              1.0 * PERSP_FOV / fov,
                                                              1E5 * PERSP_FOV / fov);

    osgViewer_->getCamera()->setLODScale(static_cast<float>(fov / PERSP_FOV));

    osg::BoundingSphere bs = environment_->getBound();
    bs._center[2]          = 0.0;
    topViewManipulator_->SetBound(bs);
}

void StudioViewer::Cleanup()
{
    while (entities_.size() > 0)
    {
        RemoveCar((int)(entities_.size() - 1));
    }
    polyLine_.clear();
    routewaypoints_->removeChildren(0, routewaypoints_->getNumChildren());
    trajectoryLines_->removeChildren(0, trajectoryLines_->getNumChildren());
}

const osg::Vec2d& StudioViewer::GetMousePosition() const
{
    return topViewManipulator_->getMousePos();
}

void StudioViewer::MoveCameraToMapCenter()
{
    auto bs = environment_->getBound();
    MoveCameraTo(bs.center()[0], bs.center()[1]);
}

void StudioViewer::MoveCameraTo(double x, double y)
{
    topViewManipulator_->moveCameraTo(x, y);
}

std::vector<double> StudioViewer::GetCameraSettings()
{
    return topViewManipulator_->getCameraSettings();
}

void StudioViewer::SetCameraSettings(const std::vector<double>& settings)
{
    topViewManipulator_->setCameraSettings(settings);
}

void StudioViewer::BackupCameraSettings()
{
    topViewManipulator_->backupCameraSettings();
}

void StudioViewer::RestoreCameraSettings()
{
    topViewManipulator_->restoreCameraSettings();
}

void StudioViewer::SetCameraDistance(double d)
{
    topViewManipulator_->setCameraDistance(d);
}

void StudioViewer::SetWindowTitle(const std::string& title)
{
    Viewer::SetWindowTitle(title.c_str());
    if (gw_)
        gw_->setWindowName(title.c_str());

#ifdef __linux__
    auto* gw = reinterpret_cast<osgViewer::GraphicsWindowX11*>(gw_);
    if (gw)
    {
        Display*   disp = gw->getDisplayToUse();
        XClassHint clH;
        clH.res_name  = (char*)"scenario_editor";
        clH.res_class = (char*)"WorldSim Scenario Editor";
        XSetClassHint(disp, gw->getWindow(), &clH);
    }
#else
    std::string icon_path = "../resources/storm.ico";
    if (!std::experimental::filesystem::exists(icon_path))
        icon_path = "resources/storm.ico";

    HANDLE hIcon = LoadImage(0, icon_path.c_str(), IMAGE_ICON, 0, 0, LR_DEFAULTSIZE | LR_LOADFROMFILE);
    if (hIcon)
    {
        // Change both icons to the same icon handle.
        HWND hwnd = GetActiveWindow();
        SendMessage(hwnd, WM_SETICON, ICON_SMALL, (LPARAM)hIcon);
        SendMessage(hwnd, WM_SETICON, ICON_BIG, (LPARAM)hIcon);

        // This will ensure that the application icon gets changed too.
        SendMessage(GetWindow(hwnd, GW_OWNER), WM_SETICON, ICON_SMALL, (LPARAM)hIcon);
        SendMessage(GetWindow(hwnd, GW_OWNER), WM_SETICON, ICON_BIG, (LPARAM)hIcon);
    }
#endif
}

bool StudioViewerEventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&)
{
    if (ea.getKey() != osgGA::GUIEventAdapter::KEY_Escape && ea.getHandled())
        return false;

    switch (ea.getEventType())
    {
        case (osgGA::GUIEventAdapter::NONE):
            break;
        case (osgGA::GUIEventAdapter::PUSH):
            break;
        case (osgGA::GUIEventAdapter::RELEASE):
            break;
        case (osgGA::GUIEventAdapter::DOUBLECLICK):
            break;
        case (osgGA::GUIEventAdapter::DRAG):
            break;
        case (osgGA::GUIEventAdapter::MOVE):
            break;
        case (osgGA::GUIEventAdapter::KEYDOWN):
            break;
        case (osgGA::GUIEventAdapter::KEYUP):
            break;
        case (osgGA::GUIEventAdapter::FRAME):
            break;
        case (osgGA::GUIEventAdapter::SCROLL):
            break;
        case (osgGA::GUIEventAdapter::PEN_PRESSURE):
            break;
        case (osgGA::GUIEventAdapter::PEN_ORIENTATION):
            break;
        case (osgGA::GUIEventAdapter::PEN_PROXIMITY_ENTER):
            break;
        case (osgGA::GUIEventAdapter::PEN_PROXIMITY_LEAVE):
            break;
        case (osgGA::GUIEventAdapter::USER):
            break;
        case (osgGA::GUIEventAdapter::RESIZE):
            viewer_->infoTextCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, ea.getWindowWidth(), 0, ea.getWindowHeight()));
            break;
        case (osgGA::GUIEventAdapter::CLOSE_WINDOW):
            viewer_->renderSemaphore.Release();  // no more rendering will happen
            viewer_->SetQuitRequest(true);
            break;
        case (osgGA::GUIEventAdapter::QUIT_APPLICATION):
            viewer_->SetQuitRequest(true);
            break;
    }

    // Send key event to registered callback subscribers
    if (ea.getKey() > 0)
    {
        for (size_t i = 0; i < viewer_->callback_.size(); i++)
        {
            KeyEvent ke = {ea.getKey(), ea.getModKeyMask(), ea.getEventType() & osgGA::GUIEventAdapter::KEYDOWN ? true : false};
            viewer_->callback_[i].func(&ke, viewer_->callback_[i].data);
        }
    }

    if (ea.getKey() == osgGA::GUIEventAdapter::KEY_Space)  // prevent OSG "view reset" action on space key
    {
        return false;
    }
    else
    {
        // forward all other key events to OSG
        return false;
    }
}