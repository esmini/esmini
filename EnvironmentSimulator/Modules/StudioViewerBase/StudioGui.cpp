#include "StudioGui.hpp"
#include <iostream>
#include <experimental/filesystem>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>

#ifdef __linux__
#include <X11/Xlib.h>
#include <osgViewer/api/X11/GraphicsWindowX11>
#endif

#include "CommonMini.hpp"
#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_impl_opengl3.h"

int menu_bar_height = 18;
int time_bar_height = 36;

namespace
{
    struct ImGuiNewFrameCallback : public osg::Camera::DrawCallback
    {
        ImGuiNewFrameCallback(StudioGui& handler) : handler_(handler)
        {
        }

        void operator()(osg::RenderInfo& renderInfo) const override
        {
            handler_.newFrame(renderInfo);
        }

    private:
        StudioGui& handler_;
    };

    struct ImGuiRenderCallback : public osg::Camera::DrawCallback
    {
        ImGuiRenderCallback(StudioGui& handler) : handler_(handler)
        {
        }

        void operator()(osg::RenderInfo& renderInfo) const override
        {
            handler_.render(renderInfo);
        }

    private:
        StudioGui& handler_;
    };

}  // anonymous namespace

StudioGui::StudioGui(viewer::StudioViewer* viewer) : time_(0.0f), mouse_wheel_(0.0f), initialized_(false), viewer_(viewer)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io    = ImGui::GetIO();
    io.IniFilename = NULL;
    io.LogFilename = NULL;

    viewer_->gw_->setCursor(osgViewer::GraphicsWindow::LeftArrowCursor);
    // ImGui::SetMouseCursor(ImGuiMouseCursor_Arrow);
}

StudioGui::~StudioGui()
{
}

void StudioGui::Exit()
{
    data_model_.mode_ = StudioMode::ZOMBIE;
    viewer_->osgViewer_->setDone(true);
}

/**
 * Imporant Note: Dear ImGui expects the control Keys indices not to be
 * greater thant 511. It actually uses an array of 512 elements. However,
 * OSG has indices greater than that. So here I do a conversion for special
 * keys between ImGui and OSG.
 */
static int ConvertOSGKeyToImGuiKey(int key)
{
    using KEY                 = osgGA::GUIEventAdapter::KeySymbol;
    std::map<KEY, ImGuiKey> m = {
        {KEY::KEY_Tab, ImGuiKey_Tab},
        {KEY::KEY_Left, ImGuiKey_LeftArrow},
        {KEY::KEY_Right, ImGuiKey_RightArrow},
        {KEY::KEY_Up, ImGuiKey_UpArrow},
        {KEY::KEY_Down, ImGuiKey_DownArrow},
        {KEY::KEY_Page_Up, ImGuiKey_PageUp},
        {KEY::KEY_Page_Down, ImGuiKey_PageDown},
        {KEY::KEY_Home, ImGuiKey_Home},
        {KEY::KEY_End, ImGuiKey_End},
        {KEY::KEY_Delete, ImGuiKey_Delete},
        {KEY::KEY_BackSpace, ImGuiKey_Backspace},
        {KEY::KEY_Return, ImGuiKey_Enter},
        {KEY::KEY_Escape, ImGuiKey_Escape},
    };
    if (m.count((KEY)key) != 0)
        return m.at((KEY)key);
    return -1;
}

void StudioGui::newFrame(osg::RenderInfo& renderInfo)
{
    ImGui_ImplOpenGL3_NewFrame();

    ImGuiIO& io = ImGui::GetIO();

    osg::Viewport* viewport = renderInfo.getCurrentCamera()->getViewport();
    io.DisplaySize          = ImVec2(viewport->width(), viewport->height());
    viewport_width_         = viewport->width();
    viewport_height_        = viewport->height();

    double currentTime = renderInfo.getView()->getFrameStamp()->getSimulationTime();
    io.DeltaTime       = currentTime - time_ + 0.00001;
    time_              = currentTime;

    io.MouseDown[0] = left_mouse_pressed_;
    io.MouseDown[1] = right_mouse_pressed_;
    io.MouseDown[2] = middle_mouse_pressed_;

    io.MouseWheel = mouse_wheel_;
    mouse_wheel_  = 0.0f;

    ImGui::NewFrame();
}

void StudioGui::render(osg::RenderInfo&)
{
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Exit"))
                Exit();
            ImGui::EndMenu();
        }
    }
    ImGui::EndMainMenuBar();

    ImGui::SetNextWindowPos(ImVec2(0, viewport_height_ - time_bar_height));
    ImGui::SetNextWindowSize((ImVec2(viewport_width_, time_bar_height)));
    ImGui::SetNextWindowBgAlpha(1.0f);
    ImGui::Begin("##timebar", nullptr, ImGuiWindowFlags_NoDecoration);
    ImGui::PushItemWidth(-1);
    data_model_.virtual_time_max_value_ = std::max(data_model_.virtual_time_, data_model_.virtual_time_max_value_);
    if (ImGui::SliderFloat("##virtual_time",
                           &data_model_.virtual_time_,
                           0.0f,
                           data_model_.virtual_time_max_value_,
                           "Virtual Time: %.2f s",
                           ImGuiSliderFlags_NoInput))
    {
        data_model_.virtual_time_             = std::round(data_model_.virtual_time_ / 0.05) * 0.05;
        data_model_.virtual_time_manipulated_ = true;
    }
    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

bool StudioGui::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if (!initialized_)
    {
        auto* osg_view = aa.asView();
        if (osg_view)
        {
            auto* camera = osg_view->getCamera();
            camera->addPreDrawCallback(new ImGuiNewFrameCallback(*this));
            camera->addPostDrawCallback(new ImGuiRenderCallback(*this));
            initialized_ = true;
        }
    }

    ImGuiIO&   io                  = ImGui::GetIO();
    const bool wantCaptureMouse    = io.WantCaptureMouse;
    const bool wantCaptureKeyboard = io.WantCaptureKeyboard;

    // static int old_e = 0, old_m = 0;
    // int e = ea.getEventType();
    // int m = ea.getButtonMask();
    // if ((e != old_e || m != old_m) && e != 0x80)
    //   LOG("event: %X, mask: %X", e, m);
    // if (e != 0x80) {
    //   old_e = e;
    //   old_m = m;
    // }

    switch (ea.getEventType())
    {
        case osgGA::GUIEventAdapter::KEYDOWN:
        case osgGA::GUIEventAdapter::KEYUP:
        {
            bool isKeyDown   = ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN;
            int  c           = ea.getKey();
            int  special_key = ConvertOSGKeyToImGuiKey(c);
            auto mod         = ea.getModKeyMask();

            if (mod & osgGA::GUIEventAdapter::MODKEY_CTRL)
                io.AddKeyEvent(ImGuiMod_Ctrl, isKeyDown);
            else
                io.AddKeyEvent(ImGuiMod_Ctrl, false);

            if (special_key > 0)
            {
                bool escape_handled = false;
                if (special_key == ImGuiKey_Escape)
                {
                }
                if (!escape_handled)
                    io.AddKeyEvent((ImGuiKey)(special_key), isKeyDown);
            }
            else if (isKeyDown && c > 0 && c < 0xFF)
            {
                io.AddInputCharacter((unsigned short)c);
            }

            // LOG_INFO("keydown %d %d mod:%d", c, special_key, (int)mod);
            return wantCaptureKeyboard;
        }
        case (osgGA::GUIEventAdapter::RELEASE):
        case (osgGA::GUIEventAdapter::PUSH):
        {
            io.MousePos           = ImVec2(ea.getX(), io.DisplaySize.y - ea.getY());
            left_mouse_pressed_   = ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON;
            right_mouse_pressed_  = ea.getButtonMask() & osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON;
            middle_mouse_pressed_ = ea.getButtonMask() & osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON;
            return wantCaptureMouse;
        }
        case (osgGA::GUIEventAdapter::DRAG):
        case (osgGA::GUIEventAdapter::MOVE):
        {
            io.MousePos = ImVec2(ea.getX(), io.DisplaySize.y - ea.getY());
            return wantCaptureMouse;
        }
        default:
        {
            return false;
        }
    }

    return false;
}
