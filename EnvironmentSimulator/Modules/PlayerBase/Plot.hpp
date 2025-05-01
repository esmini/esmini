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

#include "imgui.h"
#include "implot.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "ScenarioEngine.hpp"
#include <stdio.h>
#include <vector>
#include <unordered_map>
#include <thread>
#include <future>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>  // Will drag system OpenGL headers

// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to maximize ease of testing and compatibility with old VS compilers.
// To link with VS2010-era libraries, VS2015+ requires linking with legacy_stdio_definitions.lib, which we do using this pragma.
// Your own project should not be affected, as you are likely to link with a newer binary of GLFW that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

#ifndef PLOT_H
#define PLOT_H

using namespace scenarioengine;

enum class PlotCategories
{
    LatVel = 0,
    LongVel,
    LatA,
    LongA,
    LaneOffset,
    LaneID,
    Time,
};  // Keep Time last, used as reference for size of the enum

class Plot
{
public:
    Plot(ScenarioEngine* scenarioengine, bool synchronous = false);
    ~Plot();
    int  Frame();
    void Thread();
    void CleanUp();
    void Quit();
    bool IsModeSynchronuous()
    {
        return !thread_.joinable();
    }

    void        updateData(std::vector<Object*>& objects, double dt);
    void        renderPlot(const char* name);  //, float window_width, float window_height);
    void        adjustPlotDataAxis(const std::pair<const PlotCategories, std::vector<float>>& d, const size_t item);
    void        adjustSelectedObjectsPlotDataAxis(const PlotCategories& y_category);
    void        createImguiWindow();
    static void glfw_error_callback(int error, const char* description);

    SE_Semaphore init_sem_;

private:
    class PlotObject
    {
    public:
        // PlotObject(float max_acc, float max_decel, float max_speed);
        PlotObject(Object* object);
        void updateData(Object* object, double time);

        // Getters
        float       getTimeMax();
        float       getMaxAcc();
        float       getMaxDecel();
        float       getMaxSpeed();
        std::string getName();

        // Data
        std::unordered_map<PlotCategories, std::vector<float>> plotData{};

    private:
        // Constants
        const float       time_max_  = {};
        const float       max_acc_   = {};
        const float       max_decel_ = {};
        const float       max_speed_ = {};
        const std::string name_      = {};
    };
    // GLFW, glsl
    const char* glsl_version = nullptr;
    GLFWwindow* window       = nullptr;
    // Imgui window
    int window_width  = 1000;
    int window_height = 1000;
    // Dynamic window
    int         window_w         = {};
    int         window_h         = {};
    const float checkbox_padding = 55.0;
    bool        quit_flag_       = false;

    // Settings
    ImPlotAxisFlags x_scaling = ImPlotAxisFlags_None;
    ImPlotAxisFlags y_scaling = ImPlotAxisFlags_None;

    // Plot Variables
    std::vector<std::unique_ptr<PlotObject>>        plot_objects_       = {};
    std::unordered_map<PlotCategories, std::string> get_category_name_  = {};
    std::unordered_map<PlotCategories, std::string> get_category_unit_  = {};
    std::unordered_map<PlotCategories, bool>        lineplot_selection_ = {};
    // Add any new data below
    std::unordered_map<PlotCategories, std::vector<std::string>> lineplot_information_ = {
        {PlotCategories::LatVel, std::vector<std::string>{"LatVel", "m/s", "false"}},
        {PlotCategories::LongVel, std::vector<std::string>{"LongVel", "m/s", "true"}},
        {PlotCategories::LatA, std::vector<std::string>{"LatA", "m/s²", "false"}},
        {PlotCategories::LongA, std::vector<std::string>{"LongA", "m/s²", "false"}},
        {PlotCategories::LaneOffset, std::vector<std::string>{"LaneOffset", "LaneOffset", "false"}},
        {PlotCategories::LaneID, std::vector<std::string>{"LaneID", "id", "false"}},
        {PlotCategories::Time, std::vector<std::string>{"Time", "s", "false"}}};

    size_t            plotcategories_size_ = {};
    std::vector<char> selected_object_     = {};
    float             time_axis_min_       = -5.0f;

    // Runtime variables
    ScenarioEngine* scenarioengine_;
    bool            initialized_ = false;
    std::thread     thread_;
};

#endif  // PLOT_H