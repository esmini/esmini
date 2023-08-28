#include "imgui.h"
#include "implot.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "Entities.hpp"
#include <stdio.h>
#include <vector>
#include <random>
#include <thread>
#include <chrono>
#include <tuple>
#include <unordered_map>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to maximize ease of testing and compatibility with old VS compilers.
// To link with VS2010-era libraries, VS2015+ requires linking with legacy_stdio_definitions.lib, which we do using this pragma.
// Your own project should not be affected, as you are likely to link with a newer binary of GLFW that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

#ifndef PLOT_H
#define PLOT_H

using namespace scenarioengine;

enum class PlotCategories {
    LatVel = 0,
    LongVel,
    LatA,
    LongA,
    LaneOffset,
    LaneID,
    Time,
}; // Keep Time last, used as reference for size of the enum


class Plot {
    public:
        Plot(std::vector<Object*>& objects);
        ~Plot();
        void CleanUp();

        void updateData(std::vector<Object*>& objects, double dt);
        void renderPlot(const char* name, float window_width, float window_height);
        void renderImguiWindow();
        static void glfw_error_callback(int error, const char* description);

    private:
        class PlotObject {
            public:
                PlotObject(float max_acc, float max_decel, float max_speed);

                void updateData(Object* object, double dt);

                // Getters
                float getTimeMax();
                float getMaxAcc();
                float getMaxDecel();
                float getMaxSpeed();

                // Data
                std::unordered_map<PlotCategories, std::vector<float>> plotData{};

            private:
                // Constants
                const float time_max_;
                const float max_acc_;
                const float max_decel_;
                const float max_speed_;

        };
        // GLFW, glsl
        const char* glsl_version{};
        GLFWwindow* window;
        int window_width = 1000;
        int window_height = 1000;
        const float checkbox_padding = 55.0;

        // Variables
        std::vector<std::unique_ptr<PlotObject>> plotObjects;

        // Settings
        ImPlotAxisFlags x_scaling = ImPlotAxisFlags_None;
        ImPlotAxisFlags y_scaling = ImPlotAxisFlags_None;

        // Object selection
        size_t bool_array_size_ = {};
        size_t plotcategories_size_ = {};
        std::vector<char> selectedItem = {};
        unsigned int selection = 0;
        std::vector<char> lineplot_selection = {};
};

#endif // PLOT_H