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

#include "Plot.hpp"

// Plot
Plot::Plot(ScenarioEngine* scenarioengine, bool synchronous)
{
    // Save some sizes for easier access later
    plotcategories_size_ = static_cast<size_t>(PlotCategories::Time);
    scenarioengine_      = scenarioengine;

    for (size_t i = 0; i < plotcategories_size_ + 1; i++)
    {
        auto cat                = static_cast<PlotCategories>(i);
        get_category_name_[cat] = lineplot_information_[cat][0];
        get_category_unit_[cat] = lineplot_information_[cat][1];
        (lineplot_information_[cat][2] == "true") ? lineplot_selection_[cat] = true : lineplot_selection_[cat] = false;
    }

    // Populate objects we want to plot and default settings for the checkbox selections
    for (size_t i = 0; i < scenarioengine_->entities_.object_.size(); i++)
    {
        plot_objects_.emplace_back(std::make_unique<PlotObject>(scenarioengine_->entities_.object_[i]));
        (i == 0) ? selected_object_.push_back(true) : selected_object_.push_back(false);
    }

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
    {
        std::cerr << "Something is wrong in IMGUI, cant init!" << std::endl;
    }
    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    glsl_version = "#version 330";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
                                                                    // glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    if (synchronous)
    {
        createImguiWindow();
    }
    else
    {
        init_sem_.Set();
        thread_ = std::thread(&Plot::Thread, this);
        init_sem_.Wait();
    }
}

Plot::~Plot()
{
}

void Plot::CleanUp()
{
    if (window == nullptr)
    {
        return;  // Already closed, or not even created yet
    }

    printf("Closing plot window\n");
#ifdef __EMSCRIPTEN__
    EMSCRIPTEN_MAINLOOP_END;
#endif

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    window = nullptr;
}

void Plot::updateData(std::vector<Object*>& objects, double time)
{
    for (size_t i = 0; i < objects.size(); i++)
    {
        plot_objects_[i]->updateData(objects[i], time);
    }
}

void Plot::plotLine(std::string plot_name, std::string unit, PlotCategories x, PlotCategories y, size_t lineplot_objects)
{
    if (ImPlot::BeginPlot(
            plot_name.c_str(),
            ImVec2(static_cast<float>(window_w) - 200.0f, (static_cast<float>(window_h) - checkbox_padding) / static_cast<float>(lineplot_objects)),
            ImPlotFlags_NoLegend))
    {
        ImPlot::SetupAxes(get_category_name_[x].c_str(), unit.c_str(), x_scaling, y_scaling);
        // For every lineplot, we want to plot x,y data for the selected objects
        for (size_t i = 0; i < selected_object_.size(); i++)
        {
            if (!selected_object_[i])
            {
                continue;
            }
            ImPlot::PlotLine(std::to_string(i).c_str(),
                             plot_objects_[i]->plotData.at(x).data(),
                             plot_objects_[i]->plotData.at(y).data(),
                             static_cast<int>(plot_objects_[i]->plotData.at(x).size()));
        }
        ImPlot::EndPlot();
    }
}

void Plot::adjustPlotDataAxis(const std::pair<const PlotCategories, std::vector<float>>& d, const size_t item)
{
    switch (d.first)
    {
        case (PlotCategories::Time):
        {
            if (!d.second.empty() && d.second.back() > plot_objects_[item]->getTimeMax())
            {
                x_scaling = ImPlotAxisFlags_AutoFit;
            }
            break;
        }
        case (PlotCategories::LatVel):
        {
            ImPlot::SetNextAxesLimits(time_axis_min_, plot_objects_[item]->getTimeMax(), -1.0f, 1.0f);
            break;
        }
        case (PlotCategories::LongVel):
        {
            float min_y_axis = 0.0f;
            if (d.second.back() < min_y_axis)
            {
                min_y_axis = d.second.back();
                y_scaling  = ImPlotAxisFlags_AutoFit;
            }
            ImPlot::SetNextAxesLimits(time_axis_min_, plot_objects_[item]->getTimeMax(), min_y_axis, plot_objects_[item]->getMaxSpeed() + 5.0f);
            break;
        }
        case (PlotCategories::LatA):
        {
            ImPlot::SetNextAxesLimits(time_axis_min_,
                                      plot_objects_[item]->getTimeMax(),
                                      plot_objects_[item]->getMaxDecel(),
                                      plot_objects_[item]->getMaxAcc());
            break;
        }
        case (PlotCategories::LongA):
        {
            ImPlot::SetNextAxesLimits(time_axis_min_,
                                      plot_objects_[item]->getTimeMax(),
                                      plot_objects_[item]->getMaxDecel(),
                                      plot_objects_[item]->getMaxAcc());
            break;
        }
        case (PlotCategories::LaneOffset):
        {
            ImPlot::SetNextAxesLimits(time_axis_min_, plot_objects_[item]->getTimeMax(), -2.5, 2.5);
            break;
        }
        case (PlotCategories::LaneID):
        {
            if (d.second.back() < -5.0f || d.second.back() > 5.0f)
            {
                y_scaling = ImPlotAxisFlags_AutoFit;
            }
            ImPlot::SetNextAxesLimits(time_axis_min_, plot_objects_[item]->getTimeMax(), -5.0, 5.0);
            break;
        }
    }
}

void Plot::renderPlot(const char* name)  //, float window_w, float window_h)
{
    // See how many boxes that has been checked (excl. time), used to scale lineplots
    size_t lineplot_objects = 0;
    for (const auto& selection : lineplot_selection_)
    {
        if (selection.first != PlotCategories::Time && selection.second)
        {
            lineplot_objects += 1;
        }
    }
    ImGui::SetNextWindowSize(ImVec2(static_cast<float>(window_w), static_cast<float>(window_h)), ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once);
    ImGui::Begin(name, nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoTitleBar);

    // Make checkboxes for all objects in the scenario
    for (size_t i = 0; i < plot_objects_.size(); i++)
    {
        std::string checkbox_name = "Object " + std::to_string(i) + " (" + plot_objects_[i]->getName() + ")";
        ImGui::Checkbox(checkbox_name.c_str(), reinterpret_cast<bool*>(&selected_object_[i]));
        if (i < plot_objects_.size() - 1)
        {
            ImGui::SameLine();
        }
    }
    auto store_pos = ImGui::GetCursorPos();

    // Wrap in scope to avoid variable conflict names later
    {
        float y_pos = 10;
        for (const auto& n : get_category_name_)
        {
            if (n.second != "Time")
            {
                ImGui::SetCursorPos(ImVec2(820, y_pos));
                ImGui::Checkbox(n.second.c_str(), &lineplot_selection_[n.first]);
                auto box_size = ImGui::CalcTextSize("Signal 0");
                y_pos += box_size[1] + 10;
            }
        }
    }
    ImGui::SetCursorPos(store_pos);  // Set the cursor back to where we start drawing the lineplots

    // Adjust axes and plotting data for all selected objects in loop below
    for (size_t item = 0; item < selected_object_.size(); item++)
    {
        if (!selected_object_[item])
        {
            continue;
        }
        for (const auto& data : plot_objects_[item]->plotData)
        {
            // Adjust axes
            adjustPlotDataAxis(data, item);

            // Plot (but not time over time or X over X)
            if (data.first == PlotCategories::Time)
            {
                continue;
            }
            else if (lineplot_selection_[data.first])  // Checkbox has to be checked
            {
                plotLine(get_category_name_[data.first], get_category_unit_[data.first], PlotCategories::Time, data.first, lineplot_objects);
            }
            y_scaling = ImPlotAxisFlags_None;
        }
    }
    ImGui::End();
}

int Plot::Frame()
{
    if (window == nullptr)
    {
        return -1;
    }

    if (glfwWindowShouldClose(window) || quit_flag_)
    {
        CleanUp();
        return -1;
    }

    // Poll and handle events (inputs, window resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse
    // data.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the
    // keyboard data. Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    glfwGetWindowSize(window, &window_w, &window_h);

    scenarioengine_->mutex_.Lock();
    updateData(scenarioengine_->entities_.object_, scenarioengine_->getSimulationTime());
    scenarioengine_->mutex_.Unlock();
    renderPlot("Line plot");  //, static_cast<float>(window_w), static_cast<float>(window_h));

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);

    return 0;
}

void Plot::Thread()
{
    static bool initialized = false;

    createImguiWindow();

    while (Frame() == 0)
    {
        if (!initialized)
        {
            initialized = true;
            init_sem_.Release();
        }
    }
}

void Plot::Quit()
{
    quit_flag_ = true;

    if (thread_.joinable())
    {
        thread_.join();
    }
    else
    {
        Frame();
    }
}

void Plot::createImguiWindow()
{
    // Create window with graphics context
    window = glfwCreateWindow(window_width, window_height, "Lineplot", nullptr, nullptr);
    if (window == nullptr)
    {
        std::cerr << "Something is wrong in IMGUI, cant create window!" << std::endl;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // Enable vsync
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    static_cast<void>(io);
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
}

void Plot::glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

// Plot

// PlotObject
Plot::PlotObject::PlotObject(Object* object)
    : time_max_(30.0f),
      max_acc_(static_cast<float>(object->GetMaxAcceleration())),
      max_decel_(static_cast<float>(-object->GetMaxDeceleration())),
      max_speed_(static_cast<float>(object->GetMaxSpeed())),
      name_(object->GetName())
{
}

void Plot::PlotObject::updateData(Object* object, double time)
{
    // Update Time
    plotData[PlotCategories::Time].push_back(static_cast<float>(time));

    // Update Velocity Lat./Long
    double lat_vel, long_vel;
    object->pos_.GetVelLatLong(lat_vel, long_vel);
    plotData[PlotCategories::LatVel].push_back(static_cast<float>(lat_vel));
    plotData[PlotCategories::LongVel].push_back(static_cast<float>(long_vel));

    // Update Lat./Long. Acceleration
    double lat_acc, long_acc;
    object->pos_.GetAccLatLong(lat_acc, long_acc);
    plotData[PlotCategories::LongA].push_back(static_cast<float>(long_acc));
    plotData[PlotCategories::LatA].push_back(static_cast<float>(lat_acc));

    // Update Lane offset
    plotData[PlotCategories::LaneOffset].push_back(static_cast<float>(object->pos_.GetOffset()));

    // Update Lane ID
    plotData[PlotCategories::LaneID].push_back(static_cast<float>(object->pos_.GetLaneId()));
}

float Plot::PlotObject::getTimeMax()
{
    return time_max_;
}
float Plot::PlotObject::getMaxAcc()
{
    return max_acc_;
}
float Plot::PlotObject::getMaxDecel()
{
    return max_decel_;
}
float Plot::PlotObject::getMaxSpeed()
{
    return max_speed_;
}
std::string Plot::PlotObject::getName()
{
    return name_;
}
// PlotObject