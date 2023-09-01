#include "Plot.hpp"

// Plot
Plot::Plot(ScenarioEngine* scenarioengine, bool synchronous)
{
    // Save some sizes for easier access later
    plotcategories_size_ = static_cast<size_t>(PlotCategories::Time);
    scenarioengine_      = scenarioengine;
    bool_array_size_     = scenarioengine_->entities_.object_.size();

    // Populate objects we want to plot and default settings for the checkbox selections
    for (size_t i = 0; i < scenarioengine_->entities_.object_.size(); i++)
    {
        plotObjects.emplace_back(std::make_unique<PlotObject>(scenarioengine_->entities_.object_[i]));
        (i == 0) ? selectedItem.push_back(true) : selectedItem.push_back(false);
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
    // delete[] selectedItem;
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
        plotObjects[i]->updateData(objects[i], time);
    }
}

void Plot::renderPlot(const char* name, float window_w, float window_h)
{
    std::string plot_name = "";
    std::string unit      = "";

    // See how many boxes that has been checked (excl. time), used to scale lineplots
    size_t lineplot_objects = 0;
    for (const auto& sel : lineplot_selection)
    {
        if (sel.first != PlotCategories::Time && sel.second)
        {
            lineplot_objects += 1;
        }
    }
    ImGui::SetNextWindowSize(ImVec2(window_w, window_h), ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once);
    ImGui::Begin(name, nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoTitleBar);

    // Make checkboxes for all objects in the scenario
    for (size_t i = 0; i < bool_array_size_; i++)
    {
        std::string checkbox_name = "Object " + std::to_string(i) + " (" + plotObjects[i]->getName() + ")";
        ImGui::Checkbox(checkbox_name.c_str(), reinterpret_cast<bool*>(&selectedItem[i]));
        if (i < bool_array_size_ - 1)
        {
            ImGui::SameLine();
        }
    }
    auto store_pos = ImGui::GetCursorPos();

    // Wrap in scope to avoid variable conflict names later
    {
        float y_pos = 10;
        // int i = 0;
        for (const auto& n : getCategoryName)
        {
            if (n.second != "Time")
            {
                ImGui::SetCursorPos(ImVec2(820, y_pos));
                ImGui::Checkbox(n.second.c_str(), &lineplot_selection[n.first]);
                auto box_size = ImGui::CalcTextSize("Signal 0");
                y_pos += box_size[1] + 10;
                // i++;
            }
        }
    }
    ImGui::SetCursorPos(store_pos);  // Set the cursor back to where we start drawing the lineplots

    // Check which ones is currently selected and recently updated
    for (size_t i = 0; i < bool_array_size_; i++)
    {
        if (selectedItem[i] && i != selection)
        {
            selection = static_cast<unsigned int>(i);
            break;
        }
    }

    // Set all not-recently checked boxes to false, only allow 1 checked box at a time (for now)
    for (size_t j = 0; j < bool_array_size_; j++)
    {
        if (j != selection)
        {
            selectedItem[j] = false;
        }
    }

    for (const auto& d : plotObjects[selection]->plotData)
    {
        // Adjust axes
        switch (d.first)
        {
            case (PlotCategories::Time):
            {
                plot_name = getCategoryName[PlotCategories::Time];
                if (!d.second.empty() && d.second.back() > plotObjects[selection]->getTimeMax())
                {
                    x_scaling = ImPlotAxisFlags_AutoFit;
                }
                break;
            }
            case (PlotCategories::LatVel):
            {
                plot_name = getCategoryName[PlotCategories::LatVel];
                unit      = "m/s";
                ImPlot::SetNextAxesLimits(-5.0f, plotObjects[selection]->getTimeMax(), -1.0f, 1.0f);
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
                plot_name = getCategoryName[PlotCategories::LongVel];
                unit      = "m/s";
                ImPlot::SetNextAxesLimits(-5.0f, plotObjects[selection]->getTimeMax(), min_y_axis, plotObjects[selection]->getMaxSpeed() + 5.0f);
                break;
            }
            case (PlotCategories::LatA):
            {
                plot_name = getCategoryName[PlotCategories::LatA];
                unit      = "m/s²";
                ImPlot::SetNextAxesLimits(-5.0f,
                                          plotObjects[selection]->getTimeMax(),
                                          plotObjects[selection]->getMaxDecel(),
                                          plotObjects[selection]->getMaxAcc());
                break;
            }
            case (PlotCategories::LongA):
            {
                plot_name = getCategoryName[PlotCategories::LongA];
                unit      = "m/s²";
                ImPlot::SetNextAxesLimits(-5.0f,
                                          plotObjects[selection]->getTimeMax(),
                                          plotObjects[selection]->getMaxDecel(),
                                          plotObjects[selection]->getMaxAcc());
                break;
            }
            case (PlotCategories::LaneOffset):
            {
                plot_name = getCategoryName[PlotCategories::LaneOffset];
                unit      = "m";
                ImPlot::SetNextAxesLimits(-5.0f, plotObjects[selection]->getTimeMax(), -2.5, 2.5);
                break;
            }
            case (PlotCategories::LaneID):
            {
                plot_name = getCategoryName[PlotCategories::LaneID];
                unit      = "id";
                if (d.second.back() < -5.0f || d.second.back() > 5.0f)
                {
                    y_scaling = ImPlotAxisFlags_AutoFit;
                }
                ImPlot::SetNextAxesLimits(-5.0f, plotObjects[selection]->getTimeMax(), -5.0, 5.0);
                break;
            }
        }

        // Plot (but not time over time or X over X)
        if (d.first == PlotCategories::Time)
        {
            continue;
        }
        else if (lineplot_selection[d.first])
        {
            if (ImPlot::BeginPlot(plot_name.c_str(),
                                  ImVec2(window_w - 200, (window_h - checkbox_padding) / static_cast<float>(lineplot_objects)),
                                  ImPlotFlags_NoLegend))
            {
                ImPlot::SetupAxes("Time [s]", unit.c_str(), x_scaling, y_scaling);
                ImPlot::PlotLine(std::to_string(selection).c_str(),
                                 plotObjects[selection]->plotData.at(PlotCategories::Time).data(),
                                 d.second.data(),
                                 static_cast<int>(plotObjects[selection]->plotData.at(PlotCategories::Time).size()));
                ImPlot::EndPlot();
            }
        }
        y_scaling = ImPlotAxisFlags_None;
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
    int window_w, window_h;
    glfwGetWindowSize(window, &window_w, &window_h);

    scenarioengine_->mutex_.Lock();
    updateData(scenarioengine_->entities_.object_, scenarioengine_->getSimulationTime());
    scenarioengine_->mutex_.Unlock();
    renderPlot("Line plot", static_cast<float>(window_w), static_cast<float>(window_h));

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