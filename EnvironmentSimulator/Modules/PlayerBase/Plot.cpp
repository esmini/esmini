#include "Plot.hpp"

Plot::Plot(std::vector<scenarioengine::Object*> objects)
{
    // Save some sizes for easier access later
    plotcategories_size_ = static_cast<size_t>(PlotCategories::Time);
    bool_array_size_ = objects.size();

    // Populate objects we want to plot and default settings for the checkbox selections
    for (size_t i = 0; i < objects.size(); i++)
    {
        plotObjects.emplace_back(std::make_unique<PlotObject>(objects[i]->GetMaxAcceleration(), objects[i]->GetMaxDeceleration(), objects[i]->GetMaxSpeed()));
        (i == 0) ? selectedItem.push_back(true) : selectedItem.push_back(false);
    }
    // Set default values for lineplot checkboxes
    for (size_t i = 0; i < plotcategories_size_; i++)
    {
        lineplot_selection.push_back(true);
    }

    glfwSetErrorCallback(glfw_error_callback);
        if (!glfwInit())
            std::cerr << "Something is wrong in IMGUI, cant init!" << std::endl;

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
}

Plot::~Plot()
{
    // delete[] selectedItem;
}

void Plot::CleanUp()
{
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
}

void Plot::updateData(std::vector<Object*> objects, double dt)
{
    for (size_t i = 0; i < objects.size(); i++)
    {
        plotObjects[i]->updateData(objects[i], dt);
    }
}

void Plot::renderPlot(const char* name, float window_w, float window_h)
{
    std::string plot_name = "";
    int lineplot_objects = plotObjects[0]->plotData.size() - 1; // Time has no own plot
    ImGui::SetNextWindowSize(ImVec2(window_w, window_h), ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once);
    ImGui::Begin(name, nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoTitleBar);

    // Make checkboxes
    for (size_t i = 0; i < bool_array_size_; i++)
    {
        ImGui::Checkbox(("Object " + std::to_string(i)).c_str(), reinterpret_cast<bool*>(&selectedItem[i]));
        if (i < bool_array_size_ - 1)
        {
            ImGui::SameLine();
        }
    }
    auto store_pos = ImGui::GetCursorPos();

    float y_pos = 10;
    for (size_t i = 0; i < lineplot_selection.size(); i++)
    {
        ImGui::SetCursorPos(ImVec2(820,y_pos));
        ImGui::Checkbox(("Signal " + std::to_string(i)).c_str(), reinterpret_cast<bool*>(&lineplot_selection[i]));
        auto box_size = ImGui::CalcTextSize("Signal 0");
        y_pos += box_size[1] + 10;
    }
    ImGui::SetCursorPos(store_pos); // Set the cursor back to where we start drawing the lineplots

    // Check which ones is currently selected and recently updated
    for (size_t i = 0; i < bool_array_size_; i++)
    {
        if (selectedItem[i] && i != selection)
        {
            selection = i;
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
    // ImGui::End();
    // ImGui::Begin("Scrollable");
    // ImGui::SetNextWindowSize(ImVec2(window_width, window_height), ImGuiCond_Once);
    // ImGui::BeginChild("ScrollingRegion", ImVec2(window_width, window_height), false, ImGuiWindowFlags_HorizontalScrollbar);

    for (const auto& d : plotObjects[selection]->plotData)
    {
        // Adjust axes
        switch(d.first)
        {
            case(PlotCategories::Time):
            {
                plot_name = "Time";
                if (!d.second.empty() && d.second.back() > plotObjects[selection]->getTimeMax())
                {
                    x_scaling = ImPlotAxisFlags_AutoFit;
                }
                break;
            }
            case(PlotCategories::LatVel):
            {
                plot_name = "Lateral Velocity";
                ImPlot::SetNextAxesLimits(-5.0f, plotObjects[selection]->getTimeMax(), -1.0f, 1.0f);
                break;
            }
            case(PlotCategories::LongVel):
            {
                plot_name = "Longitudinal Velocity";
                ImPlot::SetNextAxesLimits(-5.0f, plotObjects[selection]->getTimeMax(), -1.0, plotObjects[selection]->getMaxSpeed() + 5.0f);
                break;
            }
            case(PlotCategories::LatA):
            {
                plot_name = "LatA";
                ImPlot::SetNextAxesLimits(-5.0f, plotObjects[selection]->getTimeMax(), plotObjects[selection]->getMaxDecel(), plotObjects[selection]->getMaxAcc());
                break;
            }
            case(PlotCategories::LongA):
            {
                plot_name = "LongA";
                ImPlot::SetNextAxesLimits(-5.0f, plotObjects[selection]->getTimeMax(), plotObjects[selection]->getMaxDecel(), plotObjects[selection]->getMaxAcc());
                break;
            }
            case(PlotCategories::LaneOffset):
            {
                plot_name = "Offset from current lane";
                // lineplot_selection = false;
                ImPlot::SetNextAxesLimits(-5.0f, plotObjects[selection]->getTimeMax(), -2.5, 2.5);
                break;
            }
            case(PlotCategories::LaneID):
            {
                plot_name = "Lane ID";
                // lineplot_selection = false;
                float y_values = abs(d.second.back());
                ImPlot::SetNextAxesLimits(-5.0f, plotObjects[selection]->getTimeMax(), -y_values - 1, y_values + 1);
                break;
            }
        }


        // Plot (but not time over time or X over X)
        if (d.first == PlotCategories::Time || !lineplot_selection[0])
        {
            lineplot_selection[0] = true;
            continue;
        }
        else
        {
            ImPlot::BeginPlot(plot_name.c_str(), ImVec2(window_w - 200, (window_h - checkbox_padding) / lineplot_objects));
            ImPlot::SetupAxes("x", "y", x_scaling, y_scaling);
            ImPlot::PlotLine(("Object " + std::to_string(selection)).c_str(), plotObjects[selection]->plotData.at(PlotCategories::Time).data(), d.second.data(), static_cast<int>(plotObjects[selection]->plotData.at(PlotCategories::Time).size()));
            ImPlot::EndPlot();
        }
    // ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle);
    }
    // ImGui::EndChild();
    ImGui::End();
}

void Plot::renderImguiWindow()
{
    // Create window with graphics context
    window = glfwCreateWindow(window_width, window_height, "Lineplot", nullptr, nullptr);
    if (window == nullptr)
        std::cerr << "Something is wrong in IMGUI, cant create window!" << std::endl;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); 
    static_cast<void>(io);
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Our state
    while (!glfwWindowShouldClose(window)) 
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        int window_w, window_h;
        glfwGetWindowSize(window, &window_w, &window_h);
        renderPlot("Line plot", static_cast<float>(window_w), static_cast<float>(window_h));

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    CleanUp();
}

void Plot::glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

Plot::PlotObject::PlotObject(float max_acc, float max_decel, float max_speed) : 
time_max_(30.0f), 
max_acc_(max_acc), 
max_decel_(-max_decel), 
max_speed_(max_speed) 
{}

void Plot::PlotObject::updateData(Object* object, double dt) 
{
    // Update Time
    (plotData[PlotCategories::Time].empty()) ? plotData[PlotCategories::Time].push_back(static_cast<float>(dt)) : plotData[PlotCategories::Time].push_back(plotData[PlotCategories::Time].back() + static_cast<float>(dt));
    
    // Update Velocity Lat./Long
    double lat_vel, long_vel;
    object->pos_.GetVelLatLong(lat_vel, long_vel);
    plotData[PlotCategories::LatVel].push_back(static_cast<float>(lat_vel));
    plotData[PlotCategories::LongVel].push_back(static_cast<float>(long_vel));
    
    // Update offset
    plotData[PlotCategories::LaneOffset].push_back(static_cast<float>(object->pos_.GetOffset()));

    // Update Lat./Long. Acceleration
    double lat_acc, long_acc;
    object->pos_.GetAccLatLong(lat_acc, long_acc);
    plotData[PlotCategories::LongA].push_back(static_cast<float>(long_acc));
    plotData[PlotCategories::LatA].push_back(static_cast<float>(lat_acc));

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