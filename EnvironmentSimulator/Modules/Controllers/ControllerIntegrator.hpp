#pragma once

#include "Controller.hpp"

#include <string>
#include <vector>
#include <optional>

namespace scenarioengine::controller
{

    using ControllerInitializer = ControllerBase* (*)(void*);

    class ControllerIntegrator
    {
    public:
        // Destructor
        ~ControllerIntegrator();
        // Sets the paths to folders in which integrated controllers will be searched
        void SetPathsToSearchControllers(std::vector<std::string> pathsToSearchControllers);
        // Loads multiple integrated controllers in the path(s) to search
        [[nodiscard]] std::vector<std::pair<std::string, ControllerInitializer>> LoadControllersInitializers();
        // Loads one integrated controller in the path sent in parameter
        [[nodiscard]] std::optional<std::pair<std::string, ControllerInitializer>> LoadSpecificController(const std::string& path);

    private:
        // Utility function to obtain controller name fromm names of .so/.dll file
        std::string GetControllerNameFromFile(const std::string& fileName) const;
        // Try to get controller from a shared/dynamic lib
        std::optional<ControllerInitializer> GetControllerInitializerFromLib(const std::string& libPath);
        // Paths to search controller libs in
        std::vector<std::string> pathsToSearchControllers_;
        // Handles to all loaded libs
        std::vector<void*> libHandles;
    };

}  // namespace scenarioengine::controller
