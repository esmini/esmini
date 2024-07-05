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
        //
        std::vector<std::pair<std::string, ControllerInitializer>> LoadControllersInitializers();

    private:
        std::string GetControllerNameFromFile(const std::string& fileName) const;
        // Try to get controller from a shared/dynamic libs
        std::optional<ControllerInitializer> GetControllerInitializerFromLib(const std::string& libPath);
        // Paths to search controller libs in
        std::vector<std::string> pathsToSearchControllers_;
        // Handles to all loaded libs
        std::vector<void*> libHandles;
    };

}  // namespace scenarioengine::controller
