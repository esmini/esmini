#pragma once

#include "Controller.hpp"

#include <string>
#include <vector>
#include <optional>

namespace scenarioengine::controller
{

using ControllerInitiazer = BaseController* (*) ();

class ControllerIntegrator
{
public:
    // Ctor takes path of controller libs folder
    ControllerIntegrator(const std::string& path);
    
    std::vector<std::pair<std::string, ControllerInitiazer>> LoadControllersInitializers() const;

private:

    std::string GetControllerNameFromFile(const std::string& fileName) const;
    // Try to get controller from a shared/dynamic libs
    std::optional<ControllerInitiazer> GetControllerInitializerFromLib(const std::string& libPath) const;
    // Path of controller libs folder
    std::string path_;    
};    

} // namespace scenarioengine::controller

