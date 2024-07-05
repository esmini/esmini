#include "ControllerIntegrator.hpp"

#include <dlfcn.h>
#include <iostream>

#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#error "Missing <filesystem> header"
#endif

namespace scenarioengine::controller
{
    ControllerIntegrator::~ControllerIntegrator()
    {
        for (auto& libHandle : libHandles)
        {
            dlclose(libHandle);
        }
    }

    void ControllerIntegrator::SetPathsToSearchControllers(std::vector<std::string> pathsToSearchControllers)
    {
        pathsToSearchControllers_ = std::move(pathsToSearchControllers);
    }

    std::string ControllerIntegrator::GetControllerNameFromFile(const std::string& fileName) const
    {
        if (size_t pos = fileName.find("lib"); pos != std::string::npos && fileName.size() > 3)
        {
            return fileName.substr(3, fileName.size() - 3);
        }
        else
        {
            return fileName;
        }
    }

    std::vector<std::pair<std::string, ControllerInitializer>> ControllerIntegrator::LoadControllersInitializers()
    {
        std::vector<std::pair<std::string, ControllerInitializer>> initializers;
        for (const auto& path : pathsToSearchControllers_)
        {
            std::cout << "Searching for integrated controllers in : " << path << '\n';
            if (fs::exists(path))
            {
                for (const auto& entry : fs::directory_iterator(path))
                {
                    if (".so" == entry.path().extension())
                    {
                        std::string libPath = entry.path();
                        if (auto initializer = GetControllerInitializerFromLib(libPath); initializer.has_value())
                        {
                            auto controllerName = GetControllerNameFromFile(entry.path().stem().string());
                            initializers.emplace_back(std::make_pair(controllerName, initializer.value()));
                        }
                    }
                }
            }
            if (!initializers.empty())  // only one folder is taken as source for controller libs
            {
                break;
            }
        }
        return initializers;
    }

    std::optional<ControllerInitializer> ControllerIntegrator::GetControllerInitializerFromLib(const std::string& path)
    {
        void* libHandle = dlopen(path.c_str(), RTLD_LAZY);
        if (libHandle == NULL)
        {
            std::cout << path << " unable to open, error : " << dlerror() << '\n';
        }
        else
        {
            ControllerInitializer controllerInitializer;
            *reinterpret_cast<void**>(&controllerInitializer) = dlsym(libHandle, "InstantiateController");
            if (controllerInitializer == nullptr)
            {
                dlclose(libHandle);
                std::cout << path << " is loaded but couldn't find InstantiateController function, error: " << dlerror() << '\n';
            }
            else
            {
                libHandles.emplace_back(libHandle);
                return controllerInitializer;
            }
        }
        return std::nullopt;
    }

}  // namespace scenarioengine::controller