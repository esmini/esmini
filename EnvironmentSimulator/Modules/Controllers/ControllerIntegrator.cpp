#include "ControllerIntegrator.hpp"

#include <iostream>
// include filesystem from C++17
#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
#error "Missing <filesystem> header"
#endif

// includes to use with dynamic libs on supported platforms
#if defined(_WIN32)
#include <windows.h>
typedef HMODULE LibHandle;
typedef FARPROC FuncHandle;
#define LOAD_LIBRARY(name)      LoadLibrary(TEXT(name))
#define GET_FUNCTION(lib, func) GetProcAddress(lib, func)
#define CLOSE_LIBRARY(lib)      FreeLibrary(lib)
#define LIB_EXTENSION           ".dll"
#elif defined(__linux__) || defined(__APPLE__)
#include <dlfcn.h>
typedef void* LibHandle;
typedef void* FuncHandle;
#define LOAD_LIBRARY(name)      dlopen(name, RTLD_LAZY)
#define GET_FUNCTION(lib, func) dlsym(lib, func)
#define CLOSE_LIBRARY(lib)      dlclose(lib)
#else
#error "Unsupported platform"
#endif

#if defined(__linux__)
#define LIB_EXTENSION ".so"
#elif defined(__APPLE__)
#define LIB_EXTENSION ".dylib"
#endif

#include "CommonMini.hpp"  // for log

namespace scenarioengine::controller
{
    ControllerIntegrator::~ControllerIntegrator()
    {
        // close all the handles of loaded libs
        for (auto& libHandle : libHandles)
        {
            CLOSE_LIBRARY(libHandle);
        }
    }

    void ControllerIntegrator::SetPathsToSearchControllers(std::vector<std::string> pathsToSearchControllers)
    {
        pathsToSearchControllers_ = std::move(pathsToSearchControllers);
    }

    std::string ControllerIntegrator::GetControllerNameFromFile(const std::string& filePath) const
    {
        size_t slashPos            = filePath.find_last_of('/');
        size_t extensionPos        = filePath.find(LIB_EXTENSION);
        size_t controllerNameStart = slashPos == std::string::npos ? 0 : slashPos + 1;
        size_t controllerNameSize  = extensionPos == std::string::npos ? filePath.size() : extensionPos - controllerNameStart;

        std::string fileNameWithoutExtension = filePath.substr(controllerNameStart, controllerNameSize);
        // on Linux and Mac, dynamic libs are prefixed with 'lib', stripping it to get actual name of the lib; we will use it as name of integrated
        // controller
        if (size_t pos = fileNameWithoutExtension.find("lib"); pos != std::string::npos && fileNameWithoutExtension.size() > 3)
        {
            return fileNameWithoutExtension.substr(3, fileNameWithoutExtension.size() - 3);
        }
        else
        {
            return fileNameWithoutExtension;
        }
    }

    [[nodiscard]] std::optional<std::pair<std::string, ControllerInitializer>> ControllerIntegrator::LoadSpecificController(const std::string& path)
    {
        std::optional<std::pair<std::string, ControllerInitializer>> controller = std::nullopt;
        if (fs::exists(path))
        {
            if (auto initializer = GetControllerInitializerFromLib(path); initializer.has_value())
            {
                auto controllerName = GetControllerNameFromFile(path);
                controller          = std::make_pair(controllerName, initializer.value());
            }
        }
        else
        {
            LOG("Controllers integrator couldn't load specific controller as the given path does't exist: %s", path.c_str());
        }
        return controller;
    }

    [[nodiscard]] std::vector<std::pair<std::string, ControllerInitializer>> ControllerIntegrator::LoadControllersInitializers()
    {
        std::vector<std::pair<std::string, ControllerInitializer>> initializers;
        for (const auto& path : pathsToSearchControllers_)
        {
            LOG("Controllers integrator searching in: %s", path.c_str());
            if (fs::exists(path))
            {
                for (const auto& entry : fs::directory_iterator(path))
                {
                    // if (".so" == entry.path().extension())
                    if (LIB_EXTENSION == entry.path().extension())
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
        void* libHandle = LOAD_LIBRARY(path.c_str());
        if (libHandle == NULL)
        {
            LOG("Controllers integrator unable to open lib error: %s", dlerror());
        }
        else
        {
            ControllerInitializer controllerInitializer;
            *reinterpret_cast<void**>(&controllerInitializer) = GET_FUNCTION(libHandle, "InstantiateController");
            if (controllerInitializer == nullptr)
            {
                CLOSE_LIBRARY(libHandle);
                LOG("Controllers integrator loaded lib but couldn't find InstantiateController function: %s", dlerror());
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