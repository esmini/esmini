#include "ControllerIntegrator.hpp"

// #include <dlfcn.h>
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

namespace scenarioengine::controller
{
    ControllerIntegrator::~ControllerIntegrator()
    {
        for (auto& libHandle : libHandles)
        {
            CLOSE_LIBRARY(libHandle);
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
            std::cout << "Searching integrated controllers in: " << path << '\n';
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
            std::cout << path << " unable to open, error : " << dlerror() << '\n';
        }
        else
        {
            ControllerInitializer controllerInitializer;
            //*reinterpret_cast<void**>(&controllerInitializer) = dlsym(libHandle, "InstantiateController");
            *reinterpret_cast<void**>(&controllerInitializer) = GET_FUNCTION(libHandle, "InstantiateController");
            if (controllerInitializer == nullptr)
            {
                CLOSE_LIBRARY(libHandle);
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