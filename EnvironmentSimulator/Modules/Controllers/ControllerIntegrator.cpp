#include "ControllerIntegrator.hpp"

#include <dlfcn.h>
#include <filesystem>
#include <iostream>

namespace scenarioengine::controller
{

ControllerIntegrator::ControllerIntegrator(const std::string& path)
: path_(path)
{
}

std::string ControllerIntegrator::GetControllerNameFromFile(const std::string& fileName) const
{
    if( size_t pos =  fileName.find("lib"); pos != std::string::npos && fileName.size() > 3)
    {
        return fileName.substr(3, fileName.size() - 3);
    }
    else
    {
        return fileName;
    }
}

std::vector<std::pair<std::string, ControllerInitiazer>> ControllerIntegrator::LoadControllersInitializers() const
{
    std::vector<std::pair<std::string, ControllerInitiazer>> initializers;
    for (const auto& entry : std::filesystem::directory_iterator(path_))
    {
        if ( ".so" == entry.path().extension() )
        {
            //++soLibrariesCount;
            std::string libPath = entry.path();
            //std::cout << "\n.so file found " << libPath << '\n';
            if( auto initializer = GetControllerInitializerFromLib(libPath); initializer.has_value())
            {
                auto controllerName = GetControllerNameFromFile(entry.path().stem().string());
                initializers.emplace_back( std::make_pair(controllerName, initializer.value()) );
            }
            else
            {
                //log error
            }
        }    
    }
    return initializers;
}


std::optional<ControllerInitiazer> ControllerIntegrator::GetControllerInitializerFromLib(const std::string& path) const
{
    void * libHandle = dlopen(path.c_str() , RTLD_LAZY);  
    if( libHandle == NULL)
    {
        //log error
        //std::cout << "unable to open lib error : " << dlerror() << '\n';
    }
    else
    {
        ControllerInitiazer controllerInitiazer;
        *(void**) (&controllerInitiazer) = dlsym(libHandle, "GetController");
        if( controllerInitiazer == nullptr)
        {
            //log error
            //std::cout << "unable to find GetController error : " << dlerror() << '\n';
        }
        else
        {
            return controllerInitiazer;
        }
    }
    return std::nullopt;
}

}   //namespace scenarioengine::controller