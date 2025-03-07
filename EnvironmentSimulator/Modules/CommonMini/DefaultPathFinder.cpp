#include "DefaultPathFinder.hpp"

#include "logger.hpp"

#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#include <dlfcn.h>
#elif defined(__linux__)
#include <dlfcn.h>
#include <unistd.h>
#include <limits.h>
#endif

#include <stdexcept>
#include <iostream>

extern "C" std::string GetLibraryPath()
{
#if defined(_WIN32)
    char    path[MAX_PATH];
    HMODULE hModule = nullptr;

    // Get the handle of the current module (library)
    if (!GetModuleHandleEx(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS, (LPCTSTR)GetLibraryPath, &hModule))
    {
        LOG_ERROR("Failed to get library handle.");
        return "";
    }

    // Get the full path of the library
    if (GetModuleFileName(hModule, path, MAX_PATH) == 0)
    {
        LOG_ERROR("Failed to get library path.");
        return "";
    }
    LOG_DEBUG("Library path: {}", path);
    std::string strPath(path);
    return strPath;

#else

    Dl_info dl_info;

    // Use dladdr to retrieve the path of the loaded library
    if (dladdr((void*)&GetLibraryPath, &dl_info) == 0)
    {
        throw std::runtime_error("Failed to get library path.");
    }

    return dl_info.dli_fname;

#endif
}

namespace esmini::common
{

    std::string DefaultPathFinder::GetDefaultPath()
    {
        char buffer[1024];
        try
        {
#if defined(_WIN32) || defined(__CYGWIN__)

            DWORD length = GetModuleFileNameA(nullptr, buffer, sizeof(buffer));
            if (length == 0 || length == sizeof(buffer))
            {
                throw std::runtime_error("Failed to get executable path on Windows");
            }
            LOG_INFO("Application path: {}", std::string(buffer));
            std::string libraryPath = GetLibraryPath();
            if (libraryPath.empty())
            {
                LOG_INFO("Unable to find the path func");
            }
            else
            {
                LOG_INFO("Library path: {}", libraryPath);
            }
#elif defined(__linux__)
            ssize_t length = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
            if (length == -1)
            {
                throw std::runtime_error("Failed to get executable path on Linux");
            }
            buffer[length] = '\0';  // Null-terminate the string

            LOG_INFO("Application path: {}", std::string(buffer));

            std::string libraryPath = GetLibraryPath();
            LOG_INFO("Library path: {}", libraryPath);

#elif defined(__APPLE__)

            uint32_t size = sizeof(buffer);
            if (_NSGetExecutablePath(buffer, &size) != 0)
            {
                throw std::runtime_error("Buffer size too small for executable path on macOS");
            }
            LOG_INFO("Application path: {}", std::string(buffer));
            std::string libraryPath = GetLibraryPath();
            LOG_INFO("Library path: {}", libraryPath);
#else
            throw std::runtime_error("Unsupported platform");
#endif

            // return std::string(buffer);
            return libraryPath;
        }
        catch (const std::exception& e)
        {
            LOG_ERROR("Error while getting default path: {}", e.what());
        }
        return "";
    }
}  // namespace esmini::common