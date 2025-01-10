#pragma once

#include <string>

// Export symbol for shared library (cross-platform)
#if defined(_WIN32) || defined(__CYGWIN__)
#ifdef BUILDING_LIBPATH
#define LIBPATH_API __declspec(dllexport)
#else
#define LIBPATH_API __declspec(dllimport)
#endif
#else
#define LIBPATH_API __attribute__((visibility("default")))
#endif

// Function to get the library's disk path
extern "C" LIBPATH_API std::string GetLibraryPath();

namespace esmini::common
{
    class DefaultPathFinder
    {
    public:
        std::string GetDefaultPath();

    private:
    };
}  // namespace esmini::common