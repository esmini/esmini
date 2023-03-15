include_guard()

set(CMAKE_CXX_STANDARD
    17)
set(CMAKE_CXX_STANDARD_REQUIRED
    ON)
set(CMAKE_CXX_EXTENSIONS
    ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS
    ON)
set(CMAKE_VERBOSE_MAKEFILE
    ON)
set(EXPLICIT_FOLDER_STRUCTURE
    ON)

# ---------------------------------------------------
if(MSVC)
    set(CMAKE_VS_INCLUDE_INSTALL_TO_DEFAULT_BUILD
        1)
elseif(MINGW)

endif()
# ---------------------------------------------------

# Enhance error reporting and compiler messages
if(ENABLE_COLORED_DIAGNOSTICS)
    if(CMAKE_CXX_COMPILER_ID
       MATCHES
       ".*Clang")
        add_compile_options(-fcolor-diagnostics)
    elseif(
        CMAKE_CXX_COMPILER_ID
        STREQUAL
        "GNU")
        add_compile_options(-fdiagnostics-color=always)
    else()
        message(STATUS "No colored compiler diagnostic set for '${CMAKE_CXX_COMPILER_ID}' compiler.")
    endif()
endif()

include(${CMAKE_CURRENT_LIST_DIR}/standards.cmake)
set_standards()

add_library(
    project_options
    INTERFACE)

target_compile_features(
    project_options
    INTERFACE cxx_std_${CMAKE_CXX_STANDARD})

include(${CMAKE_CURRENT_LIST_DIR}/compiler_warnings.cmake)
set_project_warnings(
    project_options
    ${ENABLE_WARNINGS_AS_ERRORS})

include(${CMAKE_CURRENT_LIST_DIR}/ccache.cmake)
if(ENABLE_CCACHE)
    enable_ccache()
endif()

# include(${CMAKE_CURRENT_LIST_DIR}/coverage.cmake) if(ENABLE_COVERAGE) enable_coverage(project_options) endif()

include(${CMAKE_CURRENT_LIST_DIR}/sanitizers.cmake)
if(ENABLE_SANITIZERS)
    enable_sanitizers(project_options)
endif()

# include(${CMAKE_CURRENT_LIST_DIR}/static_analysis.cmake) if(ENABLE_STATIC_ANALYSIS) enable_static_analysis() endif()

# include(${CMAKE_CURRENT_LIST_DIR}/iwyu.cmake) if(ENABLE_INCLUDE_WHAT_YOU_USE) enable_iwyu() endif()

# include(${CMAKE_CURRENT_LIST_DIR}/valgrind.cmake) enable_valgrind()
