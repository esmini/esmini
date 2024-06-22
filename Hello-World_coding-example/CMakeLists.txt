# This is a minimalistic example of how to create build files for an application
# making use of the esmini shared library.
#
# Make sure that the headerfile esminiLib.hpp and library esminiLib
# (exact filename depending on platform) is available, either in this folder or in
# the original location (relative this folder).

cmake_minimum_required(VERSION 3.7.1)

set(TARGET esmini-player)
set(LINK_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/../bin)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../EnvironmentSimulator/Libraries/esminiLib)
link_directories(${LINK_DIRECTORIES})

project(${TARGET})

# Windows: Uncomment next line to completely suppress the console
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")

add_executable(${TARGET} main.cpp)

# embed $origin (location of exe file) and link dirs as execution dyn lib search paths
set_target_properties(${TARGET}
     PROPERTIES
     BUILD_WITH_INSTALL_RPATH true
     INSTALL_RPATH "$ORIGIN:${LINK_DIRECTORIES}"
)

target_link_libraries(${TARGET} esminiLib)
