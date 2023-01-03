# This is a minimalistic example of how to create build files for an application
# making use of the esmini shared library.
#
# Make sure that the headerfile esminiLib.hpp and library esminiLib
# (exact filename depending on platform) is available, either in this folder or in
# the original location (relative this folder).

set(TARGET esmini-player)

include_directories(. ${CMAKE_CURRENT_SOURCE_DIR}/../EnvironmentSimulator/Libraries/esminiLib)
link_directories(. ${CMAKE_CURRENT_SOURCE_DIR}/../bin)

project(${TARGET})

# Windows: Uncomment next line to completely suppress the console
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")

add_executable(${TARGET} main.cpp)

target_link_libraries(${TARGET} esminiLib)
