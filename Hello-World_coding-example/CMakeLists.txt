# This is a minimalistic example of how to create build files for an application 
# making use of the esmini shared library.
# 
# Make sure that the headerfile esminiLib.hpp and library esminiLib 
# (exact filename depending on platform) is available, either in this folder or in 
# the original location (relative this folder).

cmake_minimum_required (VERSION 3.5.0 FATAL_ERROR)

set(TARGET esmini-player)

include_directories(. ../include ../EnvironmentSimulator/Libraries/esminiLib)
link_directories(. ../lib ../bin)

project(${TARGET})

add_executable(${TARGET} main.cpp)

target_link_libraries(${TARGET} esminiLib)

# Copy the executable to the main folder
install(TARGETS ${TARGET} DESTINATION ${CMAKE_HOME_DIRECTORY}/.)

