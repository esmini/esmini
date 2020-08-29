cmake_minimum_required (VERSION 3.5.0 FATAL_ERROR)

set(TARGET esmini-player)

include_directories(.)
link_directories(.)

project(${TARGET})

add_executable(${TARGET} main.cpp)

target_link_libraries(${TARGET} ScenarioEngineDLL)

install(TARGETS ${TARGET} DESTINATION ${CMAKE_HOME_DIRECTORY}/.)