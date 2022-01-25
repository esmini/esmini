cmake_minimum_required (VERSION 3.5.0 FATAL_ERROR)

set(TARGET AZtest)

project(${TARGET})

set (COMMON_MINI_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../EnvironmentSimulator/Modules/CommonMini")
set (ROADMANAGER_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../EnvironmentSimulator/Modules/RoadManager")

add_subdirectory(${COMMON_MINI_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/build/commonMini)
add_subdirectory(${ROADMANAGER_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/build/roadManager)

include_directories (
  ${COMMON_MINI_INCLUDE_DIR}
  ${ROADMANAGER_INCLUDE_DIR}  
)

add_executable(${TARGET} main.cpp)

target_link_libraries(${TARGET} RoadManager CommonMini pthread)


