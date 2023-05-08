# This is a minimalistic example of how to create build files for an application making use of the esmini shared library.
#
# Make sure that the headerfile esminiLib.hpp and library esminiLib (exact filename depending on platform) is available, either in this folder or in
# the original location (relative this folder).
message(
    STATUS "CMake version: "
           ${CMAKE_VERSION})
cmake_minimum_required(
    VERSION 3.5.0
    FATAL_ERROR)

set(TARGET
    esmini)
set(CMAKE_CXX_STANDARD
    17)
# set(CMAKE_BUILD_TYPE "Debug") set(CMAKE_CXX_FLAGS_DEBUG  "$ENV{CXXFLAGS} -O0 -Wall -g")             # 调试包不优化

project(${TARGET})

set(SCENARIOENGINE_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/ScenarioEngine/SourceFiles"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/ScenarioEngine/OSCTypeDefs")
set(CONTROLLERS_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/Controllers")
set(ROADMANAGER_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/RoadManager")
set(PLAYERBASE_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/PlayerBase")
set(COMMON_MINI_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/CommonMini")
set(PUGIXML_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../externals/pugixml")
set(EXPR_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../externals/expr")

file(
    GLOB
    CONTROLLERS_SOURCES
    "../../Modules/Controllers/*.cpp")

file(
    GLOB
    ROADMANAGER_SOURCES
    "../../Modules/RoadManager/*.cpp")

file(
    GLOB
    PLAYERBASE_SOURCES
    "../../Modules/PlayerBase/*.cpp")

file(
    GLOB
    SCENARIOENGINE_SOURCES
    "../../Modules/ScenarioEngine/SourceFiles/*.[hc]pp"
    "../../Modules/ScenarioEngine/SourceFiles/*.[hc]"
    "../../Modules/ScenarioEngine/OSCTypeDefs/*.[hc]pp")

file(
    GLOB
    COMMONMINI_SOURCES
    "../../Modules/CommonMini/*.cpp")

file(
    GLOB
    PUGIXML_SOURCES
    "../../../externals/pugixml/*.cpp")

list(
    REMOVE_ITEM
    CONTROLLERS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/Controllers/ControllerSumo.cpp")
list(
    REMOVE_ITEM
    CONTROLLERS_INCLUDE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/Controllers/ControllerSumo.hpp")

list(
    REMOVE_ITEM
    SCENARIOENGINE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/ScenarioEngine/SourceFiles/OSIReporter.hpp")
list(
    REMOVE_ITEM
    SCENARIOENGINE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Modules/ScenarioEngine/SourceFiles/OSIReporter.cpp")

include_directories(
    ${SCENARIOENGINE_INCLUDE_DIRS}
    ${CONTROLLERS_INCLUDE_DIR}
    ${ROADMANAGER_INCLUDE_DIR}
    ${PLAYERBASE_INCLUDE_DIR}
    ${COMMON_MINI_INCLUDE_DIR}
    ${PUGIXML_INCLUDE_DIR}
    ${EXPR_INCLUDE_DIR}
    esminijs.hpp)

if(EMSCRIPTEN)
    message("build "${TARGET}.js)
    set(CMAKE_EXECUTABLE_SUFFIX
        ".js")
    add_executable(
        ${TARGET}
        embind.cpp
        esminijs.cpp
        ${CONTROLLERS_SOURCES}
        ${ROADMANAGER_SOURCES}
        ${PLAYERBASE_SOURCES}
        ${SCENARIOENGINE_SOURCES}
        ${COMMONMINI_SOURCES}
        ${PUGIXML_SOURCES})

    set_target_properties(
        ${TARGET}
        PROPERTIES
            LINK_FLAGS
            "\
      --bind \
      -s ENVIRONMENT=web \
      -s MODULARIZE=1 \
      -s EXPORT_NAME=\"esmini\" \
      -s EXPORTED_RUNTIME_METHODS=[\"cwrap\"] \
      -s FORCE_FILESYSTEM=1 \
      -s ALLOW_MEMORY_GROWTH=1 \
      -s WASM_ASYNC_COMPILATION=0 \
      -s WASM=0 \
      -s SINGLE_FILE=1 \
  ")
endif()
