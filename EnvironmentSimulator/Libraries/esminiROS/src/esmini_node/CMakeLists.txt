cmake_minimum_required(VERSION 3.7.1)

project(esmini)

set(CMAKE_CXX_STANDARD
    17)

if(${CMAKE_SYSTEM_NAME}
   MATCHES
   "Linux")
    set(LINUX
        true)
else()
    set(LINUX
        false)
endif()

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/locations.cmake)
set_project_internal_paths()
set_project_external_paths()
set_project_os_specific_paths()
set_project_includes()
set_project_library_paths()

include(${SUPPORT_PATH}/cmake/common/definitions.cmake)
set_definitions()

# ############################### Downloading cloud packages #########################################################

include(${SUPPORT_PATH}/cmake/cloud/set_cloud_links.cmake)
set_cloud_links()

include(${SUPPORT_PATH}/cmake/cloud/download.cmake)

message(STATUS "Downloading dependencies...")

download(
    osg
    ${EXTERNALS_OSG_PATH}
    ${EXTERNALS_OSG_OS_SPECIFIC_PATH}
    "*"
    "${OSG_PACKAGE_URL}")

download(
    osi
    ${EXTERNALS_OSI_PATH}
    ${EXTERNALS_OSI_OS_SPECIFIC_PATH}
    "*"
    "${OSI_PACKAGE_URL}")

download(
    sumo
    ${EXTERNALS_SUMO_PATH}
    ${EXTERNALS_SUMO_OS_SPECIFIC_PATH}
    "*"
    "${SUMO_PACKAGE_URL}")

download(
    implot
    ${EXTERNALS_IMPLOT_PATH}
    ${EXTERNALS_IMPLOT_OS_SPECIFIC_PATH}
    "*"
    "${IMPLOT_PACKAGE_URL}")

download(
    models
    ${RESOURCES_PATH}
    ${MODELS_PATH}
    "models"
    "${MODELS_PACKAGE_URL}")

find_package(
    catkin
    REQUIRED
    COMPONENTS message_generation
               roscpp
               roslib
               geometry_msgs)

# Generate message
add_message_files(
    DIRECTORY
    msg)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    roslib
    message_runtime)

# cpp files
file(
    GLOB
    ESMINI_SOURCES
    "${COMMON_MINI_PATH}/*.cpp"
    "${CONTROLLERS_PATH}/*.cpp"
    "${PLAYER_BASE_PATH}/*.cpp"
    "${ROAD_MANAGER_PATH}/*.cpp"
    "${SCENARIO_ENGINE_PATH}/OSCTypeDefs/*.cpp"
    "${SCENARIO_ENGINE_PATH}/SourceFiles/*.cpp"
    "${SCENARIO_ENGINE_PATH}/SourceFiles/*.c"
    "${ESMINI_LIB_PATH}/*.cpp"
    "${ESMINI_LIB_PATH}/*.cs"
    "${EXTERNALS_PUGIXML_PATH}/*.cpp")

if(USE_IMPLOT)
    include(${SUPPORT_PATH}/cmake/external/implot.cmake)
    set_implot_libs()
    include_directories(${EXTERNALS_IMPLOT_INCLUDES})
else()
    list(
        REMOVE_ITEM
        ESMINI_SOURCES
        "${PLAYER_BASE_PATH}/Plot.cpp")
    list(
        REMOVE_ITEM
        PLAYER_BASE_PATH
        "${PLAYER_BASE_PATH}/Plot.hpp")
endif()

if(USE_OSG)
    include(${SUPPORT_PATH}/cmake/external/osg.cmake)
    set_osg_libs()
    include_directories(
        ${VIEWER_BASE_PATH}
        ${EXTERNALS_OSG_INCLUDES})
    file(
        GLOB
        VIEWER_SOURCES
        "${VIEWER_BASE_PATH}/*.cpp")

    if(NOT
       USE_OSI)
        list(
            REMOVE_ITEM
            VIEWER_BASE_PATH
            "${VIEWER_BASE_PATH}/OSISensorView.hpp")
        list(
            REMOVE_ITEM
            VIEWER_SOURCES
            "${VIEWER_BASE_PATH}/OSISensorView.cpp")
    endif()

    list(
        APPEND
        ESMINI_SOURCES
        ${VIEWER_SOURCES})
endif()

if(USE_OSI)
    include(${SUPPORT_PATH}/cmake/external/osi.cmake)
    set_osi_libs()
    include_directories(${EXTERNALS_OSI_INCLUDES})
else()
    list(
        REMOVE_ITEM
        ESMINI_SOURCES
        "${SCENARIO_ENGINE_PATH}/SourceFiles/OSIReporter.cpp"
        "${SCENARIO_ENGINE_PATH}/SourceFiles/OSITrafficCommand.cpp")
    list(
        REMOVE_ITEM
        SCENARIO_ENGINE_PATH
        "${SCENARIO_ENGINE_PATH}/SourceFiles/OSIReporter.hpp"
        "${SCENARIO_ENGINE_PATH}/SourceFiles/OSITrafficCommand.hpp")
endif()

if(USE_SUMO)
    include(${SUPPORT_PATH}/cmake/external/sumo.cmake)
    set_sumo_libs()
    include_directories(${EXTERNALS_SUMO_INCLUDES})
else()
    list(
        REMOVE_ITEM
        ESMINI_SOURCES
        "${CONTROLLERS_PATH}/ControllerSumo.cpp")
    list(
        REMOVE_ITEM
        CONTROLLERS_PATH
        "${CONTROLLERS_PATH}/ControllerSumo.hpp")
endif()

include_directories(
    include
    ${COMMON_MINI_PATH}
    ${CONTROLLERS_PATH}
    ${PLAYER_BASE_PATH}
    ${ROAD_MANAGER_PATH}
    ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
    ${SCENARIO_ENGINE_PATH}/SourceFiles
    ${EXTERNALS_SPDLOG_INCLUDES}
    ${EXTERNALS_PUGIXML_PATH}
    ${EXTERNALS_EXPR_PATH}
    ${ESMINI_LIB_PATH}
    ${catkin_INCLUDE_DIRS})

add_executable(
    esmini_ros_node
    src/main.cpp
    src/esmini_node.cpp
    ${ESMINI_SOURCES})
target_link_libraries(
    esmini_ros_node
    ${catkin_LIBRARIES})

add_executable(
    clock_publisher
    src/clock_publisher.cpp)
target_link_libraries(
    clock_publisher
    ${catkin_LIBRARIES})
add_dependencies(
    clock_publisher
    ${catkin_EXPORTED_TARGETS})

if(USE_IMPLOT)
    target_link_libraries(
        esmini_ros_node
        ${IMPLOT_LIBRARIES})
endif()

if(USE_OSG)
    target_link_libraries(
        esmini_ros_node
        ${OSG_LIBRARIES})
endif()

if(USE_OSI)
    target_link_libraries(
        esmini_ros_node
        ${OSI_LIBRARIES})
endif()

if(USE_SUMO)
    target_link_libraries(
        esmini_ros_node
        ${SUMO_LIBRARIES})
endif()

# add lib for missing filesystem functions in case g++ 7.5 is used
target_link_libraries(
    esmini_ros_node
    "stdc++fs")

add_dependencies(
    esmini_ros_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})

install(
    TARGETS esmini_ros_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
