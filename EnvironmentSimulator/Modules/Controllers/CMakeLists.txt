# ############################### Setting targets ####################################################################

set(TARGET
    Controllers)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/enable_fpic.cmake)

# ############################### Setting target files ###############################################################

file(
    GLOB
    CONTROLLERS_SOURCES
    "*.cpp")

file(
    GLOB
    CONTROLLERS_INCLUDES
    "*.hpp")

if(NOT
   USE_SUMO)
    list(
        REMOVE_ITEM
        CONTROLLERS_SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/ControllerSumo.cpp")
    list(
        REMOVE_ITEM
        CONTROLLERS_INCLUDES
        "${CMAKE_CURRENT_LIST_DIR}/ControllerSumo.hpp")
endif()

# ############################### Creating library ###################################################################

add_library(
    ${TARGET}
    STATIC
    ${CONTROLLERS_SOURCES}
    ${CONTROLLERS_INCLUDES})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options)

target_include_directories(
    ${TARGET}
    PRIVATE ${COMMON_MINI_PATH}
            ${CONTROLLERS_PATH}
            ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
            ${SCENARIO_ENGINE_PATH}/SourceFiles
            ${VIEWER_BASE_PATH}
            ${PLAYER_BASE_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM
    PUBLIC ${ROAD_MANAGER_PATH}
           ${EXTERNALS_OSI_INCLUDES}
           ${EXTERNALS_SUMO_INCLUDES}
           ${EXTERNALS_PUGIXML_PATH}
           ${EXTERNALS_OSG_INCLUDES}
           ${EXTERNALS_SPDLOG_INCLUDES})

disable_static_analysis(${TARGET})
disable_iwyu(${TARGET})
enable_fpic(${TARGET})
