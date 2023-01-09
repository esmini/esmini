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

include_directories(
    ${COMMON_MINI_PATH}
    ${CONTROLLERS_PATH}
    ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
    ${SCENARIO_ENGINE_PATH}/SourceFiles)

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
    PRIVATE
    ${ROAD_MANAGER_PATH}
    ${EXTERNALS_SUMO_INCLUDES}
    ${EXTERNALS_PUGIXML_PATH})

disable_static_analysis(${TARGET})
disable_iwyu(${TARGET})
enable_fpic(${TARGET})
