# ############################### Setting targets ####################################################################

set(TARGET
    esmini)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

# ############################### Creating executable ################################################################

if(USE_OSG)
    set (VIEWER_BASE ViewerBase)
endif()

add_executable(
    ${TARGET}
    ${SOURCES})

target_include_directories(
    ${TARGET}
    PRIVATE
    ${ROAD_MANAGER_PATH}
    ${SCENARIO_ENGINE_PATH}/SourceFiles
    ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
    ${VIEWER_BASE_PATH}
    ${PLAYER_BASE_PATH}
    ${CONTROLLERS_PATH}
    ${COMMON_MINI_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM PUBLIC
    ${EXTERNALS_OSI_INCLUDES}
    ${EXTERNALS_PUGIXML_PATH}
    ${EXTERNALS_OSG_INCLUDES}
    ${EXTERNALS_SUMO_INCLUDES})

target_link_libraries(
    ${TARGET}
    PRIVATE
    project_options
    PlayerBase
    ScenarioEngine
    CommonMini
    Controllers
    RoadManager
    ${VIEWER_BASE}
    ${OSI_LIBRARIES}
    ${SUMO_LIBRARIES}
    ${TIME_LIB}
    ${SOCK_LIB})

if(USE_OSG)
target_link_libraries(
    ${TARGET}
    PRIVATE
    ViewerBase
    ${OSG_LIBRARIES})
endif()

disable_static_analysis(${TARGET})
disable_iwyu(${TARGET})

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET}
    DESTINATION "${INSTALL_PATH}")
