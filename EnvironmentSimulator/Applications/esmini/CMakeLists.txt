# ############################### Setting targets ####################################################################

set(TARGET
    esmini)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/system_link.cmake)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

# ############################### Creating executable ################################################################

include_directories(
    ${SCENARIO_ENGINE_PATH}/SourceFiles
    ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
    ${VIEWER_BASE_PATH}
    ${PLAYER_BASE_PATH}
    ${CONTROLLERS_PATH}
    ${COMMON_MINI_PATH})

add_executable(
    ${TARGET}
    ${SOURCES})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options)

target_include_system_directories(
    ${TARGET}
    PRIVATE
    ${ROAD_MANAGER_PATH}
    ${EXTERNALS_OSI_INCLUDES}
    ${EXTERNALS_PUGIXML_PATH}
    ${EXTERNALS_OSG_INCLUDES}
    ${EXTERNALS_SUMO_INCLUDES})

target_link_libraries(
    ${TARGET}
    PRIVATE PlayerBase
    PRIVATE ScenarioEngine
    PRIVATE CommonMini
    PRIVATE Controllers
    PRIVATE RoadManager
    PRIVATE ${TIME_LIB}
    PRIVATE ${SOCK_LIB})

if(USE_OSG)
    target_link_libraries(
        ${TARGET}
        PRIVATE ViewerBase)

    target_link_system_libraries(
        ${TARGET}
        PRIVATE
        OSG)
endif()

if(USE_SUMO)
    target_link_system_libraries(
        ${TARGET}
        PRIVATE
        ${SUMO_LIBRARIES})
endif()

if(USE_OSI)
    target_link_system_libraries(
        ${TARGET}
        PRIVATE
        OSI)
endif()

disable_static_analysis(${TARGET})
disable_iwyu(${TARGET})

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET}
    DESTINATION "${INSTALL_PATH}")
