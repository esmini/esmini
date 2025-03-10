# ############################### Setting targets ####################################################################

set(TARGET
    scenariostudio)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Replay.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Replayer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

set(INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/collision.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Replay.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Replayer.hpp)

# ############################### Creating executable ################################################################

add_executable(
    ${TARGET}
    ${SOURCES}
    ${INCLUDES})

target_include_directories(
    ${TARGET}
    PRIVATE ${SCENARIO_ENGINE_PATH}/SourceFiles
            ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
            ${VIEWER_BASE_PATH}
            ${STUDIO_VIEWER_BASE_PATH}
            ${PLAYER_BASE_PATH}
            ${COMMON_MINI_PATH}
            ${CONTROLLERS_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM
    PUBLIC ${ROAD_MANAGER_PATH}
           ${EXTERNALS_OSI_INCLUDES}
           ${EXTERNALS_PUGIXML_PATH}
           ${EXTERNALS_OSG_INCLUDES}
           ${EXTERNALS_DIRENT_INCLUDES})

target_link_libraries(
    ${TARGET}
    PRIVATE RoadManager
    PRIVATE CommonMini
    PRIVATE ${SPDLOG_LIBRARIES}
    PRIVATE ${TIME_LIB})

if(USE_OSG)
    if(MSVC)
        set_target_properties(
            ${TARGET}
            PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY
                       "$(SolutionDir)")
        target_link_libraries(
            ${TARGET}
            PRIVATE StudioViewerBase
                    PlayerBase
                    ViewerBase
                    ScenarioEngine
                    CommonMini
                    Controllers
                    RoadManager
                    ${OSG_LIBRARIES}
                    ${SUMO_LIBRARIES}
                    ${IMPLOT_LIBRARIES}
                    ${SOCK_LIB})
    else()
        target_link_libraries(
            ${TARGET}
            PRIVATE StudioViewerBase
                    PlayerBase
                    ViewerBase
                    ScenarioEngine
                    CommonMini
                    Controllers
                    RoadManager
                    ${OSG_LIBRARIES}
                    ${SUMO_LIBRARIES}
                    ${IMPLOT_LIBRARIES}
                    stdc++fs)
    endif()
endif()

if(USE_OSI)
    target_link_libraries(
        ${TARGET}
        PRIVATE ${OSI_LIBRARIES})
endif()

disable_static_analysis(${TARGET})
disable_iwyu(${TARGET})

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET}
    DESTINATION "${INSTALL_PATH}")
