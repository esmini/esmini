# ############################### Setting targets ####################################################################

set(TARGET1
    replayer)

set(TARGET2
    osireceiver)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################

set(TARGET1_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Replay.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/helpText.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/collision.hpp
    ${SCENARIO_ENGINE_PATH}/SourceFiles/DatLogger.cpp)

set(TARGET1_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/Replay.hpp)

set(TARGET2_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/osi_receiver.cpp)

# ############################### Creating executable for target1 (replayer) #########################################

if(USE_OSG)

    add_executable(
        ${TARGET1}
        ${TARGET1_SOURCES}
        ${TARGET1_INCLUDES})

    target_link_libraries(
        ${TARGET1}
        PRIVATE project_options)

    target_include_directories(
        ${TARGET1}
        PRIVATE ${SCENARIO_ENGINE_PATH}/SourceFiles
                ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
                ${VIEWER_BASE_PATH}
                ${CONTROLLERS_PATH}
                ${COMMON_MINI_PATH})

    target_include_directories(
        ${TARGET1}
        SYSTEM
        PUBLIC ${ROAD_MANAGER_PATH}
               ${EXTERNALS_OSI_INCLUDES}
               ${EXTERNALS_PUGIXML_PATH}
               ${EXTERNALS_OSG_INCLUDES}
               ${EXTERNALS_DIRENT_INCLUDES})

    target_link_libraries(
        ${TARGET1}
        PRIVATE RoadManager
                ScenarioEngine
                CommonMini
                ${TIME_LIB})

    if(USE_OSG)
        target_link_libraries(
            ${TARGET1}
            PRIVATE ViewerBase
                    ${OSG_LIBRARIES})
    endif()

    if(USE_OSI)
        target_link_libraries(
            ${TARGET1}
            PRIVATE ${OSI_LIBRARIES})
    endif()

    disable_static_analysis(${TARGET1})
    disable_iwyu(${TARGET1})

    install(
        TARGETS ${TARGET1}
        DESTINATION "${INSTALL_PATH}")

endif()

# ############################### Creating executable for TARGET2 (osireceiver) ######################################

if(USE_OSI)

    add_executable(
        ${TARGET2}
        ${TARGET2_SOURCES})

    target_include_directories(
        ${TARGET2}
        SYSTEM
        PUBLIC ${ROAD_MANAGER_PATH}
               ${EXTERNALS_OSI_INCLUDES}
               ${EXTERNALS_DIRENT_INCLUDES})

    target_link_libraries(
        ${TARGET2}
        PRIVATE project_options
                ${TIME_LIB}
                ${SOCK_LIB}
                ${OSI_LIBRARIES})

    disable_static_analysis(${TARGET2})
    disable_iwyu(${TARGET2})

    install(
        TARGETS ${TARGET2}
        DESTINATION "${INSTALL_PATH}")

endif()
