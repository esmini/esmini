# ############################### Setting targets ####################################################################

set(TARGET1
    replayer)

set(TARGET2
    dat2csv)

set(TARGET3
    osireceiver)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/system_link.cmake)

# ############################### Setting target files ###############################################################

set(TARGET1_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Replay.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/helpText.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/collision.hpp)

set(TARGET1_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/Replay.hpp)

set(TARGET2_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/dat2csv.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Replay.cpp)

set(TARGET3_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/osi_receiver.cpp)

# ############################### Creating executable for target1 (replayer) #########################################

include_directories(
    ${SCENARIO_ENGINE_PATH}/SourceFiles
    ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
    ${VIEWER_BASE_PATH}
    ${CONTROLLERS_PATH}
    ${COMMON_MINI_PATH})

if(USE_OSG)

    add_executable(
        ${TARGET1}
        ${TARGET1_SOURCES}
        ${TARGET1_INCLUDES})

    target_link_libraries(
        ${TARGET1}
        PRIVATE project_options)

    target_include_system_directories(
        ${TARGET1}
        PRIVATE
        ${ROAD_MANAGER_PATH}
        ${EXTERNALS_OSI_INCLUDES}
        ${EXTERNALS_PUGIXML_PATH}
        ${EXTERNALS_OSG_INCLUDES}
        ${EXTERNALS_DIRENT_INCLUDES})

    target_link_libraries(
        ${TARGET1}
        PRIVATE RoadManager
        PRIVATE ScenarioEngine
        PRIVATE ViewerBase
        PRIVATE CommonMini
        PRIVATE ${TIME_LIB})

    target_link_system_libraries(
        ${TARGET1}
        PRIVATE
        OSG)

    if(USE_OSI)
        target_link_system_libraries(
            ${TARGET1}
            PRIVATE
            OSI)
    endif()

endif()

disable_static_analysis(${TARGET1})
disable_iwyu(${TARGET1})

# ############################### Creating executable for target2 (dat2csv) ##########################################

add_executable(
    ${TARGET2}
    ${TARGET2_SOURCES})

target_link_libraries(
    ${TARGET2}
    PRIVATE project_options)

target_include_system_directories(
    ${TARGET2}
    PRIVATE
    ${ROAD_MANAGER_PATH}
    ${EXTERNALS_OSI_INCLUDES}
    ${EXTERNALS_PUGIXML_PATH}
    ${EXTERNALS_OSG_INCLUDES}
    ${EXTERNALS_DIRENT_INCLUDES})

target_link_libraries(
    ${TARGET2}
    PRIVATE RoadManager
    PRIVATE CommonMini
    PRIVATE ${TIME_LIB})

if(USE_OSI)
    target_link_system_libraries(
        ${TARGET2}
        PRIVATE
        OSI)
endif()

disable_static_analysis(${TARGET2})
disable_iwyu(${TARGET2})

# ############################### Creating executable for target3 (osireceiver) ######################################

if(USE_OSI)

    add_executable(
        ${TARGET3}
        ${TARGET3_SOURCES})

    target_link_libraries(
        ${TARGET3}
        PRIVATE project_options)

    target_include_system_directories(
        ${TARGET3}
        PRIVATE
        ${ROAD_MANAGER_PATH}
        ${EXTERNALS_OSI_INCLUDES}
        ${EXTERNALS_PUGIXML_PATH}
        ${EXTERNALS_OSG_INCLUDES}
        ${EXTERNALS_DIRENT_INCLUDES})

    target_link_libraries(
        ${TARGET3}
        PRIVATE ${TIME_LIB}
        PRIVATE ${SOCK_LIB})

    target_link_system_libraries(
        ${TARGET3}
        PRIVATE
        OSI)

endif()

disable_static_analysis(${TARGET3})
disable_iwyu(${TARGET3})

# # ############################### Install ############################################################################

install(
    TARGETS ${TARGET1}
            ${TARGET2}
            ${TARGET3}
    DESTINATION "${INSTALL_PATH}")
