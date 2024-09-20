# ############################### Setting targets ###################################################################################################

set(TARGET_SHARED
    esminiLib)

set(TARGET_STATIC
    esminiLib_static)

# ############################### Loading desired rules ##############################################################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/esminiLib.cpp
    ${CONTROLLERS_PATH}/vehicle.cpp)

set(INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/esminiLib.hpp
    ${CONTROLLERS_PATH}/vehicle.hpp)

# ############################### Creating library ##################################################################################################

add_library(
    ${TARGET_SHARED}
    SHARED
    ${SOURCES}
    ${INCLUDES})

# embed $origin (location of exe file) as execution dyn lib search paths
set(RPATH_DIRS
    "$ORIGIN")

if(DYN_PROTOBUF)
    # add OSI library folder to execution lib search paths
    set(RPATH_DIRS
        ${RPATH_DIRS}:${EXTERNALS_OSI_LIBRARY_PATH}/$<IF:$<CONFIG:Debug>,debug,release>)
endif()

set_target_properties(
    ${TARGET_SHARED}
    PROPERTIES BUILD_WITH_INSTALL_RPATH
               true
               INSTALL_RPATH
               "${RPATH_DIRS}")

add_library(
    ${TARGET_STATIC}
    STATIC
    ${SOURCES}
    ${INCLUDES})

foreach(
    TARGET
    IN
    ITEMS ${TARGET_SHARED}
          ${TARGET_STATIC})

    target_link_libraries(
        ${TARGET}
        PRIVATE project_options)

    target_include_directories(
        ${TARGET}
        PRIVATE ${SCENARIO_ENGINE_PATH}/SourceFiles
                ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
                ${VIEWER_BASE_PATH}
                ${PLAYER_BASE_PATH}
                ${COMMON_MINI_PATH}
                ${CONTROLLERS_PATH})

    target_include_directories(
        ${TARGET}
        SYSTEM
        PUBLIC ${ROAD_MANAGER_PATH}
               ${EXTERNALS_OSI_INCLUDES}
               ${EXTERNALS_OSG_INCLUDES}
               ${EXTERNALS_PUGIXML_PATH}
               ${EXTERNALS_SPDLOG_INCLUDES})

    target_link_libraries(
        ${TARGET}
        PRIVATE PlayerBase
                ScenarioEngine
                CommonMini
                Controllers
                RoadManager
                ${SPDLOG_LIBRARIES}
                ${TIME_LIB}
                ${SOCK_LIB})

    if(USE_OSG)
        target_link_libraries(
            ${TARGET}
            PRIVATE ViewerBase
                    ${OSG_LIBRARIES})
    endif()

    if(USE_SUMO)
        target_link_libraries(
            ${TARGET}
            PRIVATE ${SUMO_LIBRARIES})
    endif()

    if(USE_OSI)
        target_link_libraries(
            ${TARGET}
            PRIVATE ${OSI_LIBRARIES})
    endif()

    disable_static_analysis(${TARGET})
    disable_iwyu(${TARGET})

endforeach()

# ############################### Install############################################################################################################

install(
    TARGETS ${TARGET_SHARED}
            ${TARGET_STATIC}
    DESTINATION "${INSTALL_PATH}")

install(
    TARGETS ${TARGET_SHARED}
            ${TARGET_STATIC}
    DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
