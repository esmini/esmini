# ############################### Setting targets ####################################################################

set(TARGET
    hello_world_static)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files################################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/hello_world_static.cpp)

# ############################### Creating executable ################################################################

add_executable(
    ${TARGET}
    ${SOURCES})

target_include_directories(
    ${TARGET}
    SYSTEM
    PUBLIC ${ESMINI_LIB_PATH})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options
            esminiLib_static
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

# embed $origin (location of exe file) and link dirs as execution dyn lib search paths
set(RPATH_DIRS
    "$ORIGIN:${INSTALL_PATH}")

if(DYN_PROTOBUF)
    # add OSI library folder to execution lib search paths
    set(RPATH_DIRS
        ${RPATH_DIRS}:${EXTERNALS_OSI_LIBRARY_PATH}/$<IF:$<CONFIG:Debug>,debug,release>)
endif()

set_target_properties(
    ${TARGET}
    PROPERTIES BUILD_WITH_INSTALL_RPATH
               true
               INSTALL_RPATH
               ${RPATH_DIRS})

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET}
    DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
