# ############################### Setting targets ####################################################################

set(TARGET
    osi-traffic_command)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/osi-traffic_command.cpp)

# ############################### Creating executable ################################################################

add_executable(
    ${TARGET}
    ${SOURCES})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options)

target_include_directories(
    ${TARGET}
    PRIVATE ${EXTERNALS_OSI_INCLUDES}
            ${EXTERNALS_PUGIXML_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM
    PUBLIC ${ESMINI_LIB_PATH}
           ${COMMON_MINI_PATH})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options
            esminiLib
            CommonMini
            RoadManager
            ${TIME_LIB}
            ${OSI_LIBRARIES})

# embed $origin (location of exe file) and linked lib dirs as execution dyn lib search paths
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
