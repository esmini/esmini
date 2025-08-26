# ############################### Setting targets ####################################################################

set(TARGET
    road-model)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/road-model.cpp)

# ############################### Creating executable ################################################################

add_executable(
    ${TARGET}
    ${SOURCES})

target_include_directories(
    ${TARGET}
    SYSTEM
    PUBLIC ${ROAD_MANAGER_PATH}
           ${VIEWER_BASE_PATH}
           ${COMMON_MINI_PATH}
           ${EXTERNALS_OSG_INCLUDES})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options
            RoadManager
            CommonMini
            RoadGeom
            ${OSG_LIBRARIES}
            ${TIME_LIB})

# embed $origin (location of exe file) and linked lib dirs as execution dyn lib search paths
set_target_properties(
    ${TARGET}
    PROPERTIES BUILD_WITH_INSTALL_RPATH
               true
               INSTALL_RPATH
               $ORIGIN:${INSTALL_PATH})

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET}
    DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
