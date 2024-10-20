# ############################### Setting targets ####################################################################

set(TARGET
    access-odr)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/access-odr.cpp)

# ############################### Creating executable ################################################################

add_executable(
    ${TARGET}
    ${SOURCES})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options)

target_include_directories(
    ${TARGET}
    PRIVATE ${ROAD_MANAGER_PATH}
            ${EXTERNALS_PUGIXML_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM
    PUBLIC ${ESMINI_LIB_PATH}
           ${COMMON_MINI_PATH})

target_link_libraries(
    ${TARGET}
    PRIVATE esminiLib
    PRIVATE CommonMini
    PRIVATE RoadManager
    PRIVATE ${TIME_LIB})

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
