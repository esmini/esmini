# ############################### Setting targets ####################################################################

set(TARGET
    odrviewer)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/helpText.hpp)

# ############################### Creating executable ################################################################

add_executable(
    ${TARGET}
    ${SOURCES})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options)

target_include_directories(
    ${TARGET}
    PRIVATE
    ${SCENARIO_ENGINE_PATH}/SourceFiles
    ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
    ${VIEWER_BASE_PATH}
    ${COMMON_MINI_PATH}
    ${CONTROLLERS_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM PUBLIC
    ${ROAD_MANAGER_PATH}
    ${EXTERNALS_OSI_INCLUDES}
    ${EXTERNALS_PUGIXML_PATH}
    ${EXTERNALS_OSG_INCLUDES})

target_link_libraries(
    ${TARGET}
    PRIVATE RoadManager
    PRIVATE CommonMini
    PRIVATE ${TIME_LIB})

if(USE_OSG)
    target_link_libraries(
        ${TARGET}
        PRIVATE
        ViewerBase
        ${OSG_LIBRARIES})
endif()

if(USE_OSI)
    target_link_libraries(
        ${TARGET}
        PRIVATE
        ${OSI_LIBRARIES})
endif()

disable_static_analysis(${TARGET})
disable_iwyu(${TARGET})

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET}
    DESTINATION "${INSTALL_PATH}")
