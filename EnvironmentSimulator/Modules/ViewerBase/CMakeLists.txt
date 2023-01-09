# ############################### Setting targets ####################################################################

set(TARGET
    ViewerBase)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/enable_fpic.cmake)

add_definitions(-DOSG_LIBRARY_STATIC)

set(SOURCES
    viewer.cpp
    roadgeom.cpp
    RubberbandManipulator.cpp)

set(INCLUDES
    viewer.hpp
    roadgeom.hpp
    RubberbandManipulator.hpp)

# ############################### Setting target files ###############################################################

# ############################### Creating library ###################################################################

add_library(
    ViewerBase
    STATIC
    ${SOURCES}
    ${INCLUDES})

target_link_libraries(
    ViewerBase
    PRIVATE
    ScenarioEngine
    project_options)

target_include_directories(
    ${TARGET}
    PRIVATE
    ${SCENARIO_ENGINE_PATH}/SourceFiles
    ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
    ${COMMON_MINI_PATH}
    ${CONTROLLERS_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM PUBLIC
    ${ROAD_MANAGER_PATH}
    ${EXTERNALS_OSG_INCLUDES}
    ${EXTERNALS_OSI_INCLUDES}
    ${EXTERNALS_PUGIXML_PATH})

disable_static_analysis(${TARGET})
disable_iwyu(${TARGET})
enable_fpic(${TARGET})
