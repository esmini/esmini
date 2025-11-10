# ############################### Setting targets ####################################################################

set(TARGET_ROAD_GEOM
    RoadGeom)

set(TARGET_VIEWER_BASE
    ViewerBase)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/enable_fpic.cmake)

add_definitions(-DOSG_LIBRARY_STATIC)

set(SOURCES_VIEWER_BASE
    viewer.cpp
    RubberbandManipulator.cpp)

if(USE_OSI)
    set(SOURCES_VIEWER_BASE
        ${SOURCES_VIEWER_BASE}
        OSISensorView.cpp)
endif(USE_OSI)

set(INCLUDES_VIEWER_BASE
    viewer.hpp
    RubberbandManipulator.hpp)

if(USE_OSI)
    set(INCLUDES_VIEWER_BASE
        ${INCLUDES_VIEWER_BASE}
        OSISensorView.hpp)
endif(USE_OSI)

# ############################### Setting target files ###############################################################

# ############################### Creating library ###################################################################

# create a separate lib for roadgeom only
add_library(
    ${TARGET_ROAD_GEOM}
    STATIC
    roadgeom.cpp
    roadgeom.hpp
    trafficlightmodel.hpp
    trafficlightmodel.cpp)

target_include_directories(
    ${TARGET_ROAD_GEOM}
    PRIVATE ${COMMON_MINI_PATH})

target_include_directories(
    ${TARGET_ROAD_GEOM}
    SYSTEM
    PUBLIC ${ROAD_MANAGER_PATH}
           ${EXTERNALS_OSG_INCLUDES}
           ${EXTERNALS_PUGIXML_PATH}
           ${EXTERNALS_FMT_INCLUDES})

# main viewer lib
add_library(
    ${TARGET_VIEWER_BASE}
    STATIC
    ${SOURCES_VIEWER_BASE}
    ${INCLUDES_VIEWER_BASE})

target_include_directories(
    ${TARGET_VIEWER_BASE}
    PRIVATE ${SCENARIO_ENGINE_PATH}/SourceFiles
            ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
            ${COMMON_MINI_PATH}
            ${CONTROLLERS_PATH})

target_include_directories(
    ${TARGET_VIEWER_BASE}
    SYSTEM
    PUBLIC ${ROAD_MANAGER_PATH}
           ${EXTERNALS_OSG_INCLUDES}
           ${EXTERNALS_OSI_INCLUDES}
           ${EXTERNALS_PUGIXML_PATH}
           ${EXTERNALS_FMT_INCLUDES})

foreach(
    TARGET
    ${TARGET_VIEWER_BASE}
    ${TARGET_ROAD_GEOM})
    disable_static_analysis(${TARGET})
    disable_iwyu(${TARGET})
    enable_fpic(${TARGET})
endforeach()
