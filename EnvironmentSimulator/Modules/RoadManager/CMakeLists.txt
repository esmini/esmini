# ############################### Setting targets ####################################################################

set(TARGET
    RoadManager)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/system_link.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/enable_fpic.cmake)

if(MSVC)
    add_definitions("/wd4482")
endif()

# ############################### Setting target files ###############################################################

set(SOURCES
    RoadManager.cpp
    odrSpiral.cpp
    LaneIndependentRouter.cpp)

set(SRC_ADDITIONAL
    ${EXTERNALS_PUGIXML_PATH}/pugixml.cpp)

source_group(
    "External Libraries"
    FILES ${SRC_ADDITIONAL})

set(INCLUDES
    RoadManager.hpp
    odrSpiral.h
    LaneIndependentRouter.hpp)

# ############################### Creating library ###################################################################

include_directories(${COMMON_MINI_PATH})

add_library(
    ${TARGET}
    STATIC
    ${SOURCES}
    ${SRC_ADDITIONAL}
    ${INCLUDES})

target_link_libraries(
    ${TARGET}
    PRIVATE CommonMini
            # project_options
)

target_include_system_directories(
    ${TARGET}
    PRIVATE
    ${EXTERNALS_PUGIXML_PATH}
    ${ROAD_MANAGER_PATH})

disable_static_analysis(${TARGET})
disable_iwyu(${TARGET})
enable_fpic(${TARGET})
