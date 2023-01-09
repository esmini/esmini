# ############################### Setting targets ####################################################################

set(TARGET
    PlayerBase)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/enable_fpic.cmake)

if(USE_OSG)
    add_definitions(-DOSG_LIBRARY_STATIC)
endif(USE_OSG)

# ############################### Setting target files ###############################################################

set(SOURCES
    playerbase.cpp)

set(INCLUDES
    playerbase.hpp
    helpText.hpp)

# ############################### Creating library ###################################################################

add_library(
    ${TARGET}
    STATIC
    ${SOURCES}
    ${INCLUDES})

target_include_directories(
    ${TARGET}
    PRIVATE
    project_options
    ${SCENARIO_ENGINE_PATH}/SourceFiles
    ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
    ${COMMON_MINI_PATH}
    ${VIEWER_BASE_PATH}
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
