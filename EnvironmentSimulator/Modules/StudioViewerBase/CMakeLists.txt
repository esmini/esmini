# ############################### Setting targets ####################################################################

set(TARGET
    StudioViewerBase)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/enable_fpic.cmake)

add_definitions(-DOSG_LIBRARY_STATIC)

set(SOURCES
    StudioDataModel.cpp
    StudioGui.cpp
    StudioViewer.cpp
    TopViewManipulator.cpp)

set(INCLUDES
    StudioDataModel.hpp
    StudioGui.hpp
    StudioViewer.hpp
    TopViewManipulator.hpp)

# ############################### Setting target files ###############################################################

# ############################### Creating library ###################################################################

add_library(
    StudioViewerBase
    STATIC
    ${SOURCES}
    ${INCLUDES})

target_include_directories(
    ${TARGET}
    PRIVATE ${SCENARIO_ENGINE_PATH}/SourceFiles
            ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
            ${COMMON_MINI_PATH}
            ${CONTROLLERS_PATH}
            ${VIEWER_BASE_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM
    PUBLIC ${ROAD_MANAGER_PATH}
           ${VIEWER_BASE_PATH}
           ${EXTERNALS_IMPLOT_INCLUDES}
           ${EXTERNALS_OSG_INCLUDES}
           ${EXTERNALS_OSI_INCLUDES}
           ${EXTERNALS_JSON_INCLUDES}
           ${EXTERNALS_PUGIXML_PATH}
           ${EXTERNALS_SPDLOG_INCLUDES})

disable_static_analysis(${TARGET})
disable_iwyu(${TARGET})
enable_fpic(${TARGET})
