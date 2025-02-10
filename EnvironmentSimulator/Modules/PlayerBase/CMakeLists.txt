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
    playerbase.cpp
    PlayerServer.cpp)

if(USE_IMPLOT)
    list(
        APPEND
        SOURCES
        Plot.cpp)
endif(USE_IMPLOT)

set(INCLUDES
    playerbase.hpp
    PlayerServer.hpp
    helpText.hpp)

if(USE_IMPLOT)
    list(
        APPEND
        INCLUDES
        Plot.hpp)
endif(USE_IMPLOT)

# ############################### Creating library ###################################################################

add_library(
    ${TARGET}
    STATIC
    ${SOURCES}
    ${INCLUDES})

target_include_directories(
    ${TARGET}
    PRIVATE ${SCENARIO_ENGINE_PATH}/SourceFiles
            ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
            ${COMMON_MINI_PATH}
            ${VIEWER_BASE_PATH}
            ${CONTROLLERS_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM
    PUBLIC ${ROAD_MANAGER_PATH}
           ${EXTERNALS_OSG_INCLUDES}
           ${EXTERNALS_OSI_INCLUDES}
           ${EXTERNALS_PUGIXML_PATH}
           ${EXTERNALS_IMPLOT_INCLUDES}
           ${EXTERNALS_SPDLOG_INCLUDES})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options)

disable_static_analysis(${TARGET})
disable_iwyu(${TARGET})
enable_fpic(${TARGET})
