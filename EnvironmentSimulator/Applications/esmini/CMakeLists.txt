# ############################### Setting targets ####################################################################

set(TARGET
    esmini)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

## LOADING STUFF FOR DEARIMGUI

find_library(myglfw glfw REQUIRED)
if (${myglfw} STREQUAL myglfw-NOTFOUND)
    message(FATAL ERROR "Stuff broken")
else()
    message(STATUS "Found ${myglfw}")
endif()
find_library(mygl GL REQUIRED)
if (${mygl} STREQUAL mygl-NOTFOUND)
    message(FATAL ERROR "Stuff broken")
else()
    message(STATUS "Found ${mygl}")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/home/slundel6/esmini/externals/imgui/imgui -I/home/slundel6/esmini/externals/imgui/backends")
file(GLOB srcs /home/slundel6/esmini/externals/imgui/imgui/*.cpp)
file(GLOB bsrcs /home/slundel6/esmini/externals/imgui/backends/*.cpp)
add_library(imgui-objects OBJECT ${srcs} ${bsrcs})
# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

# ############################### Creating executable ################################################################

if(USE_OSG)
    set(VIEWER_BASE
        ViewerBase)
endif()

# Imgui objects
add_executable(
    ${TARGET}
    ${SOURCES} $<TARGET_OBJECTS:imgui-objects>)

target_include_directories(
    ${TARGET}
    PRIVATE ${ROAD_MANAGER_PATH}
            ${SCENARIO_ENGINE_PATH}/SourceFiles
            ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
            ${VIEWER_BASE_PATH}
            ${PLAYER_BASE_PATH}
            ${CONTROLLERS_PATH}
            ${COMMON_MINI_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM
    PUBLIC ${EXTERNALS_OSI_INCLUDES}
           ${EXTERNALS_PUGIXML_PATH}
           ${EXTERNALS_OSG_INCLUDES}
           ${EXTERNALS_SUMO_INCLUDES})

# GLFW, GL, DL added here
target_link_libraries(
    ${TARGET}
    PRIVATE project_options
            PlayerBase
            ScenarioEngine
            CommonMini
            Controllers
            RoadManager
            ${VIEWER_BASE}
            ${OSI_LIBRARIES}
            ${SUMO_LIBRARIES}
            ${TIME_LIB}
            ${SOCK_LIB}
            ${myglfw}
            ${mygl}
            dl)

if(USE_OSG)
    target_link_libraries(
        ${TARGET}
        PRIVATE ViewerBase
                ${OSG_LIBRARIES})
endif()

disable_static_analysis(${TARGET})
disable_iwyu(${TARGET})

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET}
    DESTINATION "${INSTALL_PATH}")
