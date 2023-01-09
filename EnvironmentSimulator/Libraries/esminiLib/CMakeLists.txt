# ############################### Setting targets ####################################################################

set(TARGET
    esminiLib)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/esminiLib.cpp
    ${CONTROLLERS_PATH}/vehicle.cpp)

set(INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/esminiLib.hpp
    ${CONTROLLERS_PATH}/vehicle.hpp)

# ############################### Creating library ###################################################################

add_library(
    ${TARGET}
    SHARED
    ${SOURCES}
    ${INCLUDES})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options)

target_include_directories(
    ${TARGET}
    PRIVATE
    ${SCENARIO_ENGINE_PATH}/SourceFiles
    ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
    ${VIEWER_BASE_PATH}
    ${PLAYER_BASE_PATH}
    ${COMMON_MINI_PATH}
    ${CONTROLLERS_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM PUBLIC
    ${ROAD_MANAGER_PATH}
    ${EXTERNALS_OSI_INCLUDES}
    ${EXTERNALS_PUGIXML_PATH})

target_link_libraries(
    ${TARGET}
    PRIVATE
    PlayerBase
    ScenarioEngine
    CommonMini
    Controllers
    RoadManager
    ${TIME_LIB}
    ${SOCK_LIB})

if(USE_OSG)
    target_link_libraries(
        ${TARGET}
        PRIVATE
        ViewerBase
        ${OSG_LIBRARIES})
endif()

if(USE_SUMO)
    target_link_libraries(
        ${TARGET}
        PRIVATE
        ${SUMO_LIBRARIES})
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

if(WIN32)
    install(
        TARGETS ${TARGET}
        RUNTIME DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
else()
    install(
        TARGETS ${TARGET}
        LIBRARY DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
endif()
