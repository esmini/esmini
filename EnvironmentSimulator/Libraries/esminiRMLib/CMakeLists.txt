# ############################### Setting targets ####################################################################

set(TARGET
    esminiRMLib)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/system_link.cmake)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/esminiRMLib.cpp)

set(INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/esminiRMLib.hpp)

# ############################### Creating library ###################################################################

include_directories(
    ${SCENARIO_ENGINE_PATH}/SourceFiles
    ${SCENARIO_ENGINE_PATH}/OSCTypeDefs
    ${COMMON_MINI_PATH})

add_library(
    ${TARGET}
    SHARED
    ${SOURCES}
    ${INCLUDES})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options)

target_include_system_directories(
    ${TARGET}
    PRIVATE
    ${ROAD_MANAGER_PATH}
    ${EXTERNALS_PUGIXML_PATH})

target_link_libraries(
    ${TARGET}
    PRIVATE RoadManager
    PRIVATE CommonMini
    PRIVATE ${TIME_LIB})

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
