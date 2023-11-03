# ############################### Setting targets ####################################################################

set(TARGET1
    dat2csv)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################

set(TARGET1_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Dat2csv.cpp
    ${REPLAYER_PATH}/Replay.cpp
    ${SCENARIO_ENGINE_PATH}/SourceFiles/DatLogger.cpp)

set(TARGET1_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/Dat2csv.hpp)

# ############################### Creating executable for target1 (dat2csv) ##########################################

add_executable(
    ${TARGET1}
    ${TARGET1_SOURCES})

target_link_libraries(
    ${TARGET1}
    PRIVATE project_options
            CommonMini)

target_include_directories(
    ${TARGET1}
    PRIVATE ${COMMON_MINI_PATH}
            ${REPLAYER_PATH}
            ${SCENARIO_ENGINE_PATH}/SourceFiles
            ${SCENARIO_ENGINE_PATH}/OSCTypeDefs)

target_include_directories(
    ${TARGET1}
    SYSTEM
    PUBLIC ${ROAD_MANAGER_PATH}
           ${EXTERNALS_OSI_INCLUDES}
           ${EXTERNALS_PUGIXML_PATH}
           ${EXTERNALS_OSG_INCLUDES}
           ${EXTERNALS_DIRENT_INCLUDES})

disable_static_analysis(${TARGET1})
disable_iwyu(${TARGET1})

install(
    TARGETS ${TARGET1}
    DESTINATION "${INSTALL_PATH}")
