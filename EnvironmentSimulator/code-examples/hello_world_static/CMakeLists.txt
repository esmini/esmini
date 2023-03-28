# ############################### Setting targets ###################################################################################################

set(TARGET
    hello_world_static)

# ############################### Setting target files###############################################################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/hello_world_static.cpp)

# ############################### Creating executable ###############################################################################################

add_executable(
    ${TARGET}
    ${SOURCES})

target_include_directories(
    ${TARGET}
    SYSTEM
    PUBLIC ${ESMINI_LIB_PATH})

target_link_libraries(
    ${TARGET}
    PRIVATE esminiLib_static
            PlayerBase
            ScenarioEngine
            CommonMini
            Controllers
            RoadManager
            ${VIEWER_BASE}
            ${OSI_LIBRARIES}
            ${SUMO_LIBRARIES}
            ${TIME_LIB}
            ${SOCK_LIB})

# ############################### Install ###########################################################################################################

install(
    TARGETS ${TARGET}
    DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
