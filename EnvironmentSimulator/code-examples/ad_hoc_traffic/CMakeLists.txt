# ############################### Setting targets ####################################################################

set(TARGET
    ad_hoc_traffic)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/ad_hoc_traffic.cpp)

# ############################### Creating executable ################################################################

add_executable(
    ${TARGET}
    ${SOURCES})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options)

target_include_directories(
    ${TARGET}
    PRIVATE
    ${EXTERNALS_PUGIXML_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM PUBLIC
    ${ESMINI_LIB_PATH})

target_link_libraries(
    ${TARGET}
    PRIVATE esminiLib)

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET}
    DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
