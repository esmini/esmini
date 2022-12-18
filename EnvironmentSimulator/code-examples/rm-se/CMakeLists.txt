# ############################### Setting targets ####################################################################

set(TARGET
    rm-se)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/system_link.cmake)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/rm-se.cpp)

# ############################### Creating executable ################################################################

include_directories(
    ${ESMINI_LIB_PATH}
    ${ESMINI_RM_LIB_PATH}
    ${COMMON_MINI_PATH})

add_executable(
    ${TARGET}
    ${SOURCES})

target_link_libraries(
    ${TARGET}
    PRIVATE project_options)

target_include_system_directories(
    ${TARGET}
    PRIVATE
    ${EXTERNALS_PUGIXML_PATH})

target_link_libraries(
    ${TARGET}
    PRIVATE esminiLib
    PRIVATE esminiRMLib
    PRIVATE CommonMini
    PRIVATE ${TIME_LIB})

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET}
    DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
