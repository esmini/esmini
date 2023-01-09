# ############################### Setting targets ####################################################################

set(TARGET
    image-capture)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/image-capture.cpp)

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
    ${ESMINI_LIB_PATH}
    ${COMMON_MINI_PATH})

target_link_libraries(
    ${TARGET}
    PRIVATE
    esminiLib
    CommonMini
    RoadManager
    ${TIME_LIB})

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET}
    DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
