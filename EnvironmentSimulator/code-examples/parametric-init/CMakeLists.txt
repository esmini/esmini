# ############################### Setting targets ####################################################################

set(TARGET1
    parametric-init)

set(TARGET2
    override-bounding-box)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################

set(SOURCES1
    ${CMAKE_CURRENT_SOURCE_DIR}/parametric-init.cpp)

set(SOURCES2
    ${CMAKE_CURRENT_SOURCE_DIR}/override-bounding-box.cpp)

# ############################### Creating executable ################################################################

add_executable(
    ${TARGET1}
    ${SOURCES1})

add_executable(
    ${TARGET2}
    ${SOURCES2})

foreach(
    TARGET
    IN
    ITEMS ${TARGET1}
          ${TARGET2})

    target_link_libraries(
        ${TARGET}
        PRIVATE project_options)

    target_include_directories(
        ${TARGET}
        PRIVATE ${EXTERNALS_PUGIXML_PATH})

    target_include_directories(
        ${TARGET}
        SYSTEM
        PUBLIC ${ESMINI_LIB_PATH}
               ${COMMON_MINI_PATH})

    target_link_libraries(
        ${TARGET}
        PRIVATE esminiLib
                CommonMini
                RoadManager
                ${TIME_LIB})

endforeach()

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET1}
            ${TARGET2}
    DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
