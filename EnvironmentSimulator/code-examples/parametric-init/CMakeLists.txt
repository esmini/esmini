# ############################### Setting targets ####################################################################

set(TARGET1
    parametric-init)

set(TARGET2
    override-bounding-box)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Creating executable ################################################################

add_executable(
    ${TARGET1}
    ${CMAKE_CURRENT_SOURCE_DIR}/parametric-init.cpp)

add_executable(
    ${TARGET2}
    ${CMAKE_CURRENT_SOURCE_DIR}/override-bounding-box.cpp)

foreach(
    target
    IN
    ITEMS ${TARGET1}
          ${TARGET2})

    target_link_libraries(
        ${target}
        PRIVATE project_options)

    target_include_directories(
        ${target}
        PRIVATE ${EXTERNALS_PUGIXML_PATH})

    target_include_directories(
        ${target}
        SYSTEM
        PUBLIC ${ESMINI_LIB_PATH}
               ${COMMON_MINI_PATH})

    target_link_libraries(
        ${target}
        PRIVATE project_options
                esminiLib
                CommonMini
                RoadManager
                ${TIME_LIB})

    # embed $origin (location of exe file) and linked lib dirs as execution dyn lib search paths
    set_target_properties(
        ${target}
        PROPERTIES BUILD_WITH_INSTALL_RPATH
                   true
                   INSTALL_RPATH
                   $ORIGIN:${INSTALL_PATH})

    set_folder(
        ${target}
        ${CodeExamplesFolder})

endforeach()

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET1}
            ${TARGET2}
    DESTINATION "${CODE_EXAMPLES_BIN_PATH}")
