# ############################### Setting targets ####################################################################

set(TARGET
    hello_world)

set(TARGET_TMP
    hello_world_tmp)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/system_link.cmake)

# ############################### Setting target files ###############################################################

file(
    GLOB
    modules
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "*.cpp")

# ############################### Creating executable ################################################################

include_directories(
    ${ESMINI_LIB_PATH}
    ${COMMON_MINI_PATH})

add_custom_target(${TARGET_TMP})

foreach(
    file_path
    ${modules})
    get_filename_component(
        filename
        ${file_path}
        NAME)

    string(
        REPLACE ".cpp"
                ""
                module_name
                ${filename})

    add_executable(
        ${module_name}
        ${filename})

    add_dependencies(
        ${TARGET_TMP}
        ${module_name})

    target_link_libraries(
        ${module_name}
        PRIVATE project_options)

    target_include_system_directories(
        ${module_name}
        PRIVATE
        ${EXTERNALS_PUGIXML_PATH})

    target_link_libraries(
        ${module_name}
        PRIVATE esminiLib)

    install(
        TARGETS ${module_name}
        DESTINATION ${CODE_EXAMPLES_BIN_PATH})

endforeach()
