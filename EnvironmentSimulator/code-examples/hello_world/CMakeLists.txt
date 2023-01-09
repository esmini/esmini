# ############################### Setting targets ####################################################################

set(TARGET
    hello_world)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Setting target files ###############################################################

file(
    GLOB
    modules
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "*.cpp")

# ############################### Creating executable ################################################################

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

    target_link_libraries(
        ${module_name}
        PRIVATE project_options)

    target_include_directories(
        ${module_name}
        PRIVATE
        ${EXTERNALS_PUGIXML_PATH}
        SYSTEM PUBLIC
        ${ESMINI_LIB_PATH}
        ${COMMON_MINI_PATH})

    target_link_libraries(
        ${module_name}
        PRIVATE esminiLib)

    set_folder( ${module_name} ${CodeExamplesFolder}/${TARGET} )

    install(
        TARGETS ${module_name}
        DESTINATION ${CODE_EXAMPLES_BIN_PATH})

endforeach()
