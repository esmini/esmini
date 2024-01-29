# ############################### Getting code-examples ################################################################

get_subdirectories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    subdirectories)

if(NOT
   USE_OSI)
    list(
        REMOVE_ITEM
        subdirectories
        "osi-groundtruth"
        "osi-groundtruth-dyn"
        "osi-traffic_command")
endif()

# ############################### Building code-examples ###############################################################

foreach(
    subdir
    ${subdirectories})
    if(NOT
       ${subdir}
       STREQUAL
       "trig_lane_change"
       AND NOT
           (${subdir}
            STREQUAL
            "osi-groundtruth-cs"
            OR ${subdir}
               STREQUAL
               "rm-basic-cs"))
        add_subdirectory(${subdir})
        if(NOT
           ${subdir}
           STREQUAL
           "hello_world")
            # Add all code examples to the same folder
            set_folder(
                ${subdir}
                ${CodeExamplesFolder})
        endif()
    endif()
endforeach()
