include_guard()

# ############################### Put target in specified folder #######################################

macro(
    set_folder
    target
    folder)

    if (${EXPLICIT_FOLDER_STRUCTURE})
        set_target_properties (${target} PROPERTIES FOLDER ${folder} )
    endif()

endmacro()
