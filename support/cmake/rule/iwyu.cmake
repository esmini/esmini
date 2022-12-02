include_guard()

macro(enable_iwyu)

    if(ENABLE_INCLUDE_WHAT_YOU_USE)
        find_program(
            INCLUDE_WHAT_YOU_USE
            include-what-you-use)
        if(INCLUDE_WHAT_YOU_USE)
            set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE
                ${INCLUDE_WHAT_YOU_USE};
                -Xiwyu
                --no_comments)
        else()
            message(
                ${WARNING_MESSAGE}
                "include-what-you-use requested but executable not found")
        endif()
    endif()

endmacro()
