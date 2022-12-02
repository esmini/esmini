include_guard()

function(
    enable_fpic
    target)

    set_property(
        TARGET ${target}
        PROPERTY POSITION_INDEPENDENT_CODE
                 ON)

endfunction()
