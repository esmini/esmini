function(
    disable_iwyu
    target)

    set_property(
        TARGET ${target}
        PROPERTY CXX_INCLUDE_WHAT_YOU_USE
                 "")
    set_property(
        TARGET ${target}
        PROPERTY C_INCLUDE_WHAT_YOU_USE
                 "")
endfunction()
