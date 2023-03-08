include_guard()

# Enable coverage reporting for gcc/clang
function(
    enable_coverage
    project_name)

    target_compile_options(
        ${project_name}
        INTERFACE -Og
                  -g
                  --coverage
                  -fkeep-inline-functions
                  -fkeep-static-functions)
    target_link_libraries(
        ${project_name}
        INTERFACE --coverage)

endfunction()
