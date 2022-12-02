function(
    enable_sanitizers
    project_name)

    if(CMAKE_CXX_COMPILER_ID
       STREQUAL
       "GNU"
       OR CMAKE_CXX_COMPILER_ID
          MATCHES
          ".*Clang")

        set(SANITIZERS
            "")

        list(
            APPEND
            SANITIZERS
            "address")

        list(
            APPEND
            SANITIZERS
            "undefined")

        list(
            JOIN
            SANITIZERS
            ","
            LIST_OF_SANITIZERS)

        target_compile_options(
            ${project_name}
            INTERFACE -fsanitize=${LIST_OF_SANITIZERS}
                      -fno-omit-frame-pointer)
        target_link_options(
            ${project_name}
            INTERFACE
            -fsanitize=${LIST_OF_SANITIZERS})

    endif()

endfunction()
