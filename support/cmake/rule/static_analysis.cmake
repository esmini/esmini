include_guard()

macro(enable_static_analysis)

    find_program(
        CPPCHECK
        cppcheck)
    if(CPPCHECK)

        # Enable all warnings that are actionable by the user of this toolset style should enable the other 3, but we'll be explicit just in case
        set(CMAKE_CXX_CPPCHECK
            ${CPPCHECK}
            --template=gcc
            --enable=style,performance,warning,portability
            --inline-suppr
            # We cannot act on a bug/missing feature of cppcheck
            --suppress=internalAstError
            # if a file does not have an internalAstError, we get an unmatchedSuppression error
            --suppress=unmatchedSuppression
            --suppress=toomanyconfigs
            --inconclusive)

        if(ENABLE_WARNINGS_AS_ERRORS)
            list(
                APPEND
                CMAKE_CXX_CPPCHECK
                --error-exitcode=2)
        endif()

        # C cppcheck
        set(CMAKE_C_CPPCHECK
            ${CMAKE_CXX_CPPCHECK})

        if(NOT
           "${CMAKE_CXX_STANDARD}"
           STREQUAL
           "")
            set(CMAKE_CXX_CPPCHECK
                ${CMAKE_CXX_CPPCHECK}
                --std=c++${CMAKE_CXX_STANDARD})
        endif()

        set(CMAKE_C_CPPCHECK
            ${CMAKE_C_CPPCHECK}
            --std=c11)

    else()
        message(
            ${WARNING_MESSAGE}
            "cppcheck requested but executable not found")
    endif()

    find_program(
        CLANGTIDY
        clang-tidy-16)
    if(CLANGTIDY)

        # construct the clang-tidy command line
        set(CMAKE_CXX_CLANG_TIDY
            ${CLANGTIDY}
            -extra-arg=-Wno-unknown-warning-option)

        # set warnings as errors
        if(ENABLE_WARNINGS_AS_ERRORS)
            list(
                APPEND
                CMAKE_CXX_CLANG_TIDY
                -warnings-as-errors=*)
        endif()

        # C clang-tidy
        set(CMAKE_C_CLANG_TIDY
            ${CMAKE_CXX_CLANG_TIDY})

        # set C++ standard
        if(NOT
           "${CMAKE_CXX_STANDARD}"
           STREQUAL
           "")
            set(CMAKE_CXX_CLANG_TIDY
                ${CMAKE_CXX_CLANG_TIDY}
                -extra-arg=-std=c++${CMAKE_CXX_STANDARD})
        endif()

        # set C standard
        if(NOT
           "${CMAKE_C_STANDARD}"
           STREQUAL
           "")

            set(CMAKE_C_CLANG_TIDY
                ${CMAKE_C_CLANG_TIDY}
                -extra-arg=-std=c${CMAKE_C_STANDARD})
        endif()

    else()
        message(
            ${WARNING_MESSAGE}
            "clang-tidy requested but executable not found")
    endif()

endmacro()
