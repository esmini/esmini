include_guard()

# ############################### Setting flags libraries ##############################################################

macro(set_special_build_flags)

    if(APPLE)
        set(CMAKE_CXX_FLAGS
            "${CXX_STD_FLAG} -std=c++14 -pthread -fPIC -flto -DGL_SILENCE_DEPRECATION -Wl,-dead_strip")
    elseif(LINUX)
        include(CheckCXXCompilerFlag)

        # Check for standard to use
        check_cxx_compiler_flag(
            -std=c++14
            HAVE_FLAG_STD_CXX14)

        if(HAVE_FLAG_STD_CXX14)
            set(CXX_STD_FLAG
                "-std=c++14")
        else()
            check_cxx_compiler_flag(
                -std=c++1y
                HAVE_FLAG_STD_CXX1Y)
            if(HAVE_FLAG_STD_CXX1Y)
                set(CXX_STD_FLAG
                    "-std=c++1y")
            else()
                message("Need compiler support for c++14 \(or 1y as the beta was called\)")
            endif()
        endif()

        set(CMAKE_C_FLAGS
            "-std=c11 ${CMAKE_C_FLAGS}")

        if(CMAKE_BUILD_TYPE
           STREQUAL
           "Release")
            set(CMAKE_CXX_FLAGS
                "${CXX_STD_FLAG} -pthread -fPIC -Wl,-strip-all")
        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "RelWithDebInfo")
            set(CMAKE_CXX_FLAGS
                "${CXX_STD_FLAG} -march=native -pthread -fPIC")
        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "Debug")
            set(CMAKE_CXX_FLAGS
                "${CXX_STD_FLAG} -march=native -O0 -pthread -fPIC")
        else()
            set(CMAKE_CXX_FLAGS
                "${CXX_STD_FLAG} -pthread -fPIC -Wl,-strip-all")
        endif()

    elseif(MSVC)
        # Get rid of historical macros preventing SUMO integration
        set(CMAKE_CXX_FLAGS
            "${CMAKE_CXX_FLAGS} /D NOMINMAX ")

        if((MSVC)
           AND (MSVC_VERSION
                GREATER_EQUAL
                1910))

            # remove default warning level from initial CMAKE_CXX_FLAGS
            string (REGEX REPLACE "/W[0-4]" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
            string (REGEX REPLACE "/W[0-4]" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")

            # suppress warning 4127 (conditional expression is constant) to avoid harmless warning from Google protobuf header
            # suppress warning 4100 (unreferenced formal parameter) to allow function headers with unreferenced parameters for increased readability

            add_compile_options(
                /permissive-
                /W4
                /wd4127
                /wd4100)
        endif()
    elseif(MINGW)
        include(CheckCXXCompilerFlag)

        # Check for standard to use
        check_cxx_compiler_flag(
            -std=c++14
            HAVE_FLAG_STD_CXX14)

        if(HAVE_FLAG_STD_CXX14)
            set(CXX_STD_FLAG
                "-std=c++14")
        else()
            check_cxx_compiler_flag(
                -std=c++1y
                HAVE_FLAG_STD_CXX1Y)
            if(HAVE_FLAG_STD_CXX1Y)
                set(CXX_STD_FLAG
                    "-std=c++1y")
            else()
                message("Need compiler support for c++14 \(or 1y as the beta was called\)")
            endif()
        endif()

        set(CMAKE_C_FLAGS
            "-std=c11 ${CMAKE_C_FLAGS}")

        if(CMAKE_BUILD_TYPE
           STREQUAL
           "Release")
            set(CMAKE_CXX_FLAGS
                "${CXX_STD_FLAG} -fPIC -Wl,-strip-all")
        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "RelWithDebInfo")
            set(CMAKE_CXX_FLAGS
                "${CXX_STD_FLAG} -march=native -pthread -fPIC")
        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "Debug")
            set(CMAKE_CXX_FLAGS
                "${CXX_STD_FLAG} -march=native -O0 -fPIC")
        else()
            set(CMAKE_CXX_FLAGS
                "${CXX_STD_FLAG} -fPIC -Wl,-strip-all")
        endif()
    endif()

endmacro()
