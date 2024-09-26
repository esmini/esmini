include_guard()

# ############################### Setting flags libraries ##############################################################

macro(set_special_build_flags)

    set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} -DSPDLOG_COMPILED_LIB")

    if(APPLE)
        set(CMAKE_CXX_FLAGS
            "${CMAKE_CXX_FLAGS} ${CXX_STD_FLAG} -std=c++17 -pthread -fPIC -flto -DGL_SILENCE_DEPRECATION")
        set(CMAKE_EXE_LINKER_FLAGS
            "${CMAKE_EXE_LINKER_FLAGS} -dead_strip")
    elseif(LINUX)
        set(CXX_STD_FLAG
            "${CXX_STD_FLAG} -std=c++17")

        set(CMAKE_C_FLAGS
            "-std=c11 ${CMAKE_C_FLAGS}")

        if(CMAKE_BUILD_TYPE
           STREQUAL
           "Release")
            set(CMAKE_CXX_FLAGS
                "${CMAKE_CXX_FLAGS} ${CXX_STD_FLAG} -pthread -fPIC -Wl,-strip-all")
        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "RelWithDebInfo")
            set(CMAKE_CXX_FLAGS
                "${CMAKE_CXX_FLAGS} ${CXX_STD_FLAG} -march=x86-64 -pthread -fPIC")
        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "Debug")
            set(CMAKE_CXX_FLAGS
                "${CMAKE_CXX_FLAGS} ${CXX_STD_FLAG} -march=native -O0 -pthread -fPIC")
        else()
            set(CMAKE_CXX_FLAGS
                "${CMAKE_CXX_FLAGS} ${CXX_STD_FLAG} -pthread -fPIC -Wl,-strip-all")
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
            string(
                REGEX
                REPLACE "/W[0-4]"
                        ""
                        CMAKE_CXX_FLAGS
                        "${CMAKE_CXX_FLAGS}")
            string(
                REGEX
                REPLACE "/W[0-4]"
                        ""
                        CMAKE_C_FLAGS
                        "${CMAKE_C_FLAGS}")

            # suppress warning 4127 (conditional expression is constant) to avoid harmless warning from Google protobuf header suppress warning 4100
            # (unreferenced formal parameter) to allow function headers with unreferenced parameters for increased readability

            add_compile_options(
                /permissive-
                /W4
                /wd4127
                /wd4100)
        endif()
    elseif(MINGW)
        set(CXX_STD_FLAG
            "-std=c++17")
        set(CMAKE_C_FLAGS
            "-std=c11 ${CMAKE_C_FLAGS}")

        if(CMAKE_BUILD_TYPE
           STREQUAL
           "Release")
            set(CMAKE_CXX_FLAGS
                "${CMAKE_CXX_FLAGS} ${CXX_STD_FLAG} -fPIC -Wl,-strip-all")
        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "RelWithDebInfo")
            set(CMAKE_CXX_FLAGS
                "${CMAKE_CXX_FLAGS} ${CXX_STD_FLAG} -march=x86-64 -pthread -fPIC")
        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "Debug")
            set(CMAKE_CXX_FLAGS
                "${CMAKE_CXX_FLAGS} ${CXX_STD_FLAG} -march=native -O0 -fPIC")
        else()
            set(CMAKE_CXX_FLAGS
                "${CMAKE_CXX_FLAGS} ${CXX_STD_FLAG} -fPIC -Wl,-strip-all")
        endif()
    endif()

endmacro()
