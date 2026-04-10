include_guard()

# ############################### Setting osi libraries ##############################################################

macro(set_osi_libs)

    set(FULL_RELEASE_PATTERNS "")
    set(FULL_DEBUG_PATTERNS "")

    add_library(osi_headers INTERFACE)
    target_include_directories(osi_headers SYSTEM INTERFACE "${EXTERNALS_OSI_INCLUDES}")

    # Search for the dependency libs in static lib folder always
    if(APPLE)
        if(DYN_PROTOBUF)
            set(LIB_SEARCH_PATTERNS "libabsl_*.a" "libutf8*.dylib" "liblz4*.dylib" "libzstd*.dylib")
        else()
            set(LIB_SEARCH_PATTERNS "libabsl_*.a" "libutf8*.a" "liblz4*.a" "libzstd*.a")
        endif()

        foreach(PATTERN ${LIB_SEARCH_PATTERNS})
            list(APPEND FULL_RELEASE_PATTERNS "${EXTERNALS_OSI_DEPS}/release/${PATTERN}")
            list(APPEND FULL_DEBUG_PATTERNS "${EXTERNALS_OSI_DEPS}/debug/${PATTERN}")
        endforeach()

        file(GLOB OSI_RELEASE_TRANSITIVE_LIBS ${FULL_RELEASE_PATTERNS})
        file(GLOB OSI_DEBUG_TRANSITIVE_LIBS ${FULL_DEBUG_PATTERNS})

        if(DYN_PROTOBUF)
            if(CMAKE_BUILD_TYPE STREQUAL "Debug")
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libopen_simulation_interface.dylib
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libprotobufd.dylib
                    ${OSI_DEBUG_TRANSITIVE_LIBS})
            else()
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libopen_simulation_interface.dylib
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libprotobuf.dylib
                    ${OSI_RELEASE_TRANSITIVE_LIBS})
            endif()
        else()
            if(CMAKE_BUILD_TYPE STREQUAL "Debug")
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libopen_simulation_interface_pic.a
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libprotobufd.a
                    ${OSI_DEBUG_TRANSITIVE_LIBS})
            else()
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libopen_simulation_interface_pic.a
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libprotobuf.a
                    ${OSI_RELEASE_TRANSITIVE_LIBS})
            endif()
        endif()

    elseif(LINUX)
        if(DYN_PROTOBUF)
            set(LIB_SEARCH_PATTERNS "libabsl_*.a" "libutf8*.so" "liblz4*.so" "libzstd*.so")
        else()
            set(LIB_SEARCH_PATTERNS "libabsl_*.a" "libutf8*.a" "liblz4*.a" "libzstd*.a")
        endif()

        foreach(PATTERN ${LIB_SEARCH_PATTERNS})
            list(APPEND FULL_RELEASE_PATTERNS "${EXTERNALS_OSI_DEPS}/release/${PATTERN}")
            list(APPEND FULL_DEBUG_PATTERNS "${EXTERNALS_OSI_DEPS}/debug/${PATTERN}")
        endforeach()

        file(GLOB OSI_RELEASE_TRANSITIVE_LIBS ${FULL_RELEASE_PATTERNS})
        file(GLOB OSI_DEBUG_TRANSITIVE_LIBS ${FULL_DEBUG_PATTERNS})

        if(DYN_PROTOBUF)
            if(CMAKE_BUILD_TYPE STREQUAL "Debug")
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libopen_simulation_interface.so
                    -Wl,--start-group
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libprotobufd.so
                    ${OSI_DEBUG_TRANSITIVE_LIBS}
                    -Wl,--end-group)
            else()
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libopen_simulation_interface.so
                    -Wl,--start-group
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libprotobuf.so
                    ${OSI_RELEASE_TRANSITIVE_LIBS}
                    -Wl,--end-group)
            endif()
        else()
            if(CMAKE_BUILD_TYPE STREQUAL "Debug")
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libopen_simulation_interface_pic.a
                    -Wl,--start-group
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libprotobufd.a
                    ${OSI_DEBUG_TRANSITIVE_LIBS}
                    -Wl,--end-group)
            else()
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libopen_simulation_interface_pic.a
                    -Wl,--start-group
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libprotobuf.a
                    ${OSI_RELEASE_TRANSITIVE_LIBS}
                    -Wl,--end-group)
            endif()
        endif()

    elseif(MSVC)
        set(LIB_SEARCH_PATTERNS "absl_*.lib" "utf8*.lib" "lz4*.lib" "zstd*.lib")

        foreach(PATTERN ${LIB_SEARCH_PATTERNS})
            list(APPEND FULL_RELEASE_PATTERNS "${EXTERNALS_OSI_DEPS}/release/${PATTERN}")
            list(APPEND FULL_DEBUG_PATTERNS "${EXTERNALS_OSI_DEPS}/debug/${PATTERN}")
        endforeach()

        file(GLOB OSI_RELEASE_TRANSITIVE_LIBS ${FULL_RELEASE_PATTERNS})
        file(GLOB OSI_DEBUG_TRANSITIVE_LIBS ${FULL_DEBUG_PATTERNS})

        set(OSI_LIBRARIES
            debug     ${EXTERNALS_OSI_LIBRARY_PATH}/debug/open_simulation_interface_pic.lib
            debug     ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libprotobufd.lib
            debug     ${OSI_DEBUG_TRANSITIVE_LIBS}
            optimized ${EXTERNALS_OSI_LIBRARY_PATH}/release/open_simulation_interface_pic.lib
            optimized ${EXTERNALS_OSI_LIBRARY_PATH}/release/libprotobuf.lib
            optimized ${OSI_RELEASE_TRANSITIVE_LIBS})

        # Explicitly suppress these warnings for OSI
        add_library(osi_suppressions INTERFACE)
        target_compile_options(osi_suppressions INTERFACE
        /wd4141 # 'inline' used more than once
        /wd4267 # size_t to int conversion
        /wd4244 # narrowing conversion
        /wd4189 # local variable initialized but not referenced
        /wd4296 # expression is always true/false
        /wd4459) # declaration hides global

        target_link_libraries(osi_headers INTERFACE osi_suppressions)
        list(APPEND OSI_LIBRARIES osi_suppressions)

    endif()

    set(EXTERNALS_OSI_INCLUDES osi_headers)
    list(APPEND OSI_LIBRARIES osi_headers)

endmacro()
