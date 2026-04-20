include_guard()

# ############################### Setting osi libraries ##############################################################

macro(set_osi_libs)
    if(${OSI_VERSION} STREQUAL "3.5.0")
        if(NOT TARGET osi_headers)
            add_library(osi_headers INTERFACE)
        endif()

        target_include_directories(osi_headers SYSTEM INTERFACE "${EXTERNALS_OSI_INCLUDES}")

        if(APPLE)
            if(DYN_PROTOBUF)
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libopen_simulation_interface.dylib
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libprotobuf.dylib)
            else()
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libopen_simulation_interface_pic.a
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libprotobuf.a)
            endif()

        elseif(LINUX)
            if(DYN_PROTOBUF)
                set(OSI_LIBRARIES
                    optimized
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libopen_simulation_interface.so
                    optimized
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libprotobuf.so
                    debug
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libopen_simulation_interface.so
                    debug
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libprotobufd.so)
            else()
                set(OSI_LIBRARIES
                    optimized
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libopen_simulation_interface_pic.a
                    optimized
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libprotobuf.a
                    debug
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libopen_simulation_interface_pic.a
                    debug
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libprotobufd.a)
            endif()
        elseif(MSVC)
            if(DYN_PROTOBUF)
                set(OSI_LIBRARIES
                    optimized
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/open_simulation_interface_pic.lib
                    optimized
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libprotobuf.lib
                    debug
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/open_simulation_interface_pic.lib
                    debug
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libprotobufd.lib)
            else()
                set(OSI_LIBRARIES
                    optimized
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/open_simulation_interface_pic.lib
                    optimized
                    ${EXTERNALS_OSI_LIBRARY_PATH}/release/libprotobuf.lib
                    debug
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/open_simulation_interface_pic.lib
                    debug
                    ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libprotobufd.lib)
            endif()
        endif()

        if(NOT TARGET osi_with_warnings)
            add_library(osi_with_warnings INTERFACE)
        endif()

        # Make sure osi_headers is before osi_libraries
        target_link_libraries(osi_with_warnings INTERFACE osi_headers ${OSI_LIBRARIES})

        # set osi_libraries with our secured order of links
        set(OSI_LIBRARIES osi_with_warnings)

    elseif(${OSI_VERSION} STREQUAL "3.8.0")
        set(FULL_RELEASE_PATTERNS "")
        set(FULL_DEBUG_PATTERNS "")

        if(NOT TARGET osi_headers)
            add_library(osi_headers INTERFACE)
        endif()

        target_include_directories(osi_headers SYSTEM INTERFACE "${EXTERNALS_OSI_INCLUDES}")

        # Search for the dependency libs in static lib folder always
        if(APPLE)
            set(LIB_SEARCH_PATTERNS "libabsl_*.a" "libutf8*.a" "liblz4*.a" "libzstd*.a")

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
            set(LIB_SEARCH_PATTERNS "libabsl_*.a" "libutf8*.a" "liblz4*.a" "libzstd*.a")

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
            if(DYN_PROTOBUF)
                set(LIB_SEARCH_PATTERNS "utf8*.lib" "lz4*.lib" "zstd*.lib")
            else()
                set(LIB_SEARCH_PATTERNS "absl_*.lib" "utf8*.lib" "lz4*.lib" "zstd*.lib")
            endif()

            set(FULL_RELEASE_PATTERNS "")
            set(FULL_DEBUG_PATTERNS "")

            foreach(PATTERN ${LIB_SEARCH_PATTERNS})
                list(APPEND FULL_RELEASE_PATTERNS "${EXTERNALS_OSI_DEPS}/release/${PATTERN}")
                list(APPEND FULL_DEBUG_PATTERNS "${EXTERNALS_OSI_DEPS}/debug/${PATTERN}")
            endforeach()

            file(GLOB OSI_RELEASE_TRANSITIVE_LIBS ${FULL_RELEASE_PATTERNS})
            file(GLOB OSI_DEBUG_TRANSITIVE_LIBS ${FULL_DEBUG_PATTERNS})

            set(OSI_LIBRARIES "")

            list(APPEND OSI_LIBRARIES
                debug ${EXTERNALS_OSI_LIBRARY_PATH}/debug/open_simulation_interface_pic.lib
                debug ${EXTERNALS_OSI_LIBRARY_PATH}/debug/libprotobufd.lib
                optimized ${EXTERNALS_OSI_LIBRARY_PATH}/release/open_simulation_interface_pic.lib
                optimized ${EXTERNALS_OSI_LIBRARY_PATH}/release/libprotobuf.lib
            )

            # Explicitly link to abseil as well, as protobuf links to abseil dynamically by default
            if(DYN_PROTOBUF)
                list(APPEND OSI_LIBRARIES
                    debug ${EXTERNALS_OSI_LIBRARY_PATH}/debug/abseil_dll.lib
                    optimized ${EXTERNALS_OSI_LIBRARY_PATH}/release/abseil_dll.lib
                )
            endif()

            # 2. Add all transitive libs for Debug
            foreach(_lib ${OSI_DEBUG_TRANSITIVE_LIBS})
                list(APPEND OSI_LIBRARIES debug ${_lib})
            endforeach()

            # 3. Add all transitive libs for Release
            foreach(_lib ${OSI_RELEASE_TRANSITIVE_LIBS})
                list(APPEND OSI_LIBRARIES optimized ${_lib})
            endforeach()

            if(NOT TARGET osi_suppressions)
                add_library(osi_suppressions INTERFACE)
                target_compile_options(osi_suppressions INTERFACE
                    /wd4141 # 'inline' used more than once
                    /wd4267 # size_t to int conversion
                    /wd4244 # narrowing conversion
                    /wd4189 # local variable initialized but not referenced
                    /wd4296 # expression is always true/false
                    /wd4459 # declaration hides global declaration (triggered by abseil headers
                    /wd4251)
            endif()

            target_link_libraries(osi_headers INTERFACE osi_suppressions)

        endif()

        if(NOT TARGET osi_with_warnings)
            add_library(osi_with_warnings INTERFACE)
        endif()

        # Make sure osi_headers is before osi_libraries
        target_link_libraries(osi_with_warnings INTERFACE osi_headers ${OSI_LIBRARIES})

        # set osi_libraries with our secured order of links
        set(OSI_LIBRARIES osi_with_warnings)

    endif()


endmacro()
