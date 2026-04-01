include_guard()

# ############################### Setting osi libraries ##############################################################
#
# OSI is provided by asam-osi-utilities (submodule in externals/asam-osi-utilities). The library must be pre-built and installed before configuring
# esmini (two-phase build). Set OSI_UTILITIES_PREFIX to the install prefix of asam-osi-utilities.
#
# IMPORTANT: On Windows, use the vcpkg-windows-static-md preset (or x64-windows-static-md triplet) to build asam-osi-utilities. This links
# protobuf/abseil statically so that each module (DLL/EXE) gets its own protobuf descriptor pool, avoiding double-registration crashes when
# esminiLib.dll and test executables both link OSI.
#
# This sets: OSI_LIBRARIES         — imported target for linking (carries includes + libs transitively) EXTERNALS_OSI_INCLUDES — include directories
# for targets that only need headers

macro(set_osi_libs)
    if(NOT
       OSI_UTILITIES_PREFIX)
        message(
            FATAL_ERROR
                "OSI_UTILITIES_PREFIX is not set. Build and install asam-osi-utilities first:\n"
                "  cmake -S externals/asam-osi-utilities -B build-deps --preset vcpkg-windows-static-md\n"
                "  cmake --build build-deps --config Release\n"
                "  cmake --install build-deps --config Release --prefix deps/\n"
                "Then configure esmini with:\n"
                "  -DOSI_UTILITIES_PREFIX=deps/\n"
                "  -DCMAKE_PREFIX_PATH=\"deps/;<vcpkg_installed>/x64-windows-static-md\"\n"
                "  -DProtobuf_PROTOC_EXECUTABLE=<vcpkg_installed>/x64-windows-static-md/tools/protobuf/protoc[.exe]")
    endif()

    message(STATUS "Using asam-osi-utilities for OSI (prefix: ${OSI_UTILITIES_PREFIX})")

    # Force CONFIG mode for Protobuf so its transitive dependencies (abseil, utf8_range) are resolved via the installed package configs rather than
    # CMake's built-in FindProtobuf module which skips them.
    find_package(
        Protobuf
        CONFIG
        REQUIRED)

    find_package(
        open_simulation_interface
        REQUIRED
        PATHS
        "${OSI_UTILITIES_PREFIX}"
        "${OSI_UTILITIES_PREFIX}/CMake"
        NO_DEFAULT_PATH)

    # Set OSI_LIBRARIES to the static PIC target — all existing target_link_libraries(... ${OSI_LIBRARIES}) calls work unchanged.
    set(OSI_LIBRARIES
        open_simulation_interface::open_simulation_interface_pic)

    # Collect include directories from the imported target so targets that only need OSI headers (without linking) can use EXTERNALS_OSI_INCLUDES. We
    # also need protobuf headers since the .pb.h files include them.
    get_target_property(
        _osi_inc_dirs
        ${OSI_LIBRARIES}
        INTERFACE_INCLUDE_DIRECTORIES)
    if(NOT
       _osi_inc_dirs)
        set(_osi_inc_dirs
            "")
    endif()

    # Protobuf headers are a transitive dependency — resolve them explicitly for targets that only use EXTERNALS_OSI_INCLUDES without linking.
    get_target_property(
        _proto_libs
        ${OSI_LIBRARIES}
        INTERFACE_LINK_LIBRARIES)
    if(_proto_libs)
        foreach(
            _lib
            ${_proto_libs})
            if(TARGET
               ${_lib})
                get_target_property(
                    _lib_dirs
                    ${_lib}
                    INTERFACE_INCLUDE_DIRECTORIES)
                if(_lib_dirs)
                    list(
                        APPEND
                        _osi_inc_dirs
                        ${_lib_dirs})
                endif()
            endif()
        endforeach()
    endif()

    set(EXTERNALS_OSI_INCLUDES
        ${_osi_inc_dirs})

    # Suppress MSVC warnings triggered by protobuf 5.x / abseil headers during template instantiation in esmini's own translation units.  Two
    # mechanisms:
    #
    # 1. osi_external_warnings — INTERFACE library carrying /wd flags.  Targets that only include OSI headers (without linking) should link this.
    # 2. OSI_LIBRARIES wrapper — targets linking OSI_LIBRARIES get the /wd flags automatically via the wrapper INTERFACE library below.
    if(MSVC)
        add_library(
            osi_external_warnings
            INTERFACE)
        target_compile_options(
            osi_external_warnings
            INTERFACE /wd4141 # 'inline' used more than once
                      /wd4267 # size_t narrowing
                      /wd4244 # narrowing conversion
                      /wd4189 # local variable initialized but not referenced
                      /wd4296 # expression is always true/false
                      /wd4459) # declaration hides global
    endif()

    # Wrap the imported target so that linking OSI_LIBRARIES also propagates the MSVC warning suppressions.
    add_library(
        osi_with_warnings
        INTERFACE)
    target_link_libraries(
        osi_with_warnings
        INTERFACE open_simulation_interface::open_simulation_interface_pic)
    if(MSVC)
        target_link_libraries(
            osi_with_warnings
            INTERFACE osi_external_warnings)
    endif()
    set(OSI_LIBRARIES
        osi_with_warnings)

    message(STATUS "OSI target: ${OSI_LIBRARIES}")
endmacro()
