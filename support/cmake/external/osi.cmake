include_guard()

# ############################### Setting osi libraries ##############################################################

macro(set_osi_libs)

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
        set(OSI_LIBRARIES
            /usr/local/lib/osi3/libopen_simulation_interface_pic.a
            protobuf
        )
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
endmacro()
