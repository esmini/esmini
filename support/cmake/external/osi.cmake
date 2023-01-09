include_guard()

# ############################### Setting osi libraries ##############################################################

macro(set_osi_libs)

    if(APPLE)
        if(DYN_PROTOBUF)
            set(OSI_LIBRARIES
                ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface.dylib
                ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.dylib)
        else()
            set(OSI_LIBRARIES
                ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface_pic.a
                ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.a)
        endif()

    elseif(LINUX)
        if(DYN_PROTOBUF)
            set(OSI_LIBRARIES
                optimized ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface.so
                optimized ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.so
                debug ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interfaced.so
                debug ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobufd.so)
        else()
            set(OSI_LIBRARIES
                optimized ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface_pic.a
                optimized ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.a
                debug ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface_picd.a
                debug ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobufd.a)
        endif()
    elseif(MSVC)
        if(DYN_PROTOBUF)
            set(OSI_LIBRARIES
                optimized ${EXTERNALS_OSI_LIBRARY_PATH}/open_simulation_interface.dll
                optimized ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.lib
                debug ${EXTERNALS_OSI_LIBRARY_PATH}/open_simulation_interfaced.dll
                debug ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobufd.lib)
        else()
            set(OSI_LIBRARIES
                optimized ${EXTERNALS_OSI_LIBRARY_PATH}/open_simulation_interface_pic.lib
                optimized ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.lib
                debug ${EXTERNALS_OSI_LIBRARY_PATH}/open_simulation_interface_picd.lib
                debug ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobufd.lib)
        endif()
    endif()
endmacro()

