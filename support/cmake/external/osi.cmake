include_guard()

# ############################### Setting osi libraries ##############################################################

macro(set_osi_libs)

    if(APPLE)
        if(DYN_PROTOBUF)
            set(OSI_LIBRARIES
                open_simulation_interface
                protobuf)
        else()
            set(OSI_LIBRARIES
                open_simulation_interface_pic
                protobuf)
        endif()
    elseif(LINUX)
        if(DYN_PROTOBUF)
            set(OSI_LIBRARIES
                optimized
                open_simulation_interface
                debug
                open_simulation_interfaced
                optimized
                protobuf
                debug
                protobufd)
        else()
            set(OSI_LIBRARIES
                optimized
                open_simulation_interface_pic
                debug
                open_simulation_interface_picd
                optimized
                protobuf
                debug
                protobufd)
        endif()
    elseif(MSVC)
        set(OSI_LIBRARIES
            optimized
            libprotobuf
            debug
            libprotobufd
            optimized
            open_simulation_interface_pic
            debug
            open_simulation_interface_picd)
    endif()

endmacro()
