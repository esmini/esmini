include_guard()

# ############################### Setting definitions ##############################################################

macro(set_definitions)

    if(USE_OSG)
        add_definitions(-D_USE_OSG)
    endif(USE_OSG)

    if(USE_OSI)
        add_definitions(-D_USE_OSI)
    endif(USE_OSI)

    if(USE_SUMO)
        add_definitions(-D_USE_SUMO)
    endif(USE_SUMO)

    if(USE_GTEST)
        add_definitions(-D_USE_GTEST)
    endif(USE_GTEST)

    if(DYN_PROTOBUF)
        add_definitions(-D_DYN_PROTOBUF)
        add_definitions(-DPROTOBUF_USE_DLLS)
    endif(DYN_PROTOBUF)

endmacro()
