include_guard()

# ############################### Setting osi libraries ##############################################################

macro(set_osi_libs)

    if(APPLE)
        set(OSI_LIBRARIES
            ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface_pic.a
            ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.a)

    elseif(LINUX)
        if(CMAKE_BUILD_TYPE
           STREQUAL
           "Release")
            set(OSI_LIBRARIES
                ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface_pic.a
                ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.a)
        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "Debug")
            set(OSI_LIBRARIES
                ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface_picd.a
                ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobufd.a)
        else()
            set(OSI_LIBRARIES
                ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface_pic.a
                ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.a)
        endif()

    elseif(MSVC)
        if(CMAKE_BUILD_TYPE
           STREQUAL
           "Release")
            set(OSI_LIBRARIES
                ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface_pic.a
                ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.a)
        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "Debug")
            set(OSI_LIBRARIES
                ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface_picd.a
                ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobufd.a)
        else()
            set(OSI_LIBRARIES
                ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface_pic.a
                ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.a)
        endif()
    endif()
endmacro()
