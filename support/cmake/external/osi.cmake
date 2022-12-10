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
        if(CMAKE_BUILD_TYPE
           STREQUAL
           "Release")

            if(DYN_PROTOBUF)
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface.so
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.so)
            else()
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface_pic.a
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.a)
            endif()

        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "Debug")

            if(DYN_PROTOBUF)
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interfaced.so
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobufd.so)
            else()
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface_picd.a
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobufd.a)
            endif()

        else()

            if(DYN_PROTOBUF)
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface.so
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.so)
            else()
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libopen_simulation_interface_pic.a
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.a)
            endif()

        endif()

    elseif(MSVC)
        if(CMAKE_BUILD_TYPE
           STREQUAL
           "Release")

            if(DYN_PROTOBUF)
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/open_simulation_interface.dll
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.lib)
            else()
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/open_simulation_interface_pic.lib
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.lib)
            endif()

        elseif(
            CMAKE_BUILD_TYPE
            STREQUAL
            "Debug")

            if(DYN_PROTOBUF)
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/open_simulation_interfaced.dll
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobufd.lib)
            else()
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/open_simulation_interface_picd.lib
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobufd.lib)
            endif()

        else()
            if(DYN_PROTOBUF)
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/open_simulation_interface.dll
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.lib)
            else()
                set(OSI_LIBRARIES
                    ${EXTERNALS_OSI_LIBRARY_PATH}/open_simulation_interface_pic.lib
                    ${EXTERNALS_OSI_LIBRARY_PATH}/libprotobuf.lib)
            endif()
        endif()
    endif()
endmacro()

# ############################### Creating osi interface library ######################################################

macro(add_osi_library)

    set_osi_libs()

    add_library(
        OSI
        INTERFACE)

    target_include_directories(
        OSI
        INTERFACE ${EXTERNALS_OSI_INCLUDES})

    target_link_libraries(
        OSI
        INTERFACE ${OSI_LIBRARIES})

endmacro()
