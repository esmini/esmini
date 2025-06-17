cmake_minimum_required(VERSION 3.5)

set(TARGET esmini)
project(${TARGET})

execute_process(
    # fetch latest esmini tag (release)
    COMMAND git describe --abbrev=0
    OUTPUT_VARIABLE VERSION
    ERROR_QUIET)

# Strip prefix "v" and any trailing line break
string(SUBSTRING "${VERSION}" 1 -1 VERSION )
string(STRIP "${VERSION}" VERSION)

set(OSIVERSION "3.5.0")
set(OSMPVERSION "1.4.0")

set(STATIC_LINKING TRUE)

if(LINUX)
    message("LINUX")
endif()

if(UNIX AND NOT APPLE)
    # assume Linux
    set(LINUX TRUE)
endif()

if(MSVC)
    set(EXT_INCL_DIR v10)
    set(CONFIG_FOLDER "Release")
    set(ADDITIONAL_LIBS Ws2_32 winmm)
    set(LIB_PREFIX "lib")
    set(LIB_POSTFIX "")
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FMI_BINARIES_PLATFORM "win64")
    else()
        set(FMI_BINARIES_PLATFORM "win32")
    endif()
elseif(APPLE)
    set(EXT_INCL_DIR mac)
    set(CONFIG_FOLDER "")
    set(ADDITIONAL_LIBS "")
    set(LIB_PREFIX "")
    set(LIB_POSTFIX .a)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FMI_BINARIES_PLATFORM "darwin64")
    else()
        set(FMI_BINARIES_PLATFORM "darwin32")
    endif()
elseif(LINUX)
    set(EXT_INCL_DIR linux)
    set(CONFIG_FOLDER "")
    set(ADDITIONAL_LIBS "")
    set(LIB_PREFIX "")
    set(LIB_POSTFIX .a)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FMI_BINARIES_PLATFORM "linux64")
    else()
        set(FMI_BINARIES_PLATFORM "linux32")
    endif()
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LINK_WITH_SHARED_OSI
    OFF
    CACHE BOOL "Link FMU with shared OSI library instead of statically linking")
set(PUBLIC_LOGGING
    OFF
    CACHE BOOL "Enable logging via FMI logger")
set(PRIVATE_LOGGING
    OFF
    CACHE BOOL "Enable private logging to file")
if(WIN32)
    set(PRIVATE_LOG_PATH_SOURCE
        "C:/TEMP/OSMPDummySourceLog.log"
        CACHE FILEPATH "Path to write private log file to")
else()
    set(PRIVATE_LOG_PATH_SOURCE
        "/tmp/OSMPDummySourceLog.log"
        CACHE FILEPATH "Path to write private log file to")
endif()
if(PRIVATE_LOGGING)
    file(TO_NATIVE_PATH ${PRIVATE_LOG_PATH_SOURCE} PRIVATE_LOG_PATH_SOURCE_NATIVE)
    string(REPLACE "\\" "\\\\" PRIVATE_LOG_PATH ${PRIVATE_LOG_PATH_SOURCE_NATIVE})
endif()
set(VERBOSE_FMI_LOGGING
    OFF
    CACHE BOOL "Enable detailed FMI function logging")
set(DEBUG_BREAKS
    OFF
    CACHE BOOL "Enable debugger traps for debug builds of FMU")

string(TIMESTAMP FMUTIMESTAMP UTC)
string(MD5 FMUGUID modelDescription.in.xml)
configure_file(modelDescription.in.xml modelDescription.xml @ONLY)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../EnvironmentSimulator/Libraries/esminiLib
                    ${CMAKE_CURRENT_SOURCE_DIR}/../externals/osi/${EXT_INCL_DIR}/include)

link_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../bin
    ${CMAKE_CURRENT_SOURCE_DIR}/../externals/osi/${EXT_INCL_DIR}/lib/release
    ${CMAKE_CURRENT_SOURCE_DIR}/../externals/sumo/${EXT_INCL_DIR}/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../externals/osg/${EXT_INCL_DIR}/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../externals/osg/${EXT_INCL_DIR}/lib/osgPlugins-3.6.5
    ${CMAKE_CURRENT_SOURCE_DIR}/../build/EnvironmentSimulator/Modules/ScenarioEngine/${CONFIG_FOLDER}
    ${CMAKE_CURRENT_SOURCE_DIR}/../build/EnvironmentSimulator/Modules/PlayerBase/${CONFIG_FOLDER}
    ${CMAKE_CURRENT_SOURCE_DIR}/../build/EnvironmentSimulator/Modules/RoadManager/${CONFIG_FOLDER}
    ${CMAKE_CURRENT_SOURCE_DIR}/../build/EnvironmentSimulator/Modules/Controllers/${CONFIG_FOLDER}
    ${CMAKE_CURRENT_SOURCE_DIR}/../build/EnvironmentSimulator/Modules/CommonMini/${CONFIG_FOLDER}
    ${CMAKE_CURRENT_SOURCE_DIR}/../build/EnvironmentSimulator/Modules/ViewerBase/${CONFIG_FOLDER})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../externals/fmi2/headers)

add_library(${TARGET} SHARED esmini.cpp)
set_target_properties(${TARGET} PROPERTIES PREFIX "")
target_compile_definitions(${TARGET} PRIVATE "FMU_SHARED_OBJECT")

if(STATIC_LINKING)
    message("Link esmini statically")
    set(EXTERNALS_OSI_LIBRARY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../externals/osi/${EXT_INCL_DIR}/lib)
    set(EXTERNALS_OSG_LIBRARY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../externals/osg/${EXT_INCL_DIR}/lib)
    set(EXTERNALS_OSG_PLUGINS_LIBRARY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../externals/osg/${EXT_INCL_DIR}/lib/osgPlugins-3.6.5)
    set(EXTERNALS_SUMO_LIBRARY_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../externals/sumo/${EXT_INCL_DIR}/lib)

    include(${CMAKE_CURRENT_SOURCE_DIR}/../support/cmake/external/osi.cmake)
    set_osi_libs()

    include(${CMAKE_CURRENT_SOURCE_DIR}/../support/cmake/external/sumo.cmake)
    set_sumo_libs()

    include(${CMAKE_CURRENT_SOURCE_DIR}/../support/cmake/external/osg.cmake)
    set_osg_libs()

    target_link_libraries(
        ${TARGET}
        PRIVATE esminiLib_static${LIB_POSTFIX}
                PlayerBase${LIB_POSTFIX}
                ScenarioEngine${LIB_POSTFIX}
                Controllers${LIB_POSTFIX}
                ViewerBase${LIB_POSTFIX}
                RoadManager${LIB_POSTFIX}
                CommonMini${LIB_POSTFIX}
                ${OSI_LIBRARIES}
                ${SUMO_LIBRARIES}
                ${OSG_LIBRARIES}
                pugixml_lib
                stdc++fs
                ${ADDITIONAL_LIBS})
else()
    message("Link esmini dynamically")
    target_link_libraries(${TARGET} esminiLib ${LIB_PREFIX}protobuf open_simulation_interface_pic)
endif()

add_custom_command(
    TARGET ${TARGET}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${TARGET}> $<$<PLATFORM_ID:Windows>:$<$<CONFIG:Debug>:$<TARGET_PDB_FILE:${TARGET}>>>
            "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
    COMMAND
        ${CMAKE_COMMAND} -E chdir "${CMAKE_CURRENT_BINARY_DIR}/buildfmu" ${CMAKE_COMMAND} -E tar "cfv" "../esmini.fmu" --format=zip
        "modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources"
        "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}")
