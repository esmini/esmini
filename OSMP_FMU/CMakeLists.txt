cmake_minimum_required(VERSION 3.5)

set(TARGET EsminiOsiSource)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../EnvironmentSimulator/Libraries/esminiLib ${CMAKE_CURRENT_SOURCE_DIR}/../externals/osi/linux/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../bin ${CMAKE_CURRENT_SOURCE_DIR}/../externals/osi/linux/lib)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../externals/fmi2/headers )

project(${TARGET})

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LINK_WITH_SHARED_OSI OFF CACHE BOOL "Link FMU with shared OSI library instead of statically linking")
set(PUBLIC_LOGGING OFF CACHE BOOL "Enable logging via FMI logger")
set(PRIVATE_LOGGING OFF CACHE BOOL "Enable private logging to file")
if(WIN32)
    set(PRIVATE_LOG_PATH_SOURCE "C:/TEMP/OSMPDummySourceLog.log" CACHE FILEPATH "Path to write private log file to")
else()
    set(PRIVATE_LOG_PATH_SOURCE "/tmp/OSMPDummySourceLog.log" CACHE FILEPATH "Path to write private log file to")
endif()
if(PRIVATE_LOGGING)
    file(TO_NATIVE_PATH ${PRIVATE_LOG_PATH_SOURCE} PRIVATE_LOG_PATH_SOURCE_NATIVE)
    string(REPLACE "\\" "\\\\" PRIVATE_LOG_PATH ${PRIVATE_LOG_PATH_SOURCE_NATIVE})
endif()
set(VERBOSE_FMI_LOGGING OFF CACHE BOOL "Enable detailed FMI function logging")
set(DEBUG_BREAKS OFF CACHE BOOL "Enable debugger traps for debug builds of FMU")

string(TIMESTAMP FMUTIMESTAMP UTC)
string(MD5 FMUGUID modelDescription.in.xml)
configure_file(modelDescription.in.xml modelDescription.xml @ONLY)

add_library(${TARGET} SHARED EsminiOsiSource.cpp)
set_target_properties(${TARGET} PROPERTIES PREFIX "")
target_compile_definitions(${TARGET} PRIVATE "FMU_SHARED_OBJECT")

target_link_libraries(${TARGET} esminiLib protobuf open_simulation_interface_pic)

if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FMI_BINARIES_PLATFORM "win64")
    else()
        set(FMI_BINARIES_PLATFORM "win32")
    endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FMI_BINARIES_PLATFORM "linux64")
    else()
        set(FMI_BINARIES_PLATFORM "linux32")
    endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FMI_BINARIES_PLATFORM "darwin64")
    else()
        set(FMI_BINARIES_PLATFORM "darwin32")
    endif()
endif()

add_custom_command(TARGET ${TARGET}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
        COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${TARGET}> $<$<PLATFORM_ID:Windows>:$<$<CONFIG:Debug>:$<TARGET_PDB_FILE:${TARGET}>>> "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
        COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_CURRENT_BINARY_DIR}/buildfmu" ${CMAKE_COMMAND} -E tar "cfv" "../EsminiOsiSource.fmu" --format=zip "modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}")
