cmake_minimum_required(VERSION 3.25)

project(
    osi-groundtruth
    CSharp)

set(TARGET
    osi-groundtruth)

file(
    GLOB
    OSI_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/osi3/*.cs)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/osi-gt.cs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Libraries/esminiLib/ESMiniWrapper.cs
    ${OSI_SOURCES})

add_executable(
    ${TARGET}
    ${SOURCES})

set_property(
    TARGET ${TARGET}
    PROPERTY VS_PACKAGE_REFERENCES
             "Google.Protobuf_3.22.1")
