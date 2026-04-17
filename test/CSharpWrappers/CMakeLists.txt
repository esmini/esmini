cmake_minimum_required(VERSION 3.11)

set(USE_OSI
    ON
    CACHE BOOL
          "If code that depend on osi should be compiled.")

project(
    libesmini_cs_wrapper_test
    CSharp)

set(TARGET
    libesmini_cs_wrapper_test)

file(
    GLOB
    OSI_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../EnvironmentSimulator/code-examples/osi-groundtruth-cs/osi3/*.cs)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/libesmini_cs_wrapper_test.cs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../EnvironmentSimulator/Libraries/esminiLib/ESMiniWrapper.cs
    ${OSI_SOURCES})

add_executable(
    ${TARGET}
    ${SOURCES})

if(USE_OSI)
    add_definitions(-D_USE_OSI)
endif(USE_OSI)

set_property(
    TARGET ${TARGET}
    PROPERTY VS_PACKAGE_REFERENCES
             "Google.Protobuf_3.22.1")
