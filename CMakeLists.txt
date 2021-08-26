message(STATUS "CMake version: " ${CMAKE_VERSION})

# Set to the current lowest tested version of CMake
if(WIN32)
  # for cmake generator VisualStudio 2017 support
  cmake_minimum_required (VERSION 3.7.0 FATAL_ERROR)
else()
  cmake_minimum_required (VERSION 2.8.12 FATAL_ERROR)
endif()

project (EnvironmentSimulator)

if(MSVC)
  set(CMAKE_VS_INCLUDE_INSTALL_TO_DEFAULT_BUILD 1)
endif()
set(CMAKE_CXX_STANDARD 14)

# Allow for folder structure
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# The static libaries needs to be compiled with position independent code
# otherwise we cant link with them when building dynamic libaries
# An alternative to this global option is to set POSITION_INDEPENDENT_CODE on each target
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


if ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "Win32")
  set(INSTALL_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin")
elseif (WIN32)
  set(INSTALL_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin")
elseif(UBUNTU32)
  set(INSTALL_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin")
elseif(APPLE)
  set(INSTALL_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin")
elseif((UNIX) AND (NOT APPLE))
  set(INSTALL_DIRECTORY "${CMAKE_HOME_DIRECTORY}/bin")
else()
  message(FATAL_ERROR "Unrecognized platform therefore there isn't an installation directory. Stopping the cmake process.")
endif()

set(INSTALL_DIRECTORY_CODE_EXAMPLES "${CMAKE_HOME_DIRECTORY}/code-examples-bin")

set(PUGIXML_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/externals/pugixml")
set(EXPR_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/externals/expr")

include_directories( ${PUGIXML_INCLUDE_DIR} ${EXPR_INCLUDE_DIR})

set(USE_OSG True CACHE BOOL "If projects that depend on OpenSceneGraph should be compiled.")
set(USE_OSI True CACHE BOOL "If code that depend on OSI should be compiled.")
set(USE_SUMO True CACHE BOOL "If code that depend on SUMO should be compiled.")
set(USE_GTEST True CACHE BOOL "If unit test suites based on googletest should be compiled.")
set(DYN_PROTOBUF False CACHE BOOL "Set for dynamic linking of protobuf library (.so/.dll)")


if (USE_OSG)
  add_definitions(-D_USE_OSG)
endif (USE_OSG)

if (USE_OSI)
  add_definitions(-D_USE_OSI)
endif (USE_OSI)

if (USE_SUMO)
  add_definitions(-D_USE_SUMO)
endif (USE_SUMO)

if (USE_GTEST)
  add_definitions(-D_USE_GTEST)
endif (USE_GTEST)

if (DYN_PROTOBUF)
  add_definitions(-D_DYN_PROTOBUF)
  add_definitions(-DPROTOBUF_USE_DLLS)
endif (DYN_PROTOBUF)

add_subdirectory(EnvironmentSimulator)

if( NOT EXISTS "test/OSC-ALKS-scenarios/.git" )
  execute_process(COMMAND git submodule update --init WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

